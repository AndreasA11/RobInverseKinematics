#include "ik_service.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_set>
#include <utility>
#include <vector>
#include "forward_kinematics.hpp"
#include "math3d.hpp"

namespace ik {

// *CHANGES*: Initialize IKService and register /ik/robot_jacobian service on middleware.
IKService::IKService(middleware::Middleware& middleware) : middleware_(middleware) {
    middleware_.advertise_service("/ik/robot_jacobian", [this](const middleware::JsonValue& args) {
        return handle_robot_jacobian(args);
    });
}

// *CHANGES*: Cleanly unadvertise IK services on destruction.
IKService::~IKService() {
    middleware_.unadvertise_service("/ik/robot_jacobian");
}

middleware::JsonValue IKService::make_error(std::size_t version, const std::string& status) {
    middleware::JsonValue values = middleware::JsonValue::make_object();
    values["description_version"] = middleware::JsonValue(version);
    values["__status__"] = status.empty() ? "error" : status;
    return values;
}

// *CHANGES*: Fetch robot_description from param_server and cache parsed RobotModel by version.
bool IKService::refresh_robot_model() {
    middleware::JsonValue args = middleware::JsonValue::make_object();
    args["name"] = "robot_description";
    auto reply = middleware_.call_service("/param_server/get_param", args);
    if (!reply.first || !reply.second.is_object() ||
        !reply.second.contains("version") || !reply.second["version"].is_number() ||
        reply.second["version"].as_number() < 1.0) {
        return false;
    }

    std::size_t version = static_cast<std::size_t>(reply.second["version"].as_number());
    if (version == last_processed_version_) {
        return is_valid_;
    }

    last_processed_version_ = version;
    if (!reply.second.contains("value") || !reply.second["value"].is_string()) {
        is_valid_ = false;
        current_error_ = "robot_description parameter must be a JSON string";
        return false;
    }

    const std::string& urdf_text = reply.second["value"].as_string();
    robot::ParseResult parsed = robot::parse_urdf(urdf_text);
    if (!parsed) {
        is_valid_ = false;
        current_error_ = parsed.error.empty() ? "invalid robot description" : parsed.error;
        return false;
    }

    current_model_ = std::move(*parsed.model);
    is_valid_ = true;
    current_error_.clear();
    return true;
}

// *CHANGES*: Implement /ik/robot_jacobian manipulator Jacobian calculation according to spec.
std::pair<bool, middleware::JsonValue> IKService::handle_robot_jacobian(const middleware::JsonValue& args) {
    std::lock_guard<std::mutex> lock(mutex_);

    bool has_robot = refresh_robot_model();
    if (!has_robot) {
        if (last_processed_version_ == 0) {
            return {false, make_error(0, "no robot description loaded")};
        }
        return {false, make_error(last_processed_version_,
                                  current_error_.empty() ? "invalid robot description" : current_error_)};
    }

    if (!args.is_object() || !args.contains("endeffector_frame") || !args["endeffector_frame"].is_string()) {
        return {false, make_error(last_processed_version_, "missing or invalid endeffector_frame")};
    }
    const std::string& ee_frame = args["endeffector_frame"].as_string();
    auto link_it = current_model_.link_index.find(ee_frame);
    if (link_it == current_model_.link_index.end()) {
        return {false, make_error(last_processed_version_, "unknown endeffector_frame: " + ee_frame)};
    }
    std::size_t ee_link_idx = link_it->second;

    if (!args.contains("joint_state") || !args["joint_state"].is_object()) {
        return {false, make_error(last_processed_version_, "missing or invalid joint_state")};
    }
    const auto& js = args["joint_state"];
    if (!js.contains("name") || !js["name"].is_array() ||
        !js.contains("position") || !js["position"].is_array()) {
        return {false, make_error(last_processed_version_, "joint_state missing name or position array")};
    }
    const auto& names = js["name"].as_array();
    const auto& positions = js["position"].as_array();
    if (names.size() != positions.size()) {
        return {false, make_error(last_processed_version_, "joint_state name and position lengths mismatch")};
    }

    // Build parent-joint lookup for each link
    std::vector<std::size_t> parent_joint(current_model_.links.size(), static_cast<std::size_t>(-1));
    for (std::size_t j = 0; j < current_model_.joints.size(); ++j) {
        parent_joint[current_model_.joints[j].child_index] = j;
    }

    // Extract chain of joints from root to end-effector link
    std::vector<std::size_t> chain_joints;
    std::size_t curr = ee_link_idx;
    while (curr != current_model_.root_index) {
        std::size_t pj = parent_joint[curr];
        if (pj == static_cast<std::size_t>(-1)) {
            break;
        }
        chain_joints.push_back(pj);
        curr = current_model_.joints[pj].parent_index;
    }
    std::reverse(chain_joints.begin(), chain_joints.end());

    // Extract movable joints in root-to-end-effector order
    std::vector<std::size_t> chain_movable_joints;
    std::vector<std::string> joint_names;
    std::unordered_set<std::string> chain_movable_names;
    for (std::size_t j : chain_joints) {
        const auto& joint = current_model_.joints[j];
        if (joint.type == robot::JointType::Revolute ||
            joint.type == robot::JointType::Continuous ||
            joint.type == robot::JointType::Prismatic) {
            chain_movable_joints.push_back(j);
            joint_names.push_back(joint.name);
            chain_movable_names.insert(joint.name);
        }
    }

    // Validate joint positions for chain movable joints and build configuration map
    std::unordered_map<std::string, double> config;
    for (std::size_t i = 0; i < names.size(); ++i) {
        if (!names[i].is_string()) continue;
        const std::string& name_str = names[i].as_string();
        if (chain_movable_names.count(name_str)) {
            if (positions[i].is_bool() || !positions[i].is_number()) {
                return {false, make_error(last_processed_version_,
                                          "position for chain joint '" + name_str + "' must be a number")};
            }
            double val = positions[i].as_number();
            if (!std::isfinite(val)) {
                return {false, make_error(last_processed_version_,
                                          "position for chain joint '" + name_str + "' is not finite")};
            }
            config[name_str] = val;
        } else {
            if (!positions[i].is_bool() && positions[i].is_number() && std::isfinite(positions[i].as_number())) {
                config[name_str] = positions[i].as_number();
            }
        }
    }

    // Compute Forward Kinematics at the requested configuration
    robot::FKResult fk = robot::compute_fk(current_model_, config);
    const math3d::Matrix4& ee_pose = fk.link_poses[ee_link_idx];
    math3d::Vector3 p_e = math3d::translation_of(ee_pose);

    // Compute Jacobian rows
    middleware::JsonValue jacobian_json = middleware::JsonValue::make_array();
    for (std::size_t j_idx : chain_movable_joints) {
        const robot::Joint& joint = current_model_.joints[j_idx];
        math3d::Matrix4 origin_transform = math3d::from_origin(joint.origin.xyz, joint.origin.rpy);
        math3d::Matrix4 joint_frame = math3d::multiply(fk.link_poses[joint.parent_index], origin_transform);
        math3d::Vector3 p_i = math3d::translation_of(joint_frame);

        math3d::Vector3 v{0.0, 0.0, 0.0};
        math3d::Vector3 omega{0.0, 0.0, 0.0};

        if (joint.type == robot::JointType::Revolute || joint.type == robot::JointType::Continuous) {
            double a_norm = math3d::norm(joint.axis);
            math3d::Vector3 unit_a = (a_norm > 0.0) ? math3d::scale(joint.axis, 1.0 / a_norm) : joint.axis;
            math3d::Vector3 z_hat = math3d::rotate_vector(joint_frame, unit_a);
            math3d::Vector3 r = math3d::subtract(p_e, p_i);
            v = math3d::cross(z_hat, r);
            omega = z_hat;
        } else if (joint.type == robot::JointType::Prismatic) {
            // Prismatic joint uses axis as written (unnormalized)
            v = math3d::rotate_vector(joint_frame, joint.axis);
            omega = {0.0, 0.0, 0.0};
        }

        middleware::JsonValue row = middleware::JsonValue::make_array();
        row.push_back(middleware::JsonValue(v[0]));
        row.push_back(middleware::JsonValue(v[1]));
        row.push_back(middleware::JsonValue(v[2]));
        row.push_back(middleware::JsonValue(omega[0]));
        row.push_back(middleware::JsonValue(omega[1]));
        row.push_back(middleware::JsonValue(omega[2]));
        jacobian_json.push_back(std::move(row));
    }

    middleware::JsonValue values = middleware::JsonValue::make_object();
    middleware::JsonValue names_json = middleware::JsonValue::make_array();
    for (const auto& name : joint_names) {
        names_json.push_back(middleware::JsonValue(name));
    }
    values["joint_names"] = std::move(names_json);
    values["jacobian"] = std::move(jacobian_json);
    values["description_version"] = middleware::JsonValue(last_processed_version_);
    values["endeffector_frame"] = middleware::JsonValue(ee_frame);
    values["__status__"] = "";

    return {true, values};
}

} // namespace ik
