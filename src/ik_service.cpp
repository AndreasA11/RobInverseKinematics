#include "ik_service.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <unordered_set>
#include <utility>
#include <vector>
#include "forward_kinematics.hpp"
#include "math3d.hpp"

namespace ik {

// *CHANGES*: Initialize IKService and register /ik/robot_jacobian, /ik/solve_robot_step, and /ik/solve_robot services.
IKService::IKService(middleware::Middleware& middleware) : middleware_(middleware) {
    middleware_.advertise_service("/ik/robot_jacobian", [this](const middleware::JsonValue& args) {
        return handle_robot_jacobian(args);
    });
    middleware_.advertise_service("/ik/solve_robot_step", [this](const middleware::JsonValue& args) {
        return handle_solve_robot_step(args);
    });
    middleware_.advertise_service("/ik/solve_robot", [this](const middleware::JsonValue& args) {
        return handle_solve_robot(args);
    });
}

// *CHANGES*: Cleanly unadvertise all IK services on destruction.
IKService::~IKService() {
    middleware_.unadvertise_service("/ik/robot_jacobian");
    middleware_.unadvertise_service("/ik/solve_robot_step");
    middleware_.unadvertise_service("/ik/solve_robot");
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

// *CHANGES*: Trace kinematic chain from root to endeffector_frame and extract movable joints.
bool IKService::extract_chain(const std::string& ee_frame, ChainInfo& out, std::string& error_msg) {
    auto link_it = current_model_.link_index.find(ee_frame);
    if (link_it == current_model_.link_index.end()) {
        error_msg = "unknown endeffector_frame: " + ee_frame;
        return false;
    }
    out.ee_link_idx = link_it->second;

    std::vector<std::size_t> parent_joint(current_model_.links.size(), static_cast<std::size_t>(-1));
    for (std::size_t j = 0; j < current_model_.joints.size(); ++j) {
        parent_joint[current_model_.joints[j].child_index] = j;
    }

    std::vector<std::size_t> chain_joints;
    std::size_t curr = out.ee_link_idx;
    while (curr != current_model_.root_index) {
        std::size_t pj = parent_joint[curr];
        if (pj == static_cast<std::size_t>(-1)) {
            break;
        }
        chain_joints.push_back(pj);
        curr = current_model_.joints[pj].parent_index;
    }
    std::reverse(chain_joints.begin(), chain_joints.end());

    out.movable_joint_indices.clear();
    out.movable_joint_names.clear();
    out.movable_joint_set.clear();

    for (std::size_t j : chain_joints) {
        const auto& joint = current_model_.joints[j];
        if (joint.type == robot::JointType::Revolute ||
            joint.type == robot::JointType::Continuous ||
            joint.type == robot::JointType::Prismatic) {
            out.movable_joint_indices.push_back(j);
            out.movable_joint_names.push_back(joint.name);
            out.movable_joint_set.insert(joint.name);
        }
    }
    return true;
}

// *CHANGES*: Parse joint_state input, ensuring valid numbers for chain movable joints and default 0.0 for omitted ones.
bool IKService::parse_joint_state(const middleware::JsonValue& js,
                                  const ChainInfo& chain,
                                  std::unordered_map<std::string, double>& config_out,
                                  std::string& error_msg) {
    if (!js.is_object() || !js.contains("name") || !js["name"].is_array() ||
        !js.contains("position") || !js["position"].is_array()) {
        error_msg = "missing or invalid joint_state (name and position arrays required)";
        return false;
    }
    const auto& names = js["name"].as_array();
    const auto& positions = js["position"].as_array();
    if (names.size() != positions.size()) {
        error_msg = "joint_state name and position lengths mismatch";
        return false;
    }

    config_out.clear();
    for (std::size_t i = 0; i < names.size(); ++i) {
        if (!names[i].is_string()) continue;
        const std::string& name_str = names[i].as_string();
        if (chain.movable_joint_set.count(name_str)) {
            if (positions[i].is_bool() || !positions[i].is_number()) {
                error_msg = "position for chain joint '" + name_str + "' must be a number";
                return false;
            }
            double val = positions[i].as_number();
            if (!std::isfinite(val)) {
                error_msg = "position for chain joint '" + name_str + "' is not finite";
                return false;
            }
            config_out[name_str] = val;
        } else {
            if (!positions[i].is_bool() && positions[i].is_number() && std::isfinite(positions[i].as_number())) {
                config_out[name_str] = positions[i].as_number();
            }
        }
    }

    // Default missing chain movable joints to 0.0
    for (const auto& name : chain.movable_joint_names) {
        if (config_out.find(name) == config_out.end()) {
            config_out[name] = 0.0;
        }
    }
    return true;
}

// *CHANGES*: Compute forward kinematics and Jacobian linear/angular velocity blocks in root-link frame.
JacobianResult IKService::compute_jacobian_and_fk(
    const ChainInfo& chain,
    const std::unordered_map<std::string, double>& config) {
    robot::FKResult fk = robot::compute_fk(current_model_, config);
    JacobianResult result;
    result.ee_pose = fk.link_poses[chain.ee_link_idx];
    result.p_e = math3d::translation_of(result.ee_pose);

    for (std::size_t j_idx : chain.movable_joint_indices) {
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
            math3d::Vector3 r = math3d::subtract(result.p_e, p_i);
            v = math3d::cross(z_hat, r);
            omega = z_hat;
        } else if (joint.type == robot::JointType::Prismatic) {
            // Prismatic joint uses axis as written (unnormalized)
            v = math3d::rotate_vector(joint_frame, joint.axis);
            omega = {0.0, 0.0, 0.0};
        }

        result.J_v.push_back(v);
        result.J_omega.push_back(omega);
    }
    return result;
}

// *CHANGES*: Compute one gradient step delta_q using Jacobian transpose or damped pseudoinverse.
std::vector<double> IKService::compute_step_delta(
    const ChainInfo& chain,
    const JacobianResult& jac,
    const math3d::Vector3& p_target,
    const math3d::Quaternion& q_target,
    bool position_only,
    const std::string& method,
    double step_length,
    double damping) {
    std::size_t n = chain.movable_joint_indices.size();
    std::vector<double> dq(n, 0.0);
    if (n == 0) return dq;

    math3d::Vector3 pos_err = math3d::subtract(p_target, jac.p_e);

    if (position_only) {
        if (method == "JacobianTranspose") {
            for (std::size_t k = 0; k < n; ++k) {
                dq[k] = step_length * math3d::dot(jac.J_v[k], pos_err);
            }
        } else {
            // JacobianPseudoinverse (Damped Least Squares with lambda >= 0)
            // A = J_v * J_v^T + lambda^2 * I_3
            std::array<std::array<double, 3>, 3> A{};
            double lambda_sq = damping * damping;
            for (std::size_t r = 0; r < 3; ++r) {
                for (std::size_t c = 0; c < 3; ++c) {
                    double sum = (r == c ? lambda_sq : 0.0);
                    for (std::size_t k = 0; k < n; ++k) {
                        sum += jac.J_v[k][r] * jac.J_v[k][c];
                    }
                    A[r][c] = sum;
                }
            }
            std::array<double, 3> b{pos_err[0], pos_err[1], pos_err[2]};
            std::array<double, 3> y{0.0, 0.0, 0.0};
            if (!math3d::solve_linear_system<3>(A, b, y)) {
                // In singular cases, add small damping regularization to avoid dividing by zero
                for (std::size_t i = 0; i < 3; ++i) A[i][i] += 1e-6;
                if (!math3d::solve_linear_system<3>(A, b, y)) {
                    std::fill(y.begin(), y.end(), 0.0);
                }
            }
            for (std::size_t k = 0; k < n; ++k) {
                dq[k] = step_length * math3d::dot(jac.J_v[k], y);
            }
        }
    } else {
        // Full pose: 6-vector error [pos_err; omega_err]
        math3d::Matrix3 r_target = math3d::from_quaternion(q_target);
        math3d::Matrix3 r_e = math3d::rotation_matrix_of(jac.ee_pose);
        math3d::Matrix3 r_diff = math3d::multiply(r_target, math3d::transpose(r_e));
        math3d::Vector3 omega_err = math3d::rotation_vector(r_diff);

        if (method == "JacobianTranspose") {
            for (std::size_t k = 0; k < n; ++k) {
                double v_part = math3d::dot(jac.J_v[k], pos_err);
                double w_part = math3d::dot(jac.J_omega[k], omega_err);
                dq[k] = step_length * (v_part + w_part);
            }
        } else {
            // JacobianPseudoinverse (Damped Least Squares with lambda >= 0)
            // A = J * J^T + lambda^2 * I_6
            std::array<std::array<double, 6>, 6> A{};
            double lambda_sq = damping * damping;
            for (std::size_t r = 0; r < 6; ++r) {
                for (std::size_t c = 0; c < 6; ++c) {
                    double sum = (r == c ? lambda_sq : 0.0);
                    for (std::size_t k = 0; k < n; ++k) {
                        double j_rk = (r < 3) ? jac.J_v[k][r] : jac.J_omega[k][r - 3];
                        double j_ck = (c < 3) ? jac.J_v[k][c] : jac.J_omega[k][c - 3];
                        sum += j_rk * j_ck;
                    }
                    A[r][c] = sum;
                }
            }
            std::array<double, 6> b{
                pos_err[0], pos_err[1], pos_err[2],
                omega_err[0], omega_err[1], omega_err[2]
            };
            std::array<double, 6> y{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            if (!math3d::solve_linear_system<6>(A, b, y)) {
                for (std::size_t i = 0; i < 6; ++i) A[i][i] += 1e-6;
                if (!math3d::solve_linear_system<6>(A, b, y)) {
                    std::fill(y.begin(), y.end(), 0.0);
                }
            }
            for (std::size_t k = 0; k < n; ++k) {
                double v_part = jac.J_v[k][0] * y[0] + jac.J_v[k][1] * y[1] + jac.J_v[k][2] * y[2];
                double w_part = jac.J_omega[k][0] * y[3] + jac.J_omega[k][1] * y[4] + jac.J_omega[k][2] * y[5];
                dq[k] = step_length * (v_part + w_part);
            }
        }
    }
    return dq;
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

    ChainInfo chain;
    std::string err;
    if (!extract_chain(ee_frame, chain, err)) {
        return {false, make_error(last_processed_version_, err)};
    }

    if (!args.contains("joint_state")) {
        return {false, make_error(last_processed_version_, "missing joint_state")};
    }
    std::unordered_map<std::string, double> config;
    if (!parse_joint_state(args["joint_state"], chain, config, err)) {
        return {false, make_error(last_processed_version_, err)};
    }

    JacobianResult jac = compute_jacobian_and_fk(chain, config);

    middleware::JsonValue jacobian_json = middleware::JsonValue::make_array();
    for (std::size_t k = 0; k < chain.movable_joint_indices.size(); ++k) {
        middleware::JsonValue row = middleware::JsonValue::make_array();
        row.push_back(middleware::JsonValue(jac.J_v[k][0]));
        row.push_back(middleware::JsonValue(jac.J_v[k][1]));
        row.push_back(middleware::JsonValue(jac.J_v[k][2]));
        row.push_back(middleware::JsonValue(jac.J_omega[k][0]));
        row.push_back(middleware::JsonValue(jac.J_omega[k][1]));
        row.push_back(middleware::JsonValue(jac.J_omega[k][2]));
        jacobian_json.push_back(std::move(row));
    }

    middleware::JsonValue values = middleware::JsonValue::make_object();
    middleware::JsonValue names_json = middleware::JsonValue::make_array();
    for (const auto& name : chain.movable_joint_names) {
        names_json.push_back(middleware::JsonValue(name));
    }
    values["joint_names"] = std::move(names_json);
    values["jacobian"] = std::move(jacobian_json);
    values["description_version"] = middleware::JsonValue(last_processed_version_);
    values["endeffector_frame"] = middleware::JsonValue(ee_frame);
    values["__status__"] = "";

    return {true, values};
}

// *CHANGES*: Implement /ik/solve_robot_step one gradient-descent step according to spec.
std::pair<bool, middleware::JsonValue> IKService::handle_solve_robot_step(const middleware::JsonValue& args) {
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

    ChainInfo chain;
    std::string err;
    if (!extract_chain(ee_frame, chain, err)) {
        return {false, make_error(last_processed_version_, err)};
    }

    if (!args.contains("joint_state")) {
        return {false, make_error(last_processed_version_, "missing joint_state")};
    }
    std::unordered_map<std::string, double> config;
    if (!parse_joint_state(args["joint_state"], chain, config, err)) {
        return {false, make_error(last_processed_version_, err)};
    }

    if (!args.contains("endeffector_pose") || !args["endeffector_pose"].is_object()) {
        return {false, make_error(last_processed_version_, "missing endeffector_pose")};
    }
    const auto& ee_pose_json = args["endeffector_pose"];
    if (!ee_pose_json.contains("position") || !ee_pose_json["position"].is_object()) {
        return {false, make_error(last_processed_version_, "endeffector_pose missing position")};
    }
    const auto& pos_json = ee_pose_json["position"];
    if (!pos_json.contains("x") || !pos_json["x"].is_number() ||
        !pos_json.contains("y") || !pos_json["y"].is_number() ||
        !pos_json.contains("z") || !pos_json["z"].is_number()) {
        return {false, make_error(last_processed_version_, "position coordinates must be numbers")};
    }
    math3d::Vector3 p_target{
        pos_json["x"].as_number(),
        pos_json["y"].as_number(),
        pos_json["z"].as_number()
    };

    bool ik_position_only = true;
    if (args.contains("ik_position_only") && args["ik_position_only"].is_bool()) {
        ik_position_only = args["ik_position_only"].as_bool();
    }

    math3d::Quaternion q_target{};
    if (!ik_position_only) {
        if (!ee_pose_json.contains("orientation") || !ee_pose_json["orientation"].is_object()) {
            return {false, make_error(last_processed_version_, "endeffector_pose missing orientation")};
        }
        const auto& orient_json = ee_pose_json["orientation"];
        if (!orient_json.contains("x") || !orient_json["x"].is_number() ||
            !orient_json.contains("y") || !orient_json["y"].is_number() ||
            !orient_json.contains("z") || !orient_json["z"].is_number() ||
            !orient_json.contains("w") || !orient_json["w"].is_number()) {
            return {false, make_error(last_processed_version_, "orientation quaternion components must be numbers")};
        }
        q_target = {
            orient_json["x"].as_number(),
            orient_json["y"].as_number(),
            orient_json["z"].as_number(),
            orient_json["w"].as_number()
        };
    }

    std::string method = "JacobianTranspose";
    if (args.contains("method") && args["method"].is_string()) {
        method = args["method"].as_string();
    }

    double step_length = 0.1;
    if (args.contains("step_length") && args["step_length"].is_number() && !args["step_length"].is_bool()) {
        step_length = args["step_length"].as_number();
    }

    double damping = 0.0;
    if (args.contains("damping") && args["damping"].is_number() && !args["damping"].is_bool()) {
        damping = args["damping"].as_number();
    }

    JacobianResult jac = compute_jacobian_and_fk(chain, config);
    std::vector<double> dq = compute_step_delta(
        chain, jac, p_target, q_target, ik_position_only, method, step_length, damping);

    middleware::JsonValue config_sol = middleware::JsonValue::make_object();
    middleware::JsonValue sol_names = middleware::JsonValue::make_array();
    middleware::JsonValue sol_positions = middleware::JsonValue::make_array();
    for (std::size_t k = 0; k < chain.movable_joint_names.size(); ++k) {
        sol_names.push_back(middleware::JsonValue(chain.movable_joint_names[k]));
        double q_current = config[chain.movable_joint_names[k]];
        sol_positions.push_back(middleware::JsonValue(q_current + dq[k]));
    }
    config_sol["name"] = std::move(sol_names);
    config_sol["position"] = std::move(sol_positions);

    middleware::JsonValue values = middleware::JsonValue::make_object();
    values["configuration_solution"] = std::move(config_sol);
    values["description_version"] = middleware::JsonValue(last_processed_version_);
    values["__status__"] = "";

    return {true, values};
}

// *CHANGES*: Implement /ik/solve_robot iterative inverse kinematics solver according to spec.
std::pair<bool, middleware::JsonValue> IKService::handle_solve_robot(const middleware::JsonValue& args) {
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

    ChainInfo chain;
    std::string err;
    if (!extract_chain(ee_frame, chain, err)) {
        return {false, make_error(last_processed_version_, err)};
    }

    if (!args.contains("joint_state")) {
        return {false, make_error(last_processed_version_, "missing joint_state")};
    }
    std::unordered_map<std::string, double> config;
    if (!parse_joint_state(args["joint_state"], chain, config, err)) {
        return {false, make_error(last_processed_version_, err)};
    }

    if (!args.contains("endeffector_pose") || !args["endeffector_pose"].is_object()) {
        return {false, make_error(last_processed_version_, "missing endeffector_pose")};
    }
    const auto& ee_pose_json = args["endeffector_pose"];
    if (!ee_pose_json.contains("position") || !ee_pose_json["position"].is_object()) {
        return {false, make_error(last_processed_version_, "endeffector_pose missing position")};
    }
    const auto& pos_json = ee_pose_json["position"];
    if (!pos_json.contains("x") || !pos_json["x"].is_number() ||
        !pos_json.contains("y") || !pos_json["y"].is_number() ||
        !pos_json.contains("z") || !pos_json["z"].is_number()) {
        return {false, make_error(last_processed_version_, "position coordinates must be numbers")};
    }
    math3d::Vector3 p_target{
        pos_json["x"].as_number(),
        pos_json["y"].as_number(),
        pos_json["z"].as_number()
    };

    bool ik_position_only = true;
    if (args.contains("ik_position_only") && args["ik_position_only"].is_bool()) {
        ik_position_only = args["ik_position_only"].as_bool();
    }

    math3d::Quaternion q_target{};
    if (!ik_position_only) {
        if (!ee_pose_json.contains("orientation") || !ee_pose_json["orientation"].is_object()) {
            return {false, make_error(last_processed_version_, "endeffector_pose missing orientation")};
        }
        const auto& orient_json = ee_pose_json["orientation"];
        if (!orient_json.contains("x") || !orient_json["x"].is_number() ||
            !orient_json.contains("y") || !orient_json["y"].is_number() ||
            !orient_json.contains("z") || !orient_json["z"].is_number() ||
            !orient_json.contains("w") || !orient_json["w"].is_number()) {
            return {false, make_error(last_processed_version_, "orientation quaternion components must be numbers")};
        }
        q_target = {
            orient_json["x"].as_number(),
            orient_json["y"].as_number(),
            orient_json["z"].as_number(),
            orient_json["w"].as_number()
        };
    }

    std::string method = "JacobianTranspose";
    if (args.contains("method") && args["method"].is_string()) {
        method = args["method"].as_string();
    }

    double step_length = 0.1;
    if (args.contains("step_length") && args["step_length"].is_number() && !args["step_length"].is_bool()) {
        step_length = args["step_length"].as_number();
    }

    double damping = 0.0;
    if (args.contains("damping") && args["damping"].is_number() && !args["damping"].is_bool()) {
        damping = args["damping"].as_number();
    }

    if (!args.contains("position_tolerance") || !args["position_tolerance"].is_number() ||
        args["position_tolerance"].is_bool() || args["position_tolerance"].as_number() <= 0.0) {
        return {false, make_error(last_processed_version_, "position_tolerance must be a number > 0")};
    }
    double pos_tol = args["position_tolerance"].as_number();

    double orient_tol = 0.0;
    if (!ik_position_only) {
        if (!args.contains("orientation_tolerance") || !args["orientation_tolerance"].is_number() ||
            args["orientation_tolerance"].is_bool() || args["orientation_tolerance"].as_number() <= 0.0) {
            return {false, make_error(last_processed_version_, "orientation_tolerance must be a number > 0")};
        }
        orient_tol = args["orientation_tolerance"].as_number();
    }

    if (!args.contains("max_iterations") || !args["max_iterations"].is_number() ||
        args["max_iterations"].is_bool() || args["max_iterations"].as_number() < 1.0) {
        return {false, make_error(last_processed_version_, "max_iterations must be an integer >= 1")};
    }
    std::size_t max_iterations = static_cast<std::size_t>(args["max_iterations"].as_number());

    // Joint positions vector for chain movable joints
    std::vector<double> q(chain.movable_joint_indices.size(), 0.0);
    for (std::size_t k = 0; k < chain.movable_joint_names.size(); ++k) {
        q[k] = config[chain.movable_joint_names[k]];
    }

    std::size_t iterations = 0;
    bool converged = false;
    double final_pos_err = 0.0;
    double final_orient_err = 0.0;

    math3d::Matrix3 r_target_mat = ik_position_only ? math3d::Matrix3{} : math3d::from_quaternion(q_target);

    while (true) {
        JacobianResult jac = compute_jacobian_and_fk(chain, config);
        math3d::Vector3 pos_diff = math3d::subtract(p_target, jac.p_e);
        final_pos_err = math3d::norm(pos_diff);

        if (!ik_position_only) {
            math3d::Matrix3 r_e = math3d::rotation_matrix_of(jac.ee_pose);
            math3d::Matrix3 r_diff = math3d::multiply(math3d::transpose(r_target_mat), r_e);
            final_orient_err = math3d::rotation_angle(r_diff);
        }

        bool pos_ok = (final_pos_err < pos_tol);
        bool orient_ok = ik_position_only || (final_orient_err < orient_tol);
        if (pos_ok && orient_ok) {
            converged = true;
            break;
        }

        if (iterations >= max_iterations) {
            converged = false;
            break;
        }

        std::vector<double> dq = compute_step_delta(
            chain, jac, p_target, q_target, ik_position_only, method, step_length, damping);

        for (std::size_t k = 0; k < chain.movable_joint_indices.size(); ++k) {
            q[k] += dq[k];
            const auto& joint = current_model_.joints[chain.movable_joint_indices[k]];
            if (joint.type == robot::JointType::Revolute || joint.type == robot::JointType::Prismatic) {
                if (joint.has_limit) {
                    q[k] = std::clamp(q[k], joint.lower_limit, joint.upper_limit);
                }
            }
            config[joint.name] = q[k];
        }

        iterations++;
    }

    middleware::JsonValue values = middleware::JsonValue::make_object();

    middleware::JsonValue config_sol = middleware::JsonValue::make_object();
    middleware::JsonValue sol_names = middleware::JsonValue::make_array();
    middleware::JsonValue sol_positions = middleware::JsonValue::make_array();
    for (std::size_t k = 0; k < chain.movable_joint_names.size(); ++k) {
        sol_names.push_back(middleware::JsonValue(chain.movable_joint_names[k]));
        sol_positions.push_back(middleware::JsonValue(q[k]));
    }
    config_sol["name"] = std::move(sol_names);
    config_sol["position"] = std::move(sol_positions);
    values["configuration_solution"] = std::move(config_sol);

    values["converged"] = middleware::JsonValue(converged);

    middleware::JsonValue final_err = middleware::JsonValue::make_object();
    final_err["position"] = middleware::JsonValue(final_pos_err);
    if (ik_position_only) {
        final_err["orientation"] = middleware::JsonValue();
    } else {
        final_err["orientation"] = middleware::JsonValue(final_orient_err);
    }
    values["final_error"] = std::move(final_err);

    values["iterations"] = middleware::JsonValue(static_cast<double>(iterations));
    values["description_version"] = middleware::JsonValue(last_processed_version_);
    values["__status__"] = converged ? "converged" : "did not converge";

    return {converged, values};
}

} // namespace ik
