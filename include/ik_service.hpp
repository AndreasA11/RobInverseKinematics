#ifndef PROJECT4_IK_SERVICE_HPP
#define PROJECT4_IK_SERVICE_HPP

#include <cstddef>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
#include "math3d.hpp"
#include "middleware/middleware.hpp"
#include "robot_model.hpp"

namespace ik {

// *CHANGES*: Shared kinematic chain info extracted from root to end effector.
struct ChainInfo {
    std::size_t ee_link_idx{0};
    std::vector<std::size_t> movable_joint_indices;
    std::vector<std::string> movable_joint_names;
    std::unordered_set<std::string> movable_joint_set;
};

// *CHANGES*: Internal Jacobian and end-effector pose structure for step and solve calculations.
struct JacobianResult {
    std::vector<math3d::Vector3> J_v;
    std::vector<math3d::Vector3> J_omega;
    math3d::Vector3 p_e{0.0, 0.0, 0.0};
    math3d::Matrix4 ee_pose;
};

class IKService {
public:
    explicit IKService(middleware::Middleware& middleware);
    ~IKService();

    IKService(const IKService&) = delete;
    IKService& operator=(const IKService&) = delete;

private:
    middleware::Middleware& middleware_;
    std::mutex mutex_;

    // Cached robot description and parsed model
    std::size_t last_processed_version_{0};
    bool is_valid_{false};
    std::string current_error_;
    robot::RobotModel current_model_;

    // Service handlers
    std::pair<bool, middleware::JsonValue> handle_robot_jacobian(const middleware::JsonValue& args);
    // *CHANGES*: Add /ik/solve_robot_step and /ik/solve_robot handlers for Project 4 final requirements.
    std::pair<bool, middleware::JsonValue> handle_solve_robot_step(const middleware::JsonValue& args);
    std::pair<bool, middleware::JsonValue> handle_solve_robot(const middleware::JsonValue& args);

    // Helpers
    bool refresh_robot_model();
    static middleware::JsonValue make_error(std::size_t version, const std::string& status);

    bool extract_chain(const std::string& ee_frame, ChainInfo& out, std::string& error_msg);
    bool parse_joint_state(const middleware::JsonValue& js,
                           const ChainInfo& chain,
                           std::unordered_map<std::string, double>& config_out,
                           std::string& error_msg);
    JacobianResult compute_jacobian_and_fk(
        const ChainInfo& chain,
        const std::unordered_map<std::string, double>& config);
    std::vector<double> compute_step_delta(
        const ChainInfo& chain,
        const JacobianResult& jac,
        const math3d::Vector3& p_target,
        const math3d::Quaternion& q_target,
        bool position_only,
        const std::string& method,
        double step_length,
        double damping);
};

} // namespace ik

#endif // PROJECT4_IK_SERVICE_HPP
