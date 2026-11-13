#ifndef PROJECT4_IK_SERVICE_HPP
#define PROJECT4_IK_SERVICE_HPP

#include <cstddef>
#include <mutex>
#include <string>
#include <unordered_map>
#include "middleware/middleware.hpp"
#include "robot_model.hpp"

namespace ik {

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

    // Helpers
    std::pair<bool, middleware::JsonValue> handle_robot_jacobian(const middleware::JsonValue& args);
    bool refresh_robot_model();
    static middleware::JsonValue make_error(std::size_t version, const std::string& status);
};

} // namespace ik

#endif // PROJECT4_IK_SERVICE_HPP
