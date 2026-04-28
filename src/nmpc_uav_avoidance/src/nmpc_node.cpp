/**
 * @file nmpc_node.cpp
 * @brief C++ NMPC UAV collision avoidance — dynamic obstacle version.
 *
 * Parameter layout (138 total):
 *   [0:8]     x0
 *   [8:11]    u_prev
 *   [11:131]  p_ref × 40 timesteps (3×40 = 120)
 *   [131:134] p_obs   ← obstacle position (current)
 *   [134:137] v_obs   ← obstacle velocity  (NEW)
 *   [137]     r_obs
 */

#include "nmpc_uav_avoidance/nmpc_node.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <functional>

namespace nmpc_uav {

// ============================================================================
//  Free helper functions  (unchanged)
// ============================================================================

EulerAngles quaternion_to_euler(double qx, double qy, double qz, double qw) {
    EulerAngles e;
    double sinr = 2.0 * (qw * qx + qy * qz);
    double cosr = 1.0 - 2.0 * (qx * qx + qy * qy);
    e.roll = std::atan2(sinr, cosr);

    double sinp = 2.0 * (qw * qy - qz * qx);
    sinp = std::clamp(sinp, -1.0, 1.0);
    e.pitch = std::asin(sinp);

    double siny = 2.0 * (qw * qz + qx * qy);
    double cosy = 1.0 - 2.0 * (qy * qy + qz * qz);
    e.yaw = std::atan2(siny, cosy);
    return e;
}

Quaternion euler_to_quaternion(double roll, double pitch, double yaw) {
    double cr = std::cos(roll/2.0), sr = std::sin(roll/2.0);
    double cp = std::cos(pitch/2.0), sp = std::sin(pitch/2.0);
    double cy = std::cos(yaw/2.0), sy = std::sin(yaw/2.0);
    return {
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy,
        cr*cp*cy + sr*sp*sy
    };
}

std::array<double, NX> dynamics_step(const std::array<double, NX>& x,
                                     const std::array<double, NU>& u) {
    double px=x[0], py=x[1], pz=x[2];
    double vx=x[3], vy=x[4], vz=x[5];
    double phi=x[6], theta=x[7];
    double T_t=u[0], phi_ref=u[1], theta_ref=u[2];

    double ax = T_t * std::cos(phi) * std::sin(theta);
    double ay = -T_t * std::sin(phi);
    double az = T_t * std::cos(phi) * std::cos(theta) - GRAVITY;

    return {
        px + Ts*vx,
        py + Ts*vy,
        pz + Ts*vz,
        vx + Ts*(ax - AX*vx),
        vy + Ts*(ay - AY*vy),
        vz + Ts*(az - AZ*vz),
        phi   + (Ts/TAU_PHI)  *(K_PHI  *phi_ref  - phi),
        theta + (Ts/TAU_THETA)*(K_THETA*theta_ref - theta)
    };
}

// ============================================================================
//  Constructor
// ============================================================================

NMPCNode::NMPCNode() : Node("nmpc_uav_node") {
    // --- static obstacle params replaced by dynamic obstacle defaults ---
    this->declare_parameter<double>("obs_x",    3.0);
    this->declare_parameter<double>("obs_y",    0.0);
    this->declare_parameter<double>("obs_z",    1.0);
    this->declare_parameter<double>("obs_vx",   0.0);   // NEW
    this->declare_parameter<double>("obs_vy",   0.0);   // NEW
    this->declare_parameter<double>("obs_vz",   0.0);   // NEW
    this->declare_parameter<double>("r_obs",    1.0);
    this->declare_parameter<double>("thrust_max", 13.5);
    this->declare_parameter<double>("thrust_min", 5.0);
    this->declare_parameter<double>("yaw_gain",   1.0);

    obs_pos_[0] = this->get_parameter("obs_x").as_double();
    obs_pos_[1] = this->get_parameter("obs_y").as_double();
    obs_pos_[2] = this->get_parameter("obs_z").as_double();
    obs_vel_[0] = this->get_parameter("obs_vx").as_double();  // NEW
    obs_vel_[1] = this->get_parameter("obs_vy").as_double();  // NEW
    obs_vel_[2] = this->get_parameter("obs_vz").as_double();  // NEW
    r_obs_  = this->get_parameter("r_obs").as_double();
    T_max_  = this->get_parameter("thrust_max").as_double();
    T_min_  = this->get_parameter("thrust_min").as_double();
    K_psi_  = this->get_parameter("yaw_gain").as_double();

    RCLCPP_INFO(this->get_logger(), "Initializing OpEn solver (C bindings)...");
    solver_ = std::make_unique<OpEnSolver>();
    solver_->start();
    RCLCPP_INFO(this->get_logger(), "OpEn solver ready.");

    rmw_qos_profile_t qos_raw = rmw_qos_profile_sensor_data;
    auto qos_px4 = rclcpp::QoS(
        rclcpp::QoSInitialization(qos_raw.history, 1), qos_raw);

    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom_nmpc", 10,
        std::bind(&NMPCNode::odom_callback, this, std::placeholders::_1));

    // NEW: subscribe to obstacle odometry (position + twist)
    sub_obs_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/obstacle/odom", 10,
        std::bind(&NMPCNode::obs_odom_callback, this, std::placeholders::_1));

    sub_path_ = this->create_subscription<nav_msgs::msg::Path>(
        "/planned_path", 10,
        std::bind(&NMPCNode::path_callback, this, std::placeholders::_1));

    sub_status_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status", qos_px4,
        std::bind(&NMPCNode::vehicle_status_callback, this, std::placeholders::_1));

    pub_thrust_    = this->create_publisher<px4_msgs::msg::VehicleThrustSetpoint>(
        "/fmu/in/vehicle_thrust_setpoint", qos_px4);
    pub_att_       = this->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
        "/fmu/in/vehicle_attitude_setpoint_v1", qos_px4);
    pub_offboard_  = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", qos_px4);
    pub_command_   = this->create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", qos_px4);
    pub_pred_path_ = this->create_publisher<nav_msgs::msg::Path>(
        "/nmpc/predicted_path", 10);
    pub_obs_marker_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/nmpc/obstacle_marker", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(Ts * 1000)),
        std::bind(&NMPCNode::control_loop, this));

    RCLCPP_INFO(this->get_logger(),
        "NMPC node initialised (dynamic obstacle, %d params)", N_PARAMS);
}

NMPCNode::~NMPCNode() {
    RCLCPP_INFO(this->get_logger(), "Disarming and shutting down...");
    disarm();
    if (solver_) solver_->kill_solver();
}

// ============================================================================
//  Callbacks
// ============================================================================

void NMPCNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    auto& p = msg->pose.pose.position;
    auto& v = msg->twist.twist.linear;
    auto& q = msg->pose.pose.orientation;
    EulerAngles euler = quaternion_to_euler(q.x, q.y, q.z, q.w);

    state_[0] = p.x;  state_[1] = p.y;  state_[2] = p.z;
    state_[3] = v.x;  state_[4] = v.y;  state_[5] = v.z;
    state_[6] = -euler.roll;
    state_[7] = -euler.pitch;
    yaw_           = euler.yaw;
    state_received_ = true;
}

// NEW: obstacle odometry → update position and velocity
void NMPCNode::obs_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    obs_pos_[0] = msg->pose.pose.position.x;
    obs_pos_[1] = msg->pose.pose.position.y;
    obs_pos_[2] = msg->pose.pose.position.z;
    obs_vel_[0] = msg->twist.twist.linear.x;
    obs_vel_[1] = msg->twist.twist.linear.y;
    obs_vel_[2] = msg->twist.twist.linear.z;
    obs_received_ = true;
}

void NMPCNode::path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    path_points_.clear();
    for (const auto& ps : msg->poses) {
        path_points_.push_back({
            ps.pose.position.x,
            ps.pose.position.y,
            ps.pose.position.z
        });
    }
    path_idx_ = 0;
    RCLCPP_INFO(this->get_logger(),
                "Received path with %zu waypoints.", path_points_.size());
}

void NMPCNode::vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
    nav_state_    = msg->nav_state;
    arming_state_ = msg->arming_state;
}

// ============================================================================
//  PX4 Command helpers  (unchanged)
// ============================================================================

void NMPCNode::publish_vehicle_command(uint16_t command,
                                       float p1, float p2, float p3, float p4,
                                       float p5, float p6, float p7) {
    px4_msgs::msg::VehicleCommand msg;
    msg.timestamp        = px4_timestamp();
    msg.param1           = p1;  msg.param2 = p2;  msg.param3 = p3;  msg.param4 = p4;
    msg.param5           = static_cast<double>(p5);
    msg.param6           = static_cast<double>(p6);
    msg.param7           = p7;
    msg.command          = command;
    msg.target_system    = 1;   msg.target_component  = 1;
    msg.source_system    = 1;   msg.source_component  = 1;
    msg.from_external    = true;
    pub_command_->publish(msg);
}

void NMPCNode::set_offboard_mode() {
    publish_vehicle_command(176, 1.0f, 6.0f);
    RCLCPP_INFO(this->get_logger(), "Offboard mode requested.");
}
void NMPCNode::arm()    { publish_vehicle_command(400, 1.0f); }
void NMPCNode::disarm() { publish_vehicle_command(400, 0.0f); }

// ============================================================================
//  Waypoint management  (unchanged)
// ============================================================================

void NMPCNode::update_reference() {
    if (path_points_.empty()) return;

    double min_dist = 1e9;
    size_t closest  = 0;
    for (size_t i = 0; i < path_points_.size(); ++i) {
        double dx = state_[0] - path_points_[i][0];
        double dy = state_[1] - path_points_[i][1];
        double dz = state_[2] - path_points_[i][2];
        double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (dist < min_dist) { min_dist = dist; closest = i; }
    }
    path_idx_ = closest;
}

void NMPCNode::global_to_body_angles(double phi_g, double theta_g,
                                     double& phi_b, double& theta_b) const {
    double c = std::cos(yaw_), s = std::sin(yaw_);
    phi_b   =  c * phi_g + s * theta_g;
    theta_b = -s * phi_g + c * theta_g;
}

// ============================================================================
//  Main control loop (20 Hz)
// ============================================================================

void NMPCNode::control_loop() {
    if (!state_received_) return;

    publish_offboard_mode();
    offboard_counter_++;

    if (offboard_counter_ == OFFBOARD_SETPOINT_COUNT && !offboard_mode_set_) {
        set_offboard_mode();
        offboard_mode_set_ = true;
    }

    update_reference();

    // Build parameter vector (138 total)
    std::vector<double> params(N_PARAMS, 0.0);

    // [0:8] state, [8:11] u_prev
    for (int i = 0; i < NX; ++i) params[i]     = state_[i];
    for (int i = 0; i < NU; ++i) params[8 + i] = u_prev_[i];

    // [11:131] reference trajectory
    const int ref_start = 11;
    for (int j = 0; j < N; ++j) {
        size_t idx = path_idx_ + j;
        if (idx >= path_points_.size())
            idx = path_points_.empty() ? 0 : path_points_.size() - 1;
        if (!path_points_.empty()) {
            params[ref_start + j*3 + 0] = path_points_[idx][0];
            params[ref_start + j*3 + 1] = path_points_[idx][1];
            params[ref_start + j*3 + 2] = path_points_[idx][2];
        } else {
            params[ref_start + j*3 + 0] = state_[0];
            params[ref_start + j*3 + 1] = state_[1];
            params[ref_start + j*3 + 2] = state_[2];
        }
    }

    // [131:134] obstacle position (current)
    params[131] = obs_pos_[0];
    params[132] = obs_pos_[1];
    params[133] = obs_pos_[2];

    // [134:137] obstacle velocity  (NEW)
    params[134] = obs_vel_[0];
    params[135] = obs_vel_[1];
    params[136] = obs_vel_[2];

    // [137] obstacle radius
    params[137] = r_obs_;

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "obs pos=[%.2f,%.2f,%.2f] vel=[%.2f,%.2f,%.2f] | "
        "UAV pos=[%.1f,%.1f,%.1f] path_idx=%zu/%zu",
        obs_pos_[0], obs_pos_[1], obs_pos_[2],
        obs_vel_[0], obs_vel_[1], obs_vel_[2],
        state_[0], state_[1], state_[2],
        path_idx_, path_points_.size());

    OpEnResult result;
    try {
        result = solver_->solve(params);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Solver call failed: %s", e.what());
        publish_fallback_setpoint();
        return;
    }

    if (!result.ok) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Solver error: %s", result.exit_status.c_str());
        publish_fallback_setpoint();
        return;
    }

    const auto& z_star = result.solution;
    if (z_star.size() < static_cast<size_t>(NU)) {
        publish_fallback_setpoint();
        return;
    }

    double T_opt         = z_star[0];
    double phi_ref_opt   = z_star[1];
    double theta_ref_opt = z_star[2];

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "SOLVE: T=%.3f phi=%.6f theta=%.6f | %s | %.2fms",
        T_opt, phi_ref_opt, theta_ref_opt,
        result.exit_status.c_str(), result.solve_time_ms);

    u_prev_ = {T_opt, phi_ref_opt, theta_ref_opt};

    double phi_ref_b   = -phi_ref_opt;
    double theta_ref_b = -theta_ref_opt;
    double yaw_rate    = -K_psi_ * yaw_;

    double T_clamped   = std::clamp(T_opt, T_min_, T_max_);
    double thrust_norm = std::sqrt(T_clamped / T_max_);

    {
        px4_msgs::msg::VehicleThrustSetpoint msg;
        msg.timestamp = px4_timestamp();
        msg.xyz[0] = 0.0f; msg.xyz[1] = 0.0f;
        msg.xyz[2] = static_cast<float>(-thrust_norm);
        pub_thrust_->publish(msg);
    }

    {
        px4_msgs::msg::VehicleAttitudeSetpoint msg;
        msg.timestamp = px4_timestamp();
        Quaternion q  = euler_to_quaternion(phi_ref_b, theta_ref_b, 0.0);
        msg.q_d[0] = static_cast<float>(q.w);
        msg.q_d[1] = static_cast<float>(q.x);
        msg.q_d[2] = static_cast<float>(q.y);
        msg.q_d[3] = static_cast<float>(q.z);
        msg.thrust_body[0] = 0.0f; msg.thrust_body[1] = 0.0f;
        msg.thrust_body[2] = static_cast<float>(-thrust_norm);
        msg.yaw_sp_move_rate = static_cast<float>(yaw_rate);
        pub_att_->publish(msg);
    }

    publish_predicted_path(z_star);
    publish_obstacle_marker();
}

// ============================================================================
//  Fallback / helpers  (unchanged except marker)
// ============================================================================

void NMPCNode::publish_fallback_setpoint() {
    double T_clamped   = std::clamp(u_prev_[0], T_min_, T_max_);
    double thrust_norm = std::sqrt(T_clamped / T_max_);
    double phi_b, theta_b;
    global_to_body_angles(u_prev_[1], u_prev_[2], phi_b, theta_b);

    px4_msgs::msg::VehicleAttitudeSetpoint msg;
    msg.timestamp = px4_timestamp();
    Quaternion q  = euler_to_quaternion(phi_b, theta_b, 0.0);
    msg.q_d[0] = static_cast<float>(q.w); msg.q_d[1] = static_cast<float>(q.x);
    msg.q_d[2] = static_cast<float>(q.y); msg.q_d[3] = static_cast<float>(q.z);
    msg.thrust_body[2]   = static_cast<float>(-thrust_norm);
    msg.yaw_sp_move_rate = static_cast<float>(-K_psi_ * yaw_);
    pub_att_->publish(msg);
}

void NMPCNode::publish_offboard_mode() {
    px4_msgs::msg::OffboardControlMode msg;
    msg.timestamp = px4_timestamp();
    msg.attitude  = true;
    pub_offboard_->publish(msg);
}

void NMPCNode::publish_predicted_path(const std::vector<double>& z_star) {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp    = this->now();
    path_msg.header.frame_id = "map";

    std::array<double, NX> x_k = state_;
    for (int j = 0; j < N; ++j) {
        int idx = j * NU;
        if (idx + NU > static_cast<int>(z_star.size())) break;
        std::array<double, NU> uj = {z_star[idx], z_star[idx+1], z_star[idx+2]};
        x_k = dynamics_step(x_k, uj);

        geometry_msgs::msg::PoseStamped ps;
        ps.header = path_msg.header;
        ps.pose.position.x = x_k[0];
        ps.pose.position.y = x_k[1];
        ps.pose.position.z = x_k[2];
        path_msg.poses.push_back(ps);
    }
    pub_pred_path_->publish(path_msg);
}

// Marker shows obstacle at current position (real-time) 
void NMPCNode::publish_obstacle_marker() {
    visualization_msgs::msg::Marker m;
    m.header.stamp    = this->now();
    m.header.frame_id = "map";
    m.ns   = "obstacle";  m.id = 0;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x  = obs_pos_[0];
    m.pose.position.y  = obs_pos_[1];
    m.pose.position.z  = obs_pos_[2];
    m.pose.orientation.w = 1.0;
    double d = 2.0 * r_obs_;
    m.scale.x = d; m.scale.y = d; m.scale.z = d;
    m.color.r = 1.0f; m.color.g = 0.3f; m.color.b = 0.0f; m.color.a = 0.5f;
    pub_obs_marker_->publish(m);
}

uint64_t NMPCNode::px4_timestamp() {
    auto now = std::chrono::system_clock::now();
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            now.time_since_epoch()).count());
}

}  // namespace nmpc_uav