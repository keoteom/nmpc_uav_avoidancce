#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <array>
#include <vector>
#include <memory>
#include <string>

#include "nmpc_uav_avoidance/open_solver.hpp"

namespace nmpc_uav {

// ============================================================================
//  Constants
// ============================================================================
constexpr int    N        = 30;
constexpr int    NX       = 8;
constexpr int    NU       = 3;
constexpr int    N_PARAMS = 118;
constexpr double Ts       = 0.05;
constexpr double GRAVITY  = 9.81;

constexpr double AX = 0.1, AY = 0.1, AZ = 0.1;

constexpr double TAU_PHI   = 0.3, K_PHI   = 1.0;
constexpr double TAU_THETA = 0.3, K_THETA = 1.0;

constexpr int OFFBOARD_SETPOINT_COUNT = 10;

// Lookahead 클램프 범위
constexpr double LOOKAHEAD_MIN = 0.5;   // m
constexpr double LOOKAHEAD_MAX = 3.0;   // m

// ============================================================================
//  POD types
// ============================================================================
struct EulerAngles { double roll, pitch, yaw; };
struct Quaternion  { double x, y, z, w; };

// ============================================================================
//  Free helpers
// ============================================================================
EulerAngles quaternion_to_euler(double qx, double qy, double qz, double qw);
Quaternion  euler_to_quaternion(double roll, double pitch, double yaw);
std::array<double, NX> dynamics_step(const std::array<double, NX>& x,
                                     const std::array<double, NU>& u);

// ============================================================================
//  NMPCNode
// ============================================================================
class NMPCNode : public rclcpp::Node {
public:
    NMPCNode();
    ~NMPCNode();

private:
    // ---- Solver ----
    std::unique_ptr<OpEnSolver> solver_;

    // ---- UAV state ----
    std::array<double, NX> state_{};
    std::array<double, NU> u_prev_{GRAVITY, 0.0, 0.0};
    double yaw_            = 0.0;
    bool   state_received_ = false;

    // ---- Path ----
    std::vector<std::array<double, 3>> path_points_;
    size_t path_idx_ = 0;

    // ---- Dynamic obstacle ----
    std::array<double, 3> obs_pos_{};
    std::array<double, 3> obs_vel_{};
    double r_obs_        = 1.0;
    bool   obs_received_ = false;

    // ---- Control params ----
    double T_max_ = 13.5;
    double T_min_ = 5.0;
    double K_psi_ = 1.0;

    // ---- Offboard ----
    int     offboard_counter_  = 0;
    bool    offboard_mode_set_ = false;
    uint8_t nav_state_         = 0;
    uint8_t arming_state_      = 0;

    // ---- Adaptive lookahead ----
    double last_solve_time_s_ = Ts;   // 초기값 = 한 스텝

    // ---- ROS interfaces ----
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      sub_odom_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      sub_obs_odom_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr          sub_path_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr sub_status_;

    rclcpp::Publisher<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr   pub_thrust_;
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr pub_att_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr     pub_offboard_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr          pub_command_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                    pub_pred_path_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr        pub_obs_marker_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr                  pub_solver_time_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr                  pub_cost_function_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr        pub_control_input_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr        pub_jerk_;

    rclcpp::TimerBase::SharedPtr timer_;

    // ---- Methods ----
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void obs_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
    void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);

    void publish_vehicle_command(uint16_t command,
                                 float p1=0, float p2=0, float p3=0, float p4=0,
                                 float p5=0, float p6=0, float p7=0);
    void set_offboard_mode();
    void arm();
    void disarm();

    void update_reference();
    void global_to_body_angles(double phi_g, double theta_g,
                                double& phi_b, double& theta_b) const;

    void control_loop();
    void publish_fallback_setpoint();
    void publish_offboard_mode();
    void publish_predicted_path(const std::vector<double>& z_star);
    void publish_obstacle_marker();

    static uint64_t px4_timestamp();
};

}  // namespace nmpc_uav