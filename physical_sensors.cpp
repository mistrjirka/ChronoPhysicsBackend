#include "physical_sensors.hpp"

using namespace chrono;

PhysicalSensors::PhysicalSensors(vehicle::ChVehicle* vehicle, 
                                TerrainSystemCoordinates* coord_system,
                                double update_rate)
    : vehicle_(vehicle)
    , coord_system_(coord_system)
    , update_interval_(1.0 / update_rate)
    , last_update_time_(0)
    , last_time_(0) {
    
    if (!ros_bridge_.connect("ws://localhost:9090")) {
        std::cerr << "Failed to connect to ROS Bridge for sensors" << std::endl;
        return;
    }
    
    if (!ros_bridge_.waitForConnection()) {
        std::cerr << "ROS Bridge connection timeout for sensors" << std::endl;
        return;
    }

    // Initialize topics
    InitializeTopics();

    // Initialize last position
    last_position_ = vehicle_->GetChassisBody()->GetPos();
}

void PhysicalSensors::InitializeTopics() {
    // Advertise odometry first
    if (!ros_bridge_.advertise("/robot0/odom", ODOM_MSG_TYPE)) {
        std::cerr << "Failed to advertise odometry topic" << std::endl;
        return;
    }
    
    // Wait half a second after first topic
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Advertise IMU second
    if (!ros_bridge_.advertise("/robot0/imu", IMU_MSG_TYPE)) {
        std::cerr << "Failed to advertise IMU topic" << std::endl;
        return;
    }
    
    // Wait another half second to ensure both topics are fully registered
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    topics_initialized_ = true;
    std::cout << "ROS topics successfully advertised" << std::endl;
}

void PhysicalSensors::Update(double time) {
    if (!topics_initialized_) return;
    if (time - last_update_time_ >= update_interval_) {
        PublishOdometry(time);
        PublishIMU(time);
        last_update_time_ = time;
    }
}

void PhysicalSensors::PublishOdometry(double time) {
    auto chassis = vehicle_->GetChassisBody();
    auto pos = chassis->GetPos();
    auto rot = chassis->GetRot();
    
    // Convert rotation quaternion to Euler angles (roll, pitch, yaw)
    double q0 = rot.e0();  // scalar part
    double q1 = rot.e1();
    double q2 = rot.e2();
    double q3 = rot.e3();

    // Compute roll (x-axis rotation)
    double roll = atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2));

    // Compute pitch (y-axis rotation)
    double sinp = 2.0 * (q0 * q2 - q3 * q1);
    double pitch = fabs(sinp) >= 1 ? copysign(M_PI / 2, sinp) : asin(sinp);

    // Compute yaw (z-axis rotation)
    double yaw = atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3));

    // Convert Chrono Euler angles to ROS rotation
    ChVector3d euler_angles(roll, pitch, yaw);
    ChVector3d ros_euler_angles = coord_system_->convertChronotToROSRotation(euler_angles);
    
    // Calculate velocities using correct methods from ChFrameMoving
    ChVector3d vel = chassis->GetPosDt();      // Linear velocity
    ChQuaternion<> rotDt = chassis->GetRotDt(); // Rotational velocity as quaternion
    
    // Convert to ROS coordinate system using the same transformation as TCP server
    ChVector3d ros_pos = coord_system_->convertChronoToROS(pos);
    ChVector3d ros_vel = coord_system_->convertChronoToROSDirection(vel);
    
    // Convert quaternion velocity to angular velocity
    ChVector3d ang_vel = 2.0 * (rotDt * rot.GetConjugate()).GetVector();
    ChVector3d ros_ang_vel = coord_system_->convertChronotToROSRotation(ang_vel);
    
    // Create quaternion from Euler angles
    double cy = cos(ros_euler_angles.z() * 0.5);
    double sy = sin(ros_euler_angles.z() * 0.5);
    double cp = cos(ros_euler_angles.y() * 0.5);
    double sp = sin(ros_euler_angles.y() * 0.5);
    double cr = cos(ros_euler_angles.x() * 0.5);
    double sr = sin(ros_euler_angles.x() * 0.5);

    ChQuaternion<> ros_quat(
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy
    );

    // Ensure quaternion is valid
    if (std::isnan(ros_quat.e0()) || std::isnan(ros_quat.e1()) || 
        std::isnan(ros_quat.e2()) || std::isnan(ros_quat.e3())) {
        ros_quat = ChQuaternion<>(1, 0, 0, 0); // Default to identity if invalid
    }

    // Create odometry message with explicit double type specification
    json odom_msg = {
        {"header", {
            {"frame_id", "ouster"},  // Changed from "odom" to "ouster"
            {"stamp", {
                {"sec", (int)time},
                {"nanosec", (int)((time - (int)time) * 1e9)}
            }}
        }},
        {"child_frame_id", "base_link"},
        {"pose", {
            {"pose", {
                {"position", {
                    {"x", ros_pos.x()},
                    {"y", ros_pos.y()},
                    {"z", ros_pos.z()}
                }},
                {"orientation", {
                    {"x", static_cast<double>(ros_quat.e1())},
                    {"y", static_cast<double>(ros_quat.e2())},
                    {"z", static_cast<double>(ros_quat.e3())},
                    {"w", static_cast<double>(ros_quat.e0())}
                }}
            }}
        }},
        {"twist", {
            {"twist", {
                {"linear", {
                    {"x", ros_vel.x()},
                    {"y", ros_vel.y()},
                    {"z", ros_vel.z()}
                }},
                {"angular", {
                    {"x", ros_ang_vel.x()},
                    {"y", ros_ang_vel.y()},
                    {"z", ros_ang_vel.z()}
                }}
            }}
        }}
    };
    
    ros_bridge_.publish("/robot0/odom", ODOM_MSG_TYPE, odom_msg);
}

void PhysicalSensors::PublishIMU(double time) {
    auto chassis = vehicle_->GetChassisBody();
    
    // Get acceleration using correct methods from ChFrameMoving
    ChVector3d acc = chassis->GetPosDt2();     // Linear acceleration
    
    // Add gravity vector in global frame (approximately 9.81 m/s^2 downward)
    ChVector3d gravity(0, 0, 9.81);
    
    // Transform gravity to vehicle's local frame using quaternion rotation
    ChQuaternion<> rot = chassis->GetRot();
    ChVector3d local_gravity = rot.RotateBack(gravity);
    
    // Add local gravity to acceleration
    acc += local_gravity;

    //std::cout << "Linear acceleration (with gravity): " << acc << std::endl;
    ChQuaternion<> rotDt = chassis->GetRotDt(); // Rotational velocity as quaternion
    
    // Convert quaternion velocity to angular velocity
    ChVector3d ang_vel = 2.0 * (rotDt * chassis->GetRot().GetConjugate()).GetVector();
    
    // Convert to ROS coordinate system
    ChVector3d ros_acc = coord_system_->convertChronoToROSDirection(acc);
    ChVector3d ros_ang_vel = coord_system_->convertChronotToROSRotation(ang_vel);
    
    // Ensure all values are valid doubles
    json imu_msg = {
        {"header", {
            {"frame_id", "ouster"},  // Changed from "imu_link" to "ouster"
            {"stamp", {
                {"sec", (int)time},
                {"nanosec", (int)((time - (int)time) * 1e9)}
            }}
        }},
        {"orientation", {
            {"x", 0.0},  // We could add orientation if needed
            {"y", 0.0},
            {"z", 0.0},
            {"w", 1.0}
        }},
        {"angular_velocity", {
            {"x", static_cast<double>(ros_ang_vel.x())},
            {"y", static_cast<double>(ros_ang_vel.y())},
            {"z", static_cast<double>(ros_ang_vel.z())}
        }},
        {"linear_acceleration", {
            {"x", static_cast<double>(ros_acc.x())},
            {"y", static_cast<double>(ros_acc.y())},
            {"z", static_cast<double>(ros_acc.z())}
        }}
    };
    
    ros_bridge_.publish("/robot0/imu", IMU_MSG_TYPE, imu_msg);
}
