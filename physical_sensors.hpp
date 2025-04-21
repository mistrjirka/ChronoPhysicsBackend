#include "PreHACDFix.hpp"  // Added to provide dummy declarations for HeapManager and ICHUll
#pragma once

#include "chrono_vehicle/ChVehicle.h"
#include "ros_bridge.hpp"
#include "terrain_system.hpp"
#include <mutex>

class PhysicalSensors {
public:
    PhysicalSensors(chrono::vehicle::ChVehicle* vehicle, 
                    TerrainSystemCoordinates* coord_system,
                    double update_rate = 50.0);  // 50 Hz default
    
    void Update(double time);

private:
    void PublishOdometry(double time);
    void PublishIMU(double time);
    void InitializeTopics();

    chrono::vehicle::ChVehicle* vehicle_;
    TerrainSystemCoordinates* coord_system_;
    ROSBridge ros_bridge_;
    double update_interval_;
    double last_update_time_;
    
    // Previous state for velocity calculation
    chrono::ChVector3d last_position_;
    double last_time_;

    bool topics_initialized_ = false;
    const std::string ODOM_MSG_TYPE = "nav_msgs/Odometry";
    const std::string IMU_MSG_TYPE = "sensor_msgs/Imu";
};
