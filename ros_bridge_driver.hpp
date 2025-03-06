#pragma once

#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
#include "chrono_models/vehicle/generic/Generic_Wheel.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono/assets/ChColor.h"
#include "chrono_vehicle/ChTerrain.h"
#include "chrono_vehicle/ChDriver.h"
#include "chrono/core/ChCoordsys.h"
#include "chrono/core/ChChrono.h"
#include "chrono_models/vehicle/hmmwv/tire/HMMWV_RigidTire.h"

#include "chrono_vehicle/chassis/ChRigidChassis.h"
#include "chrono_models/vehicle/generic/Generic_Vehicle.h"

#include "chrono_models/vehicle/ChVehicleModelDefs.h"
#include "chrono/core/ChVector3.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChVector2.h"
#include "ros_bridge.hpp"
#include <mutex>
#include <atomic>  // Add this include if needed

using namespace chrono::vehicle;
using namespace chrono;

struct PIDController {
    double Kp, Ki, Kd;
    double integral = 0;
    double prev_error = 0;
    double output_min, output_max;

    PIDController(double kp, double ki, double kd, double min, double max) 
        : Kp(kp), Ki(ki), Kd(kd), output_min(min), output_max(max) {}

    double update(double error, double dt) {
        integral += error * dt;
        double derivative = (error - prev_error) / dt;
        prev_error = error;
        
        double output = Kp * error + Ki * integral + Kd * derivative;
        return std::max(output_min, std::min(output_max, output));
    }

    void reset() {
        integral = 0;
        prev_error = 0;
    }
};

class ROSDriver : public ChDriver {
public:
    ROSDriver(ChVehicle& vehicle, double max_speed = 20.0, double max_steering = 0.5) 
        : ChDriver(vehicle), 
          max_speed_(max_speed), 
          max_steering_(max_steering),
          m_step_size(1.0/100.0),  // Add this: default 100Hz simulation
          speed_controller(2.0, 0.1, 0.1, -1.0, 1.0),  // Tune these PID values
          steering_controller(2.0, 0.0, 0.5, -1.0, 1.0) {
        
        // Cast to wheeled vehicle to get axle information
        if (auto wheeled_vehicle = dynamic_cast<ChWheeledVehicle*>(&vehicle)) {
            // Calculate wheelbase as distance between front and rear axles
            auto front_axle = wheeled_vehicle->GetAxle(0);
            auto rear_axle = wheeled_vehicle->GetAxle(wheeled_vehicle->GetNumberAxles() - 1);
            
            if (front_axle && rear_axle) {
                // Get wheel positions through suspension
                auto front_wheel = front_axle->GetWheels()[0];  // Get first wheel of front axle
                auto rear_wheel = rear_axle->GetWheels()[0];    // Get first wheel of rear axle
                
                if (front_wheel && rear_wheel) {
                    auto front_pos = front_wheel->GetSpindle()->GetPos();
                    auto rear_pos = rear_wheel->GetSpindle()->GetPos();
                    wheelbase_ = (front_pos - rear_pos).Length();
                    std::cout << "Calculated vehicle wheelbase: " << wheelbase_ << " m" << std::endl;
                } else {
                    std::cerr << "Could not access wheels, using default wheelbase" << std::endl;
                    wheelbase_ = 3.0;  // Default wheelbase for MAN Kat 1
                }
            } else {
                std::cerr << "Could not access vehicle axles, using default wheelbase" << std::endl;
                wheelbase_ = 3.0;
            }
        } else {
            std::cerr << "Vehicle is not a wheeled vehicle, using default wheelbase" << std::endl;
            wheelbase_ = 3.0;
        }
        
        // Initialize ROS Bridge
        std::cout << "Connecting to ROS Bridge" << std::endl;
        if (!ros_bridge_.connect("ws://localhost:9090")) {
            std::cerr << "Failed to initiate ROS Bridge connection" << std::endl;
            return;
        }
        
        if (!ros_bridge_.waitForConnection(5000)) {
            std::cerr << "Failed to connect to ROS Bridge within timeout" << std::endl;
            return;
        }
        
        std::cout << "Connected to ROS Bridge" << std::endl;  
        
        // Subscribe to cmd_vel topic
        if (!ros_bridge_.subscribe("/robot0/cmd_vel", "geometry_msgs/Twist",
            [this](const json& msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                target_linear_vel_ = msg["linear"]["x"].get<double>();
                target_angular_vel_ = msg["angular"]["z"].get<double>();
            })) {
            std::cerr << "Failed to subscribe to /robot0/cmd_vel" << std::endl;
        } else {
            std::cout << "Successfully subscribed to /robot0/cmd_vel" << std::endl;
        }
    }

    ~ROSDriver() {
        ros_bridge_.disconnect();
    }

    virtual void Synchronize(double time) override {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // Get current vehicle state
        auto chassis = m_vehicle.GetChassis();
        ChVector3d vel = chassis->GetBody()->GetPosDt();
        double current_speed = vel.Length();
        double current_heading = chassis->GetRot().GetCardanAnglesXYZ().z();
        
        // Calculate desired heading from angular velocity
        static double desired_heading = current_heading;
        desired_heading += target_angular_vel_ * m_step_size;

        // Special handling for zero linear velocity command
        if (std::abs(target_linear_vel_) < 1e-3) {  // Small threshold for zero
            m_throttle = 0.0;
            m_braking = 1.0;  // Apply significant braking force (80%)
        } else {
            // Normal PID control for non-zero speeds
            double speed_error = target_linear_vel_ - current_speed;
            double throttle_brake = speed_controller.update(speed_error, m_step_size);
            
            if (throttle_brake > 0) {
                m_throttle = throttle_brake;
                m_braking = 0;
            } else {
                m_throttle = 0;
                m_braking = -throttle_brake;
            }
        }

        // Steering control remains the same
        double heading_error = desired_heading - current_heading;
        while (heading_error > CH_PI) heading_error -= 2 * CH_PI;
        while (heading_error < -CH_PI) heading_error += 2 * CH_PI;
        
        m_steering = steering_controller.update(heading_error, m_step_size);

        // Debug output
        if (time - last_debug_time_ >= 1.0) {
            std::cout << "Control state at t=" << time 
                     << "\nTarget: v=" << target_linear_vel_ 
                     << " ω=" << target_angular_vel_
                     << "\nCurrent: v=" << current_speed 
                     << " θ=" << current_heading
                     << "\nOutputs: throttle=" << m_throttle 
                     << " brake=" << m_braking 
                     << " steering=" << m_steering << std::endl;
            last_debug_time_ = time;
        }
    }

    // Add getter methods for debugging
    double GetCurrentSteering() const { return m_steering; }
    double GetCurrentThrottle() const { return m_throttle; }
    double GetCurrentBraking() const { return m_braking; }

private:
    ROSBridge ros_bridge_;
    std::mutex mutex_;
    double max_speed_;    // Maximum speed in m/s
    double max_steering_; // Maximum steering angle in radians
    // Add timestamp for last received command
    double last_command_time = 0;
    double wheelbase_;  // Vehicle wheelbase for steering calculations
    double m_step_size;  // Add this member variable

    // Target values (set by callback)
    double target_linear_vel_ = 0;
    double target_angular_vel_ = 0;
    
    // Controllers
    PIDController speed_controller;
    PIDController steering_controller;
    
    // Debug timing
    double last_debug_time_ = 0;
};
