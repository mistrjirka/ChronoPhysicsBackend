#include "PreHACDFix.hpp"
#ifndef MAIN_H
#define MAIN_H
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
#include "ros_bridge_driver.hpp"
#include "physical_sensors.hpp"
#include "terrain_system.hpp"

#include "TcpPositionServer.hpp"

// Driver class for controlling the vehicle
class MyDriver : public chrono::vehicle::ChDriver {
public:
    MyDriver(chrono::vehicle::ChVehicle& vehicle, double delay);
    ~MyDriver() {}

    virtual void Synchronize(double time) override;

private:
    double m_delay;
};

// Main simulation class
class ChronoSimulation {
public:
    // Patch type enum
    enum class PatchType { FLAT, MESH, HEIGHTMAP };

    // Configuration structure
    struct Config {
        // Simulation parameters
        PatchType patchType;
        double stepSize;
        double renderStepSize;
        bool renderWireframe;
        double driverDelay;
        
        // Vehicle parameters
        chrono::ChVector3d initLoc;
        chrono::ChQuaternion<> initRot;
        bool useTerrainMesh;
        bool useVisualization; // <-- Add this line
        
        // Terrain parameters
        chrono::ChVector2d patchSize;
        std::string heightmapFile;
        double terrainHeight;
        double terrainWidth;
        double terrainZ;
        double terrainDelta;
        chrono::ChVector2d corner;
        
        // Visualization parameters
        int targetFps;
        
        // Soil parameters
        double soilKphi;      // Bekker Kphi
        double soilKc;        // Bekker Kc
        double soilN;         // Bekker n exponent
        double soilCohesion;  // Mohr cohesive limit (Pa)
        double soilFriction;  // Mohr friction limit (degrees)
        double soilJanosi;    // Janosi shear coefficient (m)
        double soilStiffness; // Elastic stiffness (Pa/m)
        double soilDamping;   // Damping (Pa s/m)
        double unrealZOfsset;
        
        // Default constructor declaration (defined in .cpp)
        Config();
    };

    // Constructor
    ChronoSimulation(const Config& config = Config());
    
    // Initialize the simulation
    void Initialize();
    
    // Run the simulation
    void Run();
    
    // Configuration setter
    void SetConfig(const Config& config) { m_config = config; }
    const Config& GetConfig() const { return m_config; }

    // Add RTF monitoring methods
    double GetRTF() const { return m_vehicle ? m_vehicle->GetRTF() : 0.0; }
    double GetStepRTF() const { return m_vehicle ? m_vehicle->GetStepRTF() : 0.0; }

private:
    Config m_config;
    
    // System components
    chrono::ChSystem* m_system;  // Raw pointer to the system (owned by m_vehicle)
    std::shared_ptr<chrono::vehicle::generic::Generic_Vehicle> m_vehicle;
    std::shared_ptr<chrono::vehicle::SCMTerrain> m_terrain;
    std::shared_ptr<chrono::vehicle::ChWheeledVehicleVisualSystemIrrlicht> m_vis;
    std::shared_ptr<TerrainSystemCoordinates> m_terrain_coords;
    std::shared_ptr<PhysicalSensors> m_sensors;
    std::shared_ptr<ROSDriver> m_driver;
    TcpPositionServer m_tcp_server;
    double last_sleep_time;
    double last_render_sleep_time;
    double z_min_offset;
    double z_scale;
    
    // Helper methods
    void SetupVehicle();
    void SetupTerrain();
    void SetupVisualization();
    void SetupSensors();

    void GetScale();
    double GetSleepTime(bool render);

    // Add RTF monitoring variables
    double m_targetRTF;
    void UpdateTimestep();
};

#endif