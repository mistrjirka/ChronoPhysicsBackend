#include "main.h"
#include "simulation_launcher.h"
#include <chrono>
#include <thread>
#include "chrono/core/ChRealtimeStep.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::generic;
using namespace chrono::vehicle::hmmwv;

// Implementation of MyDriver methods
MyDriver::MyDriver(ChVehicle& vehicle, double delay) : ChDriver(vehicle), m_delay(delay) {}

void MyDriver::Synchronize(double time) {
    m_throttle = 0;
    m_steering = 0;
    m_braking = 0;

    double eff_time = time - m_delay;

    // Do not generate any driver inputs for a duration equal to m_delay.
    if (eff_time < 0)
        return;

    if (eff_time > 0.2)
        m_throttle = 0.7;
    else
        m_throttle = 3.5 * eff_time;

    if (eff_time < 2)
        m_steering = 0;
    else
        m_steering = 0.6 * std::sin(CH_2PI * (eff_time - 2) / 6);
}

// ChronoSimulation::Config implementation
ChronoSimulation::Config::Config() :
    patchType(PatchType::HEIGHTMAP),
    stepSize(3e-3),
    renderStepSize(1.0 / 100),
    renderWireframe(false),
    driverDelay(0.5),
    initLoc(ChVector3d(-140, 0, 5.0)),
    initRot(ChQuaternion<>(1, 0, 0, 0)),
    useTerrainMesh(true),
    patchSize(ChVector2d(40.0, 40.0)),
    heightmapFile("../heightmap.bmp"),
    terrainHeight(300),
    terrainWidth(100),
    terrainZ(2.83),
    terrainDelta(0.1),
    targetFps(30),
    soilKphi(2e6),
    soilKc(0),
    soilN(1.1),
    soilCohesion(0),
    soilFriction(30),
    soilJanosi(0.01),
    soilStiffness(2e8),
    soilDamping(3e4),
    corner(ChVector2d(2500, -7500))
{}

// ChronoSimulation implementation
ChronoSimulation::ChronoSimulation(const Config& config) : m_config(config), m_system(nullptr) {
}

void ChronoSimulation::Initialize() {
    // Setup the vehicle
    SetupVehicle();
    
    // Setup terrain
    SetupTerrain();

    SetupSensors();
    // Setup visualization
    SetupVisualization();
    
    // Create driver
    m_driver = std::make_shared<ROSDriver>(*m_vehicle);
    m_driver->Initialize();
    
    // Get system pointer (owned by the vehicle, do not delete)
    m_system = m_vehicle->GetSystem();
    
    // Configure solver
    m_system->SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    m_system->GetSolver()->AsIterative()->SetMaxIterations(150);
}

void ChronoSimulation::SetupSensors() {
    // Create the physical sensors
    m_sensors = std::make_shared<PhysicalSensors>(m_vehicle.get(), m_terrain_coords.get());
}

void ChronoSimulation::SetupVehicle() {
    // Determine initial location based on patch type
    ChVector3d init_loc = m_config.initLoc;
    if (m_config.patchType == PatchType::FLAT) {
        init_loc = ChVector3d(5.0, 2.0, 0.6);
    } else if (m_config.patchType == PatchType::MESH) {
        init_loc = ChVector3d(-12.0, -12.0, 1.6);
    }
    
    // Create the vehicle
    m_vehicle = std::make_shared<Generic_Vehicle>(
        false, 
        SuspensionTypeWV::DOUBLE_WISHBONE, 
        SuspensionTypeWV::DOUBLE_WISHBONE,
        SteeringTypeWV::PITMAN_ARM, 
        DrivelineTypeWV::AWD, 
        BrakeType::SHAFTS, 
        false, 
        false
    );
    
    m_vehicle->Initialize(ChCoordsys<>(init_loc, m_config.initRot));
    m_vehicle->SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    m_vehicle->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    m_vehicle->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    m_vehicle->SetWheelVisualizationType(VisualizationType::PRIMITIVES);
    m_vehicle->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    
    // Initialize powertrain
    auto engine_type = EngineModelType::SHAFTS;
    auto transmission_type = TransmissionModelType::AUTOMATIC_SHAFTS;
    m_vehicle->CreateAndInitializePowertrain(engine_type, transmission_type);
    
    // Setup tires
    bool use_mesh = m_config.useTerrainMesh;
    auto tire_FL = chrono_types::make_shared<HMMWV_RigidTire>("FL", use_mesh);
    auto tire_FR = chrono_types::make_shared<HMMWV_RigidTire>("FR", use_mesh);
    auto tire_RL = chrono_types::make_shared<HMMWV_RigidTire>("RL", use_mesh);
    auto tire_RR = chrono_types::make_shared<HMMWV_RigidTire>("RR", use_mesh);

    m_vehicle->InitializeTire(tire_FL, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
    m_vehicle->InitializeTire(tire_FR, m_vehicle->GetAxle(0)->m_wheels[RIGHT], VisualizationType::NONE);
    m_vehicle->InitializeTire(tire_RL, m_vehicle->GetAxle(1)->m_wheels[LEFT], VisualizationType::NONE);
    m_vehicle->InitializeTire(tire_RR, m_vehicle->GetAxle(1)->m_wheels[RIGHT], VisualizationType::NONE);
}

double ChronoSimulation::GetScale() {
    double scale = m_config.terrainZ;
    double actual_height = 0.0;
    double tolerance = 0.01; // 1 cm tolerance
    int max_iterations = 1;
    int iteration = 0;

    double min_z = 0.0;
    double max_z = 0.0;
    std::shared_ptr<SCMTerrain> TmpTerrain = std::make_shared<SCMTerrain>(m_vehicle->GetSystem());
    TmpTerrain->SetSoilParameters(
        m_config.soilKphi,
        m_config.soilKc,
        m_config.soilN,
        m_config.soilCohesion,
        m_config.soilFriction,
        m_config.soilJanosi,
        m_config.soilStiffness,
        m_config.soilDamping
    );

    TmpTerrain->Initialize(
        m_config.heightmapFile, 
        m_config.terrainHeight, 
        m_config.terrainWidth, 
        0, 
        m_config.terrainZ, 
        m_config.terrainDelta
    );
    
    auto meshHolder = TmpTerrain->GetMesh();
    auto mesh = meshHolder->GetMesh();
    const auto &vertices = mesh->GetCoordsVertices();

    min_z = std::numeric_limits<double>::max();
    max_z = std::numeric_limits<double>::lowest();

    for (const auto &v : vertices)
    {
        if (v.z() < min_z)
            min_z = v.z();
        if (v.z() > max_z)
            max_z = v.z();
    }


    actual_height = max_z - min_z;


    double scaling_factor = m_config.terrainZ / actual_height;
    scale *= scaling_factor;
    return scale;
}

void ChronoSimulation::SetupTerrain() {
    // Create terrain
    m_terrain = std::make_shared<SCMTerrain>(m_vehicle->GetSystem());
    
    // Set soil parameters
    m_terrain->SetSoilParameters(
        m_config.soilKphi,
        m_config.soilKc,
        m_config.soilN,
        m_config.soilCohesion,
        m_config.soilFriction,
        m_config.soilJanosi,
        m_config.soilStiffness,
        m_config.soilDamping
    );
    
    // Add moving patch and initialize
    m_terrain->AddMovingPatch(m_vehicle->GetChassisBody(), ChVector3d(0, 0, 0), ChVector3d(5, 3, 1));

    double scale = m_config.terrainZ;
    double actual_height = 0.0;

    m_terrain->Initialize(
        m_config.heightmapFile, 
        m_config.terrainHeight, 
        m_config.terrainWidth, 
        m_config.terrainZ, 
        GetScale(), 
        m_config.terrainDelta
    );
    
    // Set terrain appearance
    m_terrain->GetMesh()->SetTexture(vehicle::GetDataFile("../data/data/vehicle/terrain/textures/grass.jpg"), 200, 200);
    m_terrain->GetMesh()->SetWireframe(m_config.renderWireframe);

    m_terrain_coords = std::make_shared<TerrainSystemCoordinates>(
        m_config.terrainWidth, 
        m_config.terrainHeight,
        m_config.terrainZ,
        m_config.corner);
}

void ChronoSimulation::SetupVisualization() {
    // Create visualization
    m_vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    m_vis->SetWindowTitle("Generic Vehicle on SCM Terrain");
    m_vis->SetChaseCamera(ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5);
    m_vis->Initialize();
    m_vis->AddSkyBox();
    m_vis->AddLogo();
    m_vis->AddLightDirectional(60, 60.0, ChColor(0.8f, 0.8f, 0.8f));
    m_vis->AttachVehicle(m_vehicle.get());
}


void ChronoSimulation::Run() {
    double render_step_size = 1.0 / m_config.targetFps;
    double last_render_time = 0.0;

    ChRealtimeStepTimer realtime_timer;
    while (m_vis->Run()) {
        // Capture the start time of this iteration

        double time = m_system->GetChTime();

        // Get driver inputs
        DriverInputs driver_inputs = m_driver->GetInputs();

        // Update all modules
        m_driver->Synchronize(time);
        m_terrain->Synchronize(time);
        m_vehicle->Synchronize(time, driver_inputs, *m_terrain);

        // Render the scene at the specified FPS
        if (time - last_render_time >= render_step_size) {

            m_vis->BeginScene();
            m_vis->Render();
            m_vis->EndScene();
            last_render_time = time;


            // Adjust next_step_time to account for rende
        }

        // Advance simulation
        m_driver->Advance(m_config.stepSize);
        m_terrain->Advance(m_config.stepSize);
        m_vehicle->Advance(m_config.stepSize);
        m_vis->Advance(m_config.stepSize);
        m_sensors->Update(time);

        realtime_timer.Spin(m_config.stepSize);
    }
}
int main(int argc, char* argv[]) {
    // Create simulation with default configuration
    ChronoSimulation::Config config;
    
    // Customize configuration if needed
    // config.patchType = ChronoSimulation::PatchType::FLAT;
    // config.heightmapFile = "../custom_heightmap.bmp";
    // config.driverDelay = 1.0;
    
    // Launch the simulation in a new thread
    SimulationLauncher launcher(config);
    launcher.Launch();

    // Do other things in the main thread
    // ...

    // Wait for the simulation thread to finish
    launcher.Join();

    return 0;
}