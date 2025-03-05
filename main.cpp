#include "main.h"
using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::generic;
using namespace chrono::vehicle::hmmwv;

// Define simulation parameters
enum class PatchType { FLAT, MESH, HEIGHMAP };
PatchType patch_type = PatchType::HEIGHMAP;
class MyDriver : public ChDriver {
  public:
    MyDriver(ChVehicle& vehicle, double delay) : ChDriver(vehicle), m_delay(delay) {}
    ~MyDriver() {}

    virtual void Synchronize(double time) override {
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

  private:
    double m_delay;
};

// Simulation step size
double step_size = 3e-3;

// Time interval between two render frames (1/FPS)
double render_step_size = 1.0 / 100;
ChQuaternion<> initRot(1, 0, 0, 0);
double delta = 0.1;
bool render_wireframe = false;  // render wireframe (flat otherwise)

int main(int argc, char* argv[]) {
    // Create the vehicle system
    ChVector3d init_loc;
    ChVector2d patch_size;
    switch (patch_type) {
        case PatchType::FLAT:
            init_loc = ChVector3d(5.0, 2.0, 0.6);
            patch_size = ChVector2d(16.0, 8.0);
            break;
        case PatchType::MESH:
            init_loc = ChVector3d(-12.0, -12.0, 1.6);
            break;
        case PatchType::HEIGHMAP:
            init_loc = ChVector3d(-140, 0, 5.0);
            patch_size = ChVector2d(40.0, 40.0);
            break;
    }
    Generic_Vehicle vehicle(false, SuspensionTypeWV::DOUBLE_WISHBONE, SuspensionTypeWV::DOUBLE_WISHBONE,
                            SteeringTypeWV::PITMAN_ARM, DrivelineTypeWV::AWD, BrakeType::SHAFTS, false, false);
    vehicle.Initialize(ChCoordsys<>(init_loc, initRot));
    vehicle.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetWheelVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    auto engine_type = EngineModelType::SHAFTS;
    auto transmission_type = TransmissionModelType::AUTOMATIC_SHAFTS;
    vehicle.CreateAndInitializePowertrain(engine_type, transmission_type);
    
    bool use_mesh = true;
    auto tire_FL = chrono_types::make_shared<HMMWV_RigidTire>("FL", use_mesh);
    auto tire_FR = chrono_types::make_shared<HMMWV_RigidTire>("FR", use_mesh);
    auto tire_RL = chrono_types::make_shared<HMMWV_RigidTire>("RL", use_mesh);
    auto tire_RR = chrono_types::make_shared<HMMWV_RigidTire>("RR", use_mesh);

    vehicle.InitializeTire(tire_FL, vehicle.GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
    vehicle.InitializeTire(tire_FR, vehicle.GetAxle(0)->m_wheels[RIGHT], VisualizationType::NONE);
    vehicle.InitializeTire(tire_RL, vehicle.GetAxle(1)->m_wheels[LEFT], VisualizationType::NONE);
    vehicle.InitializeTire(tire_RR, vehicle.GetAxle(1)->m_wheels[RIGHT], VisualizationType::NONE);


    // Create SCM deformable terrain
    auto system = vehicle.GetSystem();
    SCMTerrain terrain(system);
    terrain.SetSoilParameters(2e6,   // Bekker Kphi
                              0,     // Bekker Kc
                              1.1,   // Bekker n exponent
                              0,     // Mohr cohesive limit (Pa)
                              30,    // Mohr friction limit (degrees)
                              0.01,  // Janosi shear coefficient (m)
                              2e8,   // Elastic stiffness (Pa/m), before plastic yield
                              3e4    // Damping (Pa s/m), proportional to negative vertical speed
    );
    //terrain.SetPlotType(SCMTerrain::PLOT_SINKAGE, 0, 0.1);
    terrain.AddMovingPatch(vehicle.GetChassisBody(), ChVector3d(0, 0, 0), ChVector3d(5, 3, 1));
    terrain.Initialize("../heightmap.bmp", 300, 100, 0, 2.83, delta);
    terrain.GetMesh()->SetTexture(vehicle::GetDataFile("../data/data/vehicle/terrain/textures/grass.jpg"), 200, 200);
    terrain.GetMesh()->SetWireframe(render_wireframe);

    // Create visualization
    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("Generic Vehicle on SCM Terrain");
    vis->SetChaseCamera(ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5);
    vis->Initialize();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AddLightDirectional(60, 60.0, ChColor(0.8f, 0.8f, 0.8f));  // Directional light
    vis->AttachVehicle(&vehicle);


    // Interactive driver system
    MyDriver driver(vehicle, 0.5);
    driver.Initialize();

    // Simulation loop
    int render_steps = (int)std::ceil(1.0 / (50 * step_size));
    int step_number = 0;

    system->SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    system->GetSolver()->AsIterative()->SetMaxIterations(150);

    while (vis->Run()) {
        double time = system->GetChTime();

        // Get driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        // Update all modules
        driver.Synchronize(time);
        terrain.Synchronize(time);
        vehicle.Synchronize(time, driver_inputs, terrain);

    
        // Render the scene
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        // Advance simulation
        driver.Advance(step_size);
        terrain.Advance(step_size);
        vehicle.Advance(step_size);
        vis->Advance(step_size);

        step_number++;
    }

    return 0;
}