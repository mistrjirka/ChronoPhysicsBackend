
#include "simulation_launcher.h"

SimulationLauncher::SimulationLauncher(const ChronoSimulation::Config& config)
    : m_config(config), m_simulation(config) {
}

void SimulationLauncher::Launch() {
    m_thread = std::thread(&SimulationLauncher::RunSimulation, m_simulation);
}

void SimulationLauncher::Join() {
    m_thread.join();
}

void SimulationLauncher::RunSimulation(ChronoSimulation simulation) {
    simulation.Initialize();
    simulation.Run();
}