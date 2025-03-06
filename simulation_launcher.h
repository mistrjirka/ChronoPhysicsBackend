
#ifndef SIMULATION_LAUNCHER_H
#define SIMULATION_LAUNCHER_H

#include <thread>
#include "main.h"

class SimulationLauncher {
public:
    SimulationLauncher(const ChronoSimulation::Config& config = ChronoSimulation::Config());
    
    void Launch();
    void Join();

private:
    ChronoSimulation::Config m_config;
    ChronoSimulation m_simulation;
    std::thread m_thread;

    static void RunSimulation(ChronoSimulation simulation);
};

#endif