#pragma once
#include "instance.h"
#include "solver.h"


class Simulator
{
    public:
        int delay_simulation = MAX_TIMESTEP; //how many timesteps we simulate for delay

        Simulator(const Instance &instance, int delay_solver, int recover_solver, int time_limit): 
        instance(instance),delay_solver(delay_solver),recover_solver(recover_solver),time_limit(time_limit)
        {
            no_delay_path.resize(instance.num_of_agents);
            curr_path.resize(instance.num_of_agents);
        }

        bool loadPaths(const string & file_name);
        bool loadDelays(const string & file_name);

        void simulation();

        void saveSimulatePaths(const string & file_name) const;

        void validateSolution() const;

    private:
        vector<Path> no_delay_path;
        vector<Path> curr_path;
        const Instance& instance;
        vector<vector<bool>> delays;
        int delay_solver = 1; //1--mcp
        int recover_solver = 1; //1--mcp
        double time_limit;
};