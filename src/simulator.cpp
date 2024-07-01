#include "simulator.h"
#include<boost/tokenizer.hpp>
#include "mcp.h"
#include "lacam2/lacam2.hpp"

void Simulator::simulation()
{
    if (delay_solver == 1 && recover_solver == 1)
    {
        MCP postmcp(instance.map_size);
        vector<Path*> temp;
        
        temp.resize(curr_path.size());
        for (int a = 0; a < instance.num_of_agents; a++)
        {
            temp[a] = &(curr_path[a]);
        }

        //postmcp.simulation_time = delay_simulation;
        postmcp.max_delay_time = delay_simulation;
        postmcp.build(temp);
        postmcp.simulate(temp,delays);
        postmcp.clear();
    }

    if (delay_solver != recover_solver && delay_solver == 1)
    {
        MCP postmcp(instance.map_size);
        vector<Path*> temp;
        
        temp.resize(curr_path.size());
        for (int a = 0; a < instance.num_of_agents; a++)
        {
            temp[a] = &(curr_path[a]);
        }

        postmcp.simulation_time = delay_simulation;
        postmcp.build(temp);
        postmcp.simulate(temp,delays);
        postmcp.clear();

        vector<int> starts;
        starts.resize(instance.num_of_agents);
        instance.guidance_path.clear();
        instance.guidance_path.resize(instance.num_of_agents);

        for (int i = 0; i < instance.num_of_agents; i++)
        {
            if (curr_path[i].size() <= delay_simulation)
            {
                starts[i] = curr_path[i].back().location;
            }
            else
            {
                starts[i] = curr_path[i][delay_simulation].location;
                instance.guidance_path[i].resize(curr_path[i].size()-delay_simulation);
                for (int j = 0; j < instance.guidance_path[i].size(); j++)
                {
                    instance.guidance_path[i][j] = curr_path[i][j+delay_simulation].location;
                }
                curr_path[i].resize(delay_simulation+1);
            }
        }

        string map_fname = instance.getMapFile();
        LACAMInstance ins = LACAMInstance(map_fname, starts, instance.getGoals());
        auto MT = std::mt19937(0);
        const auto deadline = Deadline(time_limit * 1000);
        const Objective objective = static_cast<Objective>(1);
        const float restart_rate = 0.01;
        string verbose = "1";
        const auto solution = solve(instance, ins, verbose, 0, &deadline, &MT, objective, restart_rate);

        bool solved = true;
        for (int agent = 0; agent < instance.num_of_agents; agent++)
        {
            if (solution.back()[agent]->index != instance.getGoals()[agent])
            {
                solved = false; //find not stay at target
                break;
            }
        }

        for (int agent = 0; agent < instance.num_of_agents; agent++)
        {
            size_t max_time = solution.size()-1;
            for (; max_time > 0; max_time--)
            {
                if (solution[max_time][agent]->index != solution[max_time-1][agent]->index)
                    break;
            }
            curr_path[agent].resize(max_time+delay_simulation+1);
            for (size_t t = 0; t <= max_time; t++)
            {
                curr_path[agent][t+delay_simulation].location = solution[t][agent]->index;
            }
        }
    }

    // if (delay_solver == 1 && recover_solver == 2)
    // {
    //     string map_fname = instance.getMapFile();
    //     LACAMInstance ins = LACAMInstance(map_fname, instance.getStarts(), instance.getGoals());
    //     auto MT = std::mt19937(0);
    //     const auto deadline = Deadline(time_limit * 1000);
    //     const Objective objective = static_cast<Objective>(1);
    //     const float restart_rate = 0.01;
    //     string verbose = "1";
    //     const auto solution = solve(ins, verbose, 0, &deadline, &MT, objective, restart_rate);

    //     bool solved = true;
    //     for (int agent = 0; agent < instance.num_of_agents; agent++)
    //     {
    //         if (solution.back()[agent]->index != instance.getGoals()[agent])
    //         {
    //             solved = false; //find not stay at target
    //             break;
    //         }
    //     }

    //     for (int agent = 0; agent < instance.num_of_agents; agent++)
    //     {
    //         size_t max_time = solution.size()-1;
    //         for (; max_time > 0; max_time--)
    //         {
    //             if (solution[max_time][agent]->index != solution[max_time-1][agent]->index)
    //                 break;
    //         }
    //         curr_path[agent].resize(max_time+1);
    //         for (size_t t = 0; t <= max_time; t++)
    //         {
    //             curr_path[agent][t].location = solution[t][agent]->index;
    //         }
    //     }
    // }

    int soc = 0;
    for (auto p: no_delay_path)
    {
        soc+=p.size()-1;
    }
    cout<<"original soc: "<<soc<<endl;
    soc = 0;
    for (auto p: curr_path)
    {
        soc+=p.size()-1;
    }
    cout<<"delayed soc: "<<soc<<endl;

    validateSolution();
}

bool Simulator::loadPaths(const string & file_name)
{
    using namespace std;

    string line;
    ifstream myfile (file_name.c_str());
    if (!myfile.is_open())
        return false;

    while(getline(myfile, line))
    {
        boost::char_separator<char> sep(":()-> ,");
        boost::tokenizer< boost::char_separator<char> > tok(line, sep);
        auto beg = tok.begin();
        beg++; // skip "Agent"
        int agent_id = atoi((*beg).c_str());
        beg++;
        while (beg != tok.end())
        {
            int row = atoi((*beg).c_str());
            beg++;
            int col = atoi((*beg).c_str());
            beg++;
            no_delay_path[agent_id].emplace_back(instance.linearizeCoordinate(row, col));
            curr_path[agent_id].emplace_back(instance.linearizeCoordinate(row, col));
        }
        if (no_delay_path[agent_id].front().location != instance.start_locations[agent_id])
        {
            cerr << "Agent " << agent_id <<"'s path starts at " << no_delay_path[agent_id].front().location
            << "=(row " << instance.getRowCoordinate(no_delay_path[agent_id].front().location)
            << ",col " << instance.getColCoordinate(no_delay_path[agent_id].front().location)
            << "), which is different from its start location " << instance.start_locations[agent_id] << endl
            << "=(row " << instance.getRowCoordinate(instance.start_locations[agent_id])
            << ",col " << instance.getColCoordinate(instance.start_locations[agent_id])
            << ")" << endl;
            exit(-1);
        }

        if (no_delay_path[agent_id].back().location != instance.goal_locations[agent_id])
        {
            cerr << "Agent " << agent_id <<"'s path stays at " << no_delay_path[agent_id].back().location
            << "=(" << instance.getColCoordinate(no_delay_path[agent_id].back().location)
            << "," << instance.getRowCoordinate(no_delay_path[agent_id].back().location)
            << "), which is different from its goal location " << instance.goal_locations[agent_id] << endl
            << "=(" << instance.getColCoordinate(instance.goal_locations[agent_id])
            << "," << instance.getRowCoordinate(instance.goal_locations[agent_id])
            << ")" << endl;
            exit(-1);
        }
    }
    myfile.close();
    return true;
}

bool Simulator::loadDelays(const string & fname)
{
    using namespace std;
	string line;
	std::ifstream myfile(fname.c_str());
	if (!myfile.is_open()) 
        return false;

	getline(myfile, line);
    while (!myfile.eof() && line[0] == '#') {
        getline(myfile, line);
    }

    boost::char_separator<char> sep(",");
    boost::tokenizer<boost::char_separator<char>> tok(line, sep);
    boost::tokenizer<boost::char_separator<char>>::iterator beg = tok.begin();

    int max_team_size = atoi((*beg).c_str());
    if (max_team_size < instance.num_of_agents)
    {
        std::cerr<<"Input file wrong, no enough agents in agent file";
        exit (-1);
    }

    delays.resize(instance.num_of_agents);
    // My benchmark
    for (int i = 0; i < instance.num_of_agents; i++)
    {
        boost::tokenizer<boost::char_separator<char>>::iterator beg;
        getline(myfile, line);
        while (!myfile.eof() && line[0] == '#')
        {
            getline(myfile, line);
        }
        boost::tokenizer< boost::char_separator<char> > tok(line,sep);
        beg = tok.begin();
        for (;beg != tok.end();beg++)
        {
            delays[i].push_back(atoi((*beg).c_str()));
        }
    }
    myfile.close();

    return true;
}

void Simulator::saveSimulatePaths(const string & file_name) const
{
    std::ofstream output;
    output.open(file_name);
    // header

    for (int agent = 0; agent < instance.num_of_agents; agent++)
    {
        output << "Agent " << agent << ":";
        for (const auto &state : curr_path[agent])
            output << "(" << instance.getRowCoordinate(state.location) << "," <<
                            instance.getColCoordinate(state.location) << ")->";
        output << endl;
    }
    output.close();
}

void Simulator::validateSolution() const
{
    int sum = 0;
    for (int a1 = 0; a1 < instance.num_of_agents; a1++)
    {
        if (curr_path[a1].empty())
        {
            cerr << "No solution for agent " << a1<< endl;
            //exit(-1);
        }
        else if (instance.start_locations[a1] != curr_path[a1].front().location)
        {
            cerr << "The path of agent " << a1 << " starts from location " << curr_path[a1].front().location
                << ", which is different from its start location " << instance.start_locations[a1] << endl;
            //exit(-1);
        }
        else if (instance.goal_locations[a1] != curr_path[a1].back().location)
        {
            cerr << "The path of agent " << a1 << " ends at location " << curr_path[a1].back().location
                 << ", which is different from its goal location " << instance.goal_locations[a1] << endl;
            //exit(-1);
        }
        for (int t = 1; t < (int) curr_path[a1].size(); t++ )
        {
            if (!instance.validMove(curr_path[a1][t - 1].location, curr_path[a1][t].location))
            {
                cerr << "The path of agent " << a1 << " jump from "
                     << curr_path[a1][t - 1].location << " to " << curr_path[a1][t].location
                     << " between timesteps " << t - 1 << " and " << t << endl;
                //exit(-1);
            }
        }
        sum += (int) curr_path[a1].size() - 1;
        for (int a2 = 0; a2 < instance.num_of_agents; a2++)
        {
            if (a1 >= a2 || curr_path[a2].empty())
                continue;
            int a1_ = curr_path[a1].size() <= curr_path[a2].size()? a1 : a2;
            int a2_ = curr_path[a1].size() <= curr_path[a2].size()? a2 : a1;
            int t = 1;
            for (; t < (int) curr_path[a1_].size(); t++)
            {
                if (curr_path[a1_][t].location == curr_path[a2_][t].location) // vertex conflict
                {
                    cerr << "Find a vertex conflict between agents " << a1_ << " and " << a2_ <<
                            " at location " << curr_path[a1_][t].location << " at timestep " << t << endl;
                    //exit(-1);
                }
                else if (curr_path[a1_][t].location == curr_path[a2_][t - 1].location &&
                        curr_path[a1_][t - 1].location == curr_path[a2_][t].location) // edge conflict
                {
                    cerr << "Find an edge conflict between agents " << a1_ << " and " << a2_ <<
                         " at edge (" << curr_path[a1_][t - 1].location << "," << curr_path[a1_][t].location <<
                         ") at timestep " << t << endl;
                    //exit(-1);
                }
            }
            int target = curr_path[a1_].back().location;
            for (; t < (int) curr_path[a2_].size(); t++)
            {
                if (curr_path[a2_][t].location == target)  // target conflict
                {
                    cerr << "Find a target conflict where agent " << a2_ << " (of length " << curr_path[a2_].size() - 1<<
                         ") traverses agent " << a1_ << " (of length " << curr_path[a1_].size() - 1<<
                         ")'s target location " << target << " at timestep " << t << endl;
                    //exit(-1);
                }
            }
        }
    }
}