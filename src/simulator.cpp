#include "simulator.h"
#include<boost/tokenizer.hpp>
#include "mcp.h"

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
        postmcp.build(temp);
        postmcp.simulate(temp,delays);
        postmcp.clear();
    }

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
        assert(0 <= agent_id < agents.size());
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