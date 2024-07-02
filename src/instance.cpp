#include<boost/tokenizer.hpp>
#include <algorithm>    // std::shuffle
#include <random>      // std::default_random_engine
#include <chrono>       // std::chrono::system_clock
#include "instance.h"


Instance::Instance(const string& map_fname, const string& agent_fname, int num_of_agents):
	map_fname(map_fname), agent_fname(agent_fname), num_of_agents(num_of_agents)
{
	bool succ = loadMap();
	if (!succ)
	{
        cerr << "Map file " << map_fname << " not found." << endl;
        exit(-1);
	}

	succ = loadAgents();
	if (!succ)
	{
        cerr << "Agent file " << agent_fname << " not found." << endl;
        exit(-1);
	}
    //printAgents();
    //printMap();
    computeHeuristics();
    //printHeuristic();
}

bool Instance::loadMap()
{
	using namespace boost;
	using namespace std;
	ifstream myfile(map_fname.c_str());
	if (!myfile.is_open())
		return false;
	string line;
	tokenizer< char_separator<char> >::iterator beg;
	getline(myfile, line);
	if (line[0] == 't') // Nathan's benchmark
	{
		char_separator<char> sep(" ");
		getline(myfile, line);
		tokenizer< char_separator<char> > tok(line, sep);
		beg = tok.begin();
		beg++;
		num_of_rows = atoi((*beg).c_str()); // read number of rows
		getline(myfile, line);
		tokenizer< char_separator<char> > tok2(line, sep);
		beg = tok2.begin();
		beg++;
		num_of_cols = atoi((*beg).c_str()); // read number of cols
		getline(myfile, line); // skip "map"
	}
	else // my benchmark
	{
		cerr << "Map file " << agent_fname << " wrong format." << endl;
        exit(-1);
	}
	map_size = num_of_cols * num_of_rows;
	my_map.resize(map_size, false);
	// read map (and start/goal locations)
	for (int i = 0; i < num_of_rows; i++) {
		getline(myfile, line);
		for (int j = 0; j < num_of_cols; j++) {
			my_map[linearizeCoordinate(i, j)] = (line[j] != '.');
		}
	}
	myfile.close();
	return true;
}


void Instance::printMap() const
{
	for (int i = 0; i< num_of_rows; i++)
	{
		for (int j = 0; j < num_of_cols; j++)
		{
			if (this->my_map[linearizeCoordinate(i, j)])
				cout << '@';
			else
				cout << '.';
		}
		cout << endl;
	}
}

bool Instance::loadAgents()
{
	using namespace std;
	using namespace boost;

	string line;
	ifstream myfile (agent_fname.c_str());
	if (!myfile.is_open()) 
	return false;

	getline(myfile, line);
	if (line[0] == 'v') // Nathan's benchmark
	{
		if (num_of_agents == 0)
		{
			cerr << "The number of agents should be larger than 0" << endl;
			exit(-1);
		}
		start_locations.resize(num_of_agents);
		goal_locations.resize(num_of_agents);
		char_separator<char> sep("\t");
		for (int i = 0; i < num_of_agents; i++)
		{
			getline(myfile, line);
			tokenizer< char_separator<char> > tok(line, sep);
			tokenizer< char_separator<char> >::iterator beg = tok.begin();
			beg++; // skip the first number
			beg++; // skip the map name
			beg++; // skip the columns
			beg++; // skip the rows
				   // read start [row,col] for agent i
			int col = atoi((*beg).c_str());
			beg++;
			int row = atoi((*beg).c_str());
			start_locations[i] = linearizeCoordinate(row, col);
			// read goal [row,col] for agent i
			beg++;
			col = atoi((*beg).c_str());
			beg++;
			row = atoi((*beg).c_str());
			goal_locations[i] = linearizeCoordinate(row, col);
		}
	}
	else // My benchmark
	{
		cerr << "Agent file " << agent_fname << " wrong format." << endl;
        exit(-1);
	}
	myfile.close();
	return true;

}


void Instance::printAgents() const
{
  for (int i = 0; i < num_of_agents; i++) 
  {
    cout << "Agent" << i << " : S=(" << getRowCoordinate(start_locations[i]) << "," << getColCoordinate(start_locations[i]) 
				<< ") ; G=(" << getRowCoordinate(goal_locations[i]) << "," << getColCoordinate(goal_locations[i]) << ")" << endl;
  }
}


list<int> Instance::getNeighbors(int curr) const
{
	list<int> neighbors;
	int candidates[4] = {curr + 1, curr - 1, curr + num_of_cols, curr - num_of_cols};
	for (int next : candidates)
	{
		if (validMove(curr, next))
			neighbors.emplace_back(next);
	}
	return neighbors;
}

void Instance::computeHeuristics()
{
    int h_size  = 0;
    cout<<"computing all pair"<<endl;
    heuristic.resize(my_map.size());

    for (int i = 0; i < heuristic.size(); i++)
    {
        if (my_map[i])
            continue;
        //heuristic[i] = std::vector<int>(heuristic.size()-i, MAX_TIMESTEP);
        heuristic[i] = std::vector<int>(heuristic.size(), MAX_TIMESTEP);
        h_size++;
    }
    for (int i = 0; i < heuristic.size(); i++)
    {
        if (my_map[i])
            continue;
        // generate a heap that can save nodes (and a open_handle)
        boost::heap::pairing_heap< Node, boost::heap::compare<Node::compare_node> > heap;

        Node root(i, 0); //compute every node to i
        heuristic[i][i] = 0;

        heap.push(root);  // add root to heap
        while (!heap.empty())
        {
            Node curr = heap.top();
            heap.pop();
            for (int next_location : getNeighbors(curr.location))
            {
                if (heuristic[i][next_location] > curr.value + 1)
                {
                    heuristic[i][next_location] = curr.value + 1;
                    Node next(next_location, curr.value + 1);
                    heap.push(next);
                }
            }
        }
    }

}

void Instance::initGuidanceHeuristics() const
{
	guidance_heuristic.clear();
	guidance_heuristic.resize(num_of_agents);
	second_guidance_heuristic.resize(num_of_agents);
	OPEN.clear();
	for (size_t i = 0; i < num_of_agents; i++) 
	{
		//cout<<"agent "<<i<<endl;
		guidance_heuristic[i] = std::vector<int>(my_map.size(), MAX_TIMESTEP);
		second_guidance_heuristic[i] = std::vector<int>(my_map.size(), MAX_TIMESTEP);

		OPEN.push_back(std::queue<Node>());
		for (int t = 0; t < guidance_path[i].size();t++)
		{
			int loc = guidance_path[i][t];
			//cout<<"loc "<<loc;
			Node root(loc, 0); //compute every node to i
        	guidance_heuristic[i][loc] = 0;
			second_guidance_heuristic[i][loc] = abs((int)guidance_path[i].size()-1-t);
        	OPEN[i].push(root);  // add root to heap
		}
	}
}

int Instance::getTimeIndependentHeuristics(int agent, int loc) const
{
	//cout<<"checking "<<loc<<endl;
	if (guidance_heuristic[agent][loc] < MAX_TIMESTEP)
		return guidance_heuristic[agent][loc];
	//expand by bfs
	while (!OPEN[agent].empty()) 
    {
        Node n = OPEN[agent].front();
		//cout<<"expanding "<<n.location<<endl;
        OPEN[agent].pop();
		int d_n = guidance_heuristic[agent][n.location];
		for (int next_location : getNeighbors(n.location))
		{
			int d_m = guidance_heuristic[agent][next_location];
			if (d_n + 1 >= d_m) continue;
			guidance_heuristic[agent][next_location] = d_n + 1;
			second_guidance_heuristic[agent][next_location] = second_guidance_heuristic[agent][n.location];
			Node next(next_location, d_n + 1);
			OPEN[agent].push(next);
		}
        if (n.location == loc)
			return d_n;
    }
    return MAX_TIMESTEP;
}

void Instance::printHeuristic() const
{
    for (int i = 0; i < num_of_agents; i++) 
    {
        cout << "Agent" << i << endl;
        for (int j = 0; j < heuristic[i].size(); j++)
            cout<<"(loc: "<<j<<", h: "<<heuristic[i][j]<<") ";
        cout<<endl;
    }
}

bool Instance::hasCollision(const Path& p1, const Path& p2) const
{
    int t = 1;
    for (; t < (int) min(p1.size(), p2.size()); t++)
    {
        if (p1[t].location == p2[t].location) // vertex conflict
        {
            return true;
        }
        else if (p1[t].location == p2[t-1].location and p1[t-1].location == p2[t].location) // edge conflict
        {
            return true;
        }
    }
    if (p1.size() == p2.size()) return false;

    auto p = p1.size() > p2.size()? p1 : p2;
    auto target = p1.size() < p2.size()? p1.back().location : p2.back().location;
    for (; t < (int) p.size(); t++)
    {
        if (p[t].location == target)  // target conflict
        {
            return true;
        }
    }
    return false;
}