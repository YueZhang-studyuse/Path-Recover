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

    struct Node
	{
		int location;
		int value;

		Node() = default;
		Node(int location, int value) : location(location), value(value) {}
		// the following is used to compare nodes in the OPEN list
		struct compare_node
		{
			// returns true if n1 > n2 (note -- this gives us *min*-heap).
			bool operator()(const Node& n1, const Node& n2) const
			{
				return n1.value >= n2.value;
			}
		};  // used by OPEN (heap) to compare nodes (top of the heap has min f-val, and then highest g-val)
	};

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