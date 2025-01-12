#pragma once
#include "common.h"

class Instance 
{
public:
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

	int num_of_cols;
	int num_of_rows;
	int map_size;

    int num_of_agents;
    vector<int> start_locations;
    vector<int> goal_locations;

	int guidance_mode = 2; //1--time-independent path

    mutable vector<vector<int>> guidance_path;

	Instance()=default;
	Instance(const string& map_fname, const string& agent_fname, int num_of_agents = 0);

	string getMapFile() const {return map_fname;};
    vector<int> getStarts() const {return start_locations;};
    vector<int> getGoals() const {return goal_locations;};


    inline bool isObstacle(int loc) const { return my_map[loc]; }
    inline bool validMove(int curr, int next) const
    {
        if (next < 0 || next >= map_size)
            return false;
        if (my_map[next])
            return false;
        return getManhattanDistance(curr, next) < 2;
    }
    list<int> getNeighbors(int curr) const;


    inline int linearizeCoordinate(int row, int col) const { return ( this->num_of_cols * row + col); }
    inline int getRowCoordinate(int id) const { return id / this->num_of_cols; }
    inline int getColCoordinate(int id) const { return id % this->num_of_cols; }
    inline pair<int, int> getCoordinate(int id) const { return make_pair(id / this->num_of_cols, id % this->num_of_cols); }
    inline int getCols() const { return num_of_cols; }

    inline int getManhattanDistance(int loc1, int loc2) const
    {
        int loc1_x = getRowCoordinate(loc1);
        int loc1_y = getColCoordinate(loc1);
        int loc2_x = getRowCoordinate(loc2);
        int loc2_y = getColCoordinate(loc2);
        return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y);
    }

    static inline int getManhattanDistance(const pair<int, int>& loc1, const pair<int, int>& loc2)
    {
        return abs(loc1.first - loc2.first) + abs(loc1.second - loc2.second);
    }

    int getAllpairDistance(int loc1, int loc2) const
	{
		return heuristic[loc1][loc2];
	}

	void initGuidanceHeuristics() const;

	int getTimeIndependentHeuristics(int agent, int loc) const;

    int getGuidanceDistance(int agent, int loc, int t) const
	{
		if (guidance_mode == 1)
		{
			if (guidance_path.empty() || guidance_path[agent].size() <= t)
				return -1;
			int h = getAllpairDistance(loc,guidance_path[agent][t]);
			return h;
		}
		else
		{
			int h = getTimeIndependentHeuristics(agent,loc);
			if (h != MAX_TIMESTEP)
				return h;
			return -1;
		}
	}

	int getSecondGuidanceDistance(int agent, int loc) const
	{
		return second_guidance_heuristic[agent][loc];
	}

	int getDegree(int loc) const
	{
		assert(loc >= 0 && loc < map_size && !my_map[loc]);
		int degree = 0;
		if (0 <= loc - num_of_cols && !my_map[loc - num_of_cols])
			degree++;
		if (loc + num_of_cols < map_size && !my_map[loc + num_of_cols])
			degree++;
		if (loc % num_of_cols > 0 && !my_map[loc - 1])
			degree++;
		if (loc % num_of_cols < num_of_cols - 1 && !my_map[loc + 1])
			degree++;
		return degree;
	}

	int getDefaultNumberOfAgents() const { return num_of_agents; }
	string getInstanceName() const { return agent_fname; }

    void computeHeuristics();

	bool hasCollision(const Path& p1, const Path& p2) const;

private:
	  // int moves_offset[MOVE_COUNT];
	  vector<bool> my_map;
	  string map_fname;
	  string agent_fname;

      vector<vector<int>> heuristic;
	  mutable vector<vector<int>> guidance_heuristic; //agent id <loc>
	  mutable vector<vector<int>> second_guidance_heuristic; //agent id <loc>
	  mutable vector<std::queue<Node>> OPEN;

	  bool loadMap();
	  void printMap() const;
      void printAgents() const;
      void printHeuristic() const;
	  bool loadAgents();
};