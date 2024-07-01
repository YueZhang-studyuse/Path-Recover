#pragma once

#include "common.h"
#include "instance.h"

class MCP {
public:

    int simulation_time = MAX_TIMESTEP;
    int max_delay_time = MAX_TIMESTEP;

    vector<int> to_go;

    MCP(int map_size):map_size(map_size){};
    void saveResults(const string &fileName);

    void build(vector<Path*>& paths);
    void clear(void)
    {
        mcp.clear();
        agent_time.clear();
        no_wait_time.clear();
        delay_for.clear();
    };
    void printAll(void);
    void print(int loc);
    void printAgentTime(int num_agents);
    void printAgentNoWaitTime(int num_agents);
    void simulate(vector<Path*>& paths, const vector<vector<bool>> & delays);

    std::set<int> delayed_agents;
    int count_pairs(vector<Path>& paths){
        std::set<std::set<int>> colliding_pairs; 
        int count = 0;
        for (int i = 0; i < paths.size(); i++)
        {
            for (int j = i + 1; j < paths.size(); j++)
            {
                for (int t = 0; t < paths[i].size(); t++)
                {
                    if (paths[i][t].location == paths[j][t].location)
                    {
                        colliding_pairs.insert({i,j});
                        count++;
                        break;
                    }
                }
            }
        }
        return colliding_pairs.size();
    };

    void print_mcp_detail(vector<Path*>& paths){
            for (auto p = unfinished_agents.begin(); p != unfinished_agents.end();p++) {
                int i = *p;

                int loc = paths[i]->at(no_wait_time[i][copy_agent_time[i]]).location;
                int prev = paths[i]->at(no_wait_time[i][copy_agent_time[i]]-1).location;

                cout<<"Agent "<<i<< "at " << prev <<" for "<<loc;
                for (auto l : copy_mcp[loc]){
                    cout <<" [";
                    for (auto a : l){
                        cout<<a<<",";
                    }
                    cout<<"]";
                }
                cout<<endl;
            }

            for (int i = 0 ; i < mcp.size();i++){
                if (mcp[i].size() == 0 )
                    continue;
                cout <<"SimulateMCP "<< i <<":";
                for (auto m : mcp[i]){
                    cout<<"[";
                    for (auto a : m) {
                        cout << a << ",";
                    }
                    cout<<"]";
                }
                cout<<endl;
            }
    }
private:
    int map_size;

    typedef list<set<int>> Occupy;
    vector<Occupy> mcp;
    vector<int> agent_time;
    vector<vector<int>> no_wait_time;
    vector<Occupy> copy_mcp;
    vector<int> copy_agent_time;
    list<int> unfinished_agents;
    vector<int> delay_for;

    bool moveAgent(vector<Path>& paths_copy, vector<Path*>& paths, list<int>::iterator& p, int t, const vector<bool> & delay);
};