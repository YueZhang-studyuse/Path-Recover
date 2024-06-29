#include "mcp.h"


void MCP::simulate(vector<Path*>& paths, const vector<vector<bool>> & delays)
{
    vector<Path> path_copy; 
    path_copy.resize(paths.size());
    copy_agent_time = agent_time;
    copy_mcp = mcp;
    unfinished_agents.clear();
    for (int i = 0; i < paths.size(); i++)
    {
        if (paths[i]->size()==0){
            continue;
        }
        unfinished_agents.push_back(i);
        path_copy[i].reserve(paths[i]->size() * 2);
        assert(copy_agent_time[i] > 0);
        
        path_copy[i].push_back(paths[i]->at(no_wait_time[i][copy_agent_time[i] - 1]));
        
    }
    cout<<"Start "<<(float)clock()/(float)CLOCKS_PER_SEC<<endl;

    for (int t = 0; !unfinished_agents.empty(); t++) {
        if (t >= simulation_time)
            break;
        cout<<"Similate t = "<<t<<endl;
        auto old_size = unfinished_agents.size();


        std::vector<int> before = copy_agent_time;
        for (auto p = unfinished_agents.begin(); p != unfinished_agents.end();) 
        {
            int i = *p;
            if (t < delays.size())
                moveAgent(path_copy, paths, p, t, delays[t]);
            else
                moveAgent(path_copy, paths, p, t, delays[t-delays.size()]);
        }

        bool no_move = true;

        for (auto p = unfinished_agents.begin(); p != unfinished_agents.end();p++) 
        {
            if (copy_agent_time[*p] != before[*p])
            {
                no_move = false;
                break;
            }
            if (t < delays.size() && delays[t][*p])
            {
                no_move = false;
                break;
            }
            else if (t >= delays.size() && delays[t-delays.size()][*p])
            {
                no_move = false;
                break;
            }
        } 
        if (no_move && !unfinished_agents.empty())
        {
            cout<<"Error: No agent moves at "<<t<<endl;
            print_mcp_detail(paths);

            _exit(1);
        }       

    }

    //add unsimulated path
    for (auto p = unfinished_agents.begin(); p != unfinished_agents.end();p++)
    {
        int i = *p;
        while (copy_agent_time[i] != (int) no_wait_time[i].size())
        {
            path_copy[i].push_back(paths[i]->at(no_wait_time[i][copy_agent_time[i]])); // move
            copy_agent_time[i]++;
            //cout<<"copy size "<<copy_agent_time[i]<<"no wait size "<<(int) no_wait_time[i].size()<<endl;
        }
    }

    for (int i=0;i<paths.size();i++)
    {
        *(paths[i]) = path_copy[i];
    }

    cout<<"Simulation done at "<<(float)clock()/(float)CLOCKS_PER_SEC << endl;


    return;
}

bool MCP::moveAgent(vector<Path>& paths_copy, vector<Path*>& paths, list<int>::iterator& p, int t, const vector<bool> & delay)
{

    int i = *p;
    if (paths_copy[i].size() == t + 2)  // we have already made the movement decision for the agent
    {
        ++p;
        return false;
    }
    assert(paths_copy[i].size() == t + 1);
    assert(copy_agent_time[i] <= (int) no_wait_time[i].size());
    if (copy_agent_time[i] == (int) no_wait_time[i].size()) // the agent has reached the last location on its path
    {
        int loc = paths[i]->back().location;
        if (paths_copy[i][t].location == loc)// the agent has reached its goal location
        {
            assert(copy_mcp[loc].front().count(i)>0);
            copy_mcp[loc].front().erase(i);
            if (copy_mcp[loc].front().empty())
                copy_mcp[loc].pop_front();
            p = unfinished_agents.erase(p);
            return true;
        }
        else 
        {
            assert(false);
        }
    }

    // check mcp to determine whether the agent should move or wait
    int loc = paths[i]->at(no_wait_time[i][copy_agent_time[i]]).location;
    int previous = paths[i]->at(no_wait_time[i][copy_agent_time[i] - 1]).location;
    assert(!copy_mcp[loc].empty());
    //check delay here
    if (loc != previous && delay[i]) 
    {
        paths_copy[i].push_back(paths_copy[i].back());
        ++p;
        cout<<"find delaied "<<i<<endl;
        return false;
    }

    if (copy_mcp[previous].begin()->count(i) > 0 && copy_mcp[loc].front().count(i)>0)
    {
        paths_copy[i].push_back(paths[i]->at(no_wait_time[i][copy_agent_time[i]])); // move
        assert(copy_agent_time[i] > 0);
        
        
        if (copy_mcp[previous].front().count(i)==0)
        {
            cout<<"error: previouse location's front has no "<<i<<" :";
            for (auto item : copy_mcp[previous])
            {
                cout<<"[";
                for(auto a : item)
                    cout << a << ", ";
                cout<<"],";
            }
            cout << endl;
        }

        copy_mcp[previous].front().erase(i);
        if (copy_mcp[previous].front().empty())
            copy_mcp[previous].pop_front();
        
        copy_agent_time[i]++;
        ++p;
        // //cout <<"["<< i <<",m],";
        // decisions.push_back(Decision ("cleared", previous/map_col, previous%map_col, i,"white"));
        //decisions.push_back(Decision("finished", loc/map_col, loc%map_col, i,"grey"));
        //cout<<"- { type: finished, id: "<<i<<", x: "<<loc/map_col<<", y: "<<loc%map_col<<"}"<<endl;
        return true;
    }


    assert(copy_mcp[loc].size() > 1);

    if (copy_mcp[previous].begin()->count(i) > 0 &&  std::next(copy_mcp[loc].begin())->count(i) > 0)
    {
        // pretend this agent can move: see whether the first agent can move successfully
        paths_copy[i].push_back(paths[i]->at(no_wait_time[i][copy_agent_time[i]])); // move

        assert(copy_mcp[previous].front().count(i) != 0);
        bool mcp_pop = false;
        bool erased = copy_mcp[previous].front().count(i) > 0;//for case the agent on current location, but top do not include current agent.
        copy_mcp[previous].front().erase(i);
        if (copy_mcp[previous].front().empty()){
            copy_mcp[previous].pop_front();
            mcp_pop = true;
        }
        
        copy_agent_time[i]++;
        

        for (int first_agent:std::set<int>(copy_mcp[loc].front())){

            bool succ = false;
            if ( 
            paths_copy[first_agent].size() == t + 1 &&  // we have not made the movement decision for the first agent
            paths_copy[first_agent][t].location == loc &&
            (!delay[first_agent]))  // the fist agent is already at loc
            {
                auto p2 = std::find(unfinished_agents.begin(), unfinished_agents.end(), first_agent);
                assert(p2 != unfinished_agents.end());
                
                succ = moveAgent(paths_copy, paths, p2, t, delay);  
            }
            // if (t==0 && delay[first_agent])
            //     cout<<"delaied "<<first_agent<<endl;

            
            if (!succ)
            // this agent cannot move
            {
                paths_copy[i][t + 1] = paths_copy[i].at(t);
                copy_agent_time[i]--;
                int previous = paths[i]->at(no_wait_time[i][copy_agent_time[i] - 1]).location;

                if(mcp_pop){
                    copy_mcp[previous].push_front(set<int>());
                }

                if (erased)
                    copy_mcp[previous].front().insert(i);
                ++p;
                // cout <<"["<< i <<",rf],";
                //cout<<"result delay "<<i<<endl;
                // Decision temp("delay", previous/map_col, previous%map_col, i,"red");
                // decisions.push_back(temp);
                // cout<<"- { type: delay, id: "<<i<<", x: "<<previous/map_col<<", y: "<<previous%map_col<<"}"<<endl;;
                
                return false;
            }
        }
        ++p;
        // cout <<"["<< i <<",rm],";
        // decisions.push_back(Decision ("cleared", previous/map_col, previous%map_col, i,"white"));
        //decisions.push_back(Decision ("finished", loc/map_col, loc%map_col, i,"grey"));
        //cout<<"- { type: finished, id: "<<i<<", x: "<<loc/map_col<<", y: "<<loc%map_col<<"}"<<endl;
        

        return true;

    }
    
    paths_copy[i].push_back(paths_copy[i].back()); // stay still
    ++p; // next agent
    //decisions.push_back(Decision ("delay", loc/map_col, loc%map_col, i,"red"));
    //cout<<"- { type: delay, id: "<<i<<", x: "<<loc/map_col<<", y: "<<loc%map_col<<"}"<<endl;
    return false;
}


void MCP::build(vector<Path*>& paths)  
{
    cout<<"SimulateMCP" <<endl;
    //if (options1.debug)
    //    cout << "Start SimulateMCP ..." << endl;
    //size_t map_size = ;
    //if (options1.debug)
    //    cout << "map_size: " << map_size << endl;
    mcp.resize(map_size);
    agent_time.resize(paths.size(), 0);
    size_t max_timestep = 0;
    for (int i = 0; i<paths.size();i++)
    {
        if (paths[i]->size() ==0)
            cout<< "error; agent " << i << " has no path" <<endl;
            // _exit(1);
        max_timestep = max(max_timestep, paths[i]->size());
    }


    // Push nodes to SimulateMCP
    no_wait_time.resize(paths.size());
    delay_for.resize(paths.size(), 0);


    for (size_t t = 0; t < max_timestep; t++)
    {
        unordered_map<int, set<int>> t_occupy_mcp;

        for (int i = 0; i<paths.size();i++)
        {

            if (t < paths[i]->size())
            {
                t_occupy_mcp[paths[i]->at(t).location].insert(i);
                no_wait_time[i].push_back(t);
            }
            
        }

        for(auto& o : t_occupy_mcp){
            mcp[o.first].push_back(o.second);
        }
    }

    for (int i = 0; i<paths.size();i++)
    {
        assert(!no_wait_time[i].empty());
        if (!no_wait_time[i].empty() && no_wait_time[i][0] == 0)
        {
            agent_time[i] = 1;
        }
        else{
            assert(false);
        }
    }
}



void MCP::printAll()
{
    cout << "==================== SimulateMCP ====================" << endl;
    for (int i = 0; i < mcp.size(); i++)
    {
        if (!mcp[i].empty())
        {
            cout << "[" << i << "]: ";
            auto &last = *(--mcp[i].end());
            for (const auto& p: mcp[i])
            {
                cout<<"[";
                for(auto& o: p){
                    cout<<o<<",";
                }
                cout<<"]";
                if (&p != &last)
                    cout << "->";
                else
                    cout << endl;
            }
        }
    }
    cout << "\n================== SimulateMCP END ==================" << endl;
}


void MCP::print(int loc)
{
    cout << "==================== SimulateMCP ====================" << endl;
    if (loc < mcp.size() && !mcp[loc].empty())
    {
        cout << "[" << loc << "]: ";
        auto &last = *(--mcp[loc].end());
        for (const auto& p: mcp[loc])
        {
                cout<<"[";
                for(auto& o: p){
                    cout<<o<<",";
                }
                cout<<"]";
                if (&p != &last)
                    cout << "->";
                else
                    cout << endl;
        }
    }
    cout << "\n================== SimulateMCP END ==================" << endl;
}


void MCP::printAgentTime(int num_agents)
{
    cout << "==================== Time ====================" << endl;
    for (int i = 0; i < num_agents; i++)
    {
        cout << "Agent " << i << ": " << agent_time[i] << endl;
    }
    cout << "================== End Time ==================" << endl;
}


void MCP::printAgentNoWaitTime(int num_agents)
{
    cout << "==================== Time ====================" << endl;
    for (int i = 0; i < num_agents; i++)
    {
        cout << "Agent " << i << ": ";
        for (int t = 0; t < no_wait_time[i].size(); t++)
            cout << no_wait_time[i][t] << ", ";
        cout << endl;
    }
    cout << "================== End Time ==================" << endl;
}
