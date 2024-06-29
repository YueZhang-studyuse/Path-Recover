#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>
#include "simulator.h"


namespace po = boost::program_options;

po::variables_map vm;

int main(int argc, char **argv)
{
    //test instance 
    //./pr -m ../instance/random/random-32-32-20.map -a ../instance/random/scen-random/random-32-32-20-random-1.scen -k 50 --inputPath ../cbs_results/random/agent-50-i-0.txt --inputDelay ../instance/delay-long/delay-0.015-1.txt --outputPath path.txt

    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("map,m", po::value<string>()->required(), "input file for map")
		    ("agents,a", po::value<string>()->required(), "input file for agents")
		    ("agentNum,k", po::value<int>()->default_value(0), "number of agents")
        ("inputPath", po::value<std::string>()->required(), "input path file name")
        ("inputDelay", po::value<std::string>()->required(), "input delay file name")
        ("output,o", po::value<std::string>()->default_value("./test"), "output file name (no extension)")
        ("outputPath", po::value<std::string>(), "output path file name")
        ("DelayTime", po::value<int>(), "timesteps of running simulation of delays")
        ("planTimeLimit,t", po::value<int>()->default_value(60), "the time limit for planner in seconds")
        ("delayPolicy", po::value<int>()->default_value(1), "execution policy with delays ( 1--MCP")
        ("recoverSolver", po::value<int>()->default_value(1), "recover solver after delays ( 1--MCP");

    po::store(po::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return 1;
    }

    po::notify(vm);

    Instance instance(vm["map"].as<string>(), vm["agents"].as<string>(), vm["agentNum"].as<int>());

    Simulator simulator(instance, vm["delayPolicy"].as<int>(), vm["recoverSolver"].as<int>(), vm["planTimeLimit"].as<int>());
    if (!simulator.loadPaths(vm["inputPath"].as<string>()))
    {
        cerr << "Input Path " << vm["inputPath"].as<string>() << " not found." << endl;
        exit(-1);
    }

    if (!simulator.loadDelays(vm["inputDelay"].as<string>()))
    {
        cerr << "Input Delay " << vm["inputDelay"].as<string>() << " not found." << endl;
        exit(-1);
    }

    simulator.simulation();

    if (vm.count("outputPath"))
      simulator.saveSimulatePaths(vm["outputPath"].as<string>());

    return 0;
}
