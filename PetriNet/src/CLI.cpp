// This is a personal academic project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com

#include "CLI.hpp"
#include "PetriNet.hpp"
#include "Utility.hpp"

#include <fstream>

const CLI::cli_func_t CLI::FUNCS[]
{
	&CLI::run,
	&CLI::addMarker,
	&CLI::addPTEdge,
	&CLI::addTPEdge
};

void CLI::showMenu()
{
	std::cout << "0. Exit\n";
	std::cout << "1. Run\n";
	std::cout << "2. Add marker\n";
	std::cout << "3. Add PT edge\n";
	std::cout << "4. Add TP edge\n";
}

void CLI::run(PetriNet& petriNet)
{
	std::ofstream ofstr("output.txt");

	const auto duration = Input<size_t>("duration");
	petriNet.run(std::chrono::seconds(duration), ofstr);

	ofstr.close();
}

void CLI::addMarker(PetriNet& petriNet)
{
	const auto positionIdx = Input<size_t>("positionIdx");
	const auto num = Input<size_t>("number of murkers");
	petriNet.addMarkers(positionIdx, num);
}

void CLI::addPTEdge(PetriNet& petriNet)
{
	const auto positionIdx = Input<size_t>("positionIdx");
	const auto transitionIdx = Input<size_t>("transitionIdx");
	const auto num = Input<size_t>("number of murkers");
	petriNet.addPTEdge(positionIdx, transitionIdx, num);
}

void CLI::addTPEdge(PetriNet& petriNet)
{
	const auto transitionIdx = Input<size_t>("transitionIdx");
	const auto positionIdx = Input<size_t>("positionIdx");
	const auto num = Input<size_t>("number of murkers");
	petriNet.addTPEdge(transitionIdx, positionIdx, num);
}
