// This is a personal academic project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com

#include "PetriNet.hpp"
#include "Utility.hpp"
#include "CLI.hpp"

#include <fstream>

auto main() -> int
{
	std::ifstream ifstr("input.txt");
	std::cin.rdbuf(ifstr.rdbuf());

	const auto positions = Input<size_t>("Positions");
	const auto transitions = Input<size_t>("Transitions");
	PetriNet petriNet(positions, transitions);

	while (true)
	{
		CLI::showMenu();
		if (auto opt = Input<size_t>("Option"); !opt)
			break;
		else if (--opt < std::size(CLI::FUNCS))
		{
			CLI::FUNCS[opt](petriNet);
			if (!opt)
				break;
		}
		else
			std::cerr << "Wrong option!\n";
	}

	ifstr.close();

	return 0;
}