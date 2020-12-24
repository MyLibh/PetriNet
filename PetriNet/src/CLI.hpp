#ifndef __CLI_HPP_INCLUDED__
#define __CLI_HPP_INCLUDED__

class PetriNet;

class CLI
{
public:
	using cli_func_t = void (*)(PetriNet&);

	static const cli_func_t FUNCS[4];

private:
	static void run(PetriNet& petriNet);

	static void addMarker(PetriNet& petriNet);

	static void addPTEdge(PetriNet& petriNet);

	static void addTPEdge(PetriNet& petriNet);

public:
	static void showMenu();
};

#endif /* !__CLI_HPP_INCLUDED__ */