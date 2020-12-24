#ifndef __UTILITY_HPP_INCLUDED__
#define __UTILITY_HPP_INCLUDED__

#include <iostream>
#include <concepts>
#include <random>
#include <iterator>

namespace std
{
	template<typename _Ty>
	concept numeric = is_arithmetic_v<_Ty>;
} // namespace std

template<std::numeric _T>
[[nodiscard]]
inline _T Input(const std::string_view msg, std::istream& istr = std::cin, std::ostream& ostr = std::cout)
{
#define INPUT_STR(msg) "Input " << msg << ": "

	ostr << INPUT_STR(msg);

	_T val;
	while (!(istr >> val))
	{
		istr.clear();
		istr.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

		ostr << "Wrong input!\n" << INPUT_STR(msg);
	}

	istr.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

	return val;

#undef INPUT_STR
}

template<typename _Iter, typename _RandomGenerator>
_Iter SelectRandomly(_Iter start, _Iter end, _RandomGenerator& g)
{
	std::uniform_int_distribution<size_t> dis(0LLU, std::distance(start, end) - 1LLU);
	std::advance(start, dis(g));

	return start;
}

template<typename _Iter>
inline _Iter SelectRandomly(_Iter start, _Iter end)
{
	static std::random_device rd;
	static std::mt19937 gen(rd());
	
	return SelectRandomly(start, end, gen);
}

template<typename _Cont>
inline auto SelectRandomly(const _Cont& cont)
{
	return SelectRandomly(std::begin(cont), std::end(cont));
}

#endif /* !__UTILITY_HPP_INCLUDED__ */