#ifndef __TRANSITION_HPP_INCLUDED__
#define __TRANSITION_HPP_INCLUDED__

#include <vector>

class Transition
{
private:
	inline void update(std::vector<size_t>& vec, const size_t idx, const size_t num) { vec.at(idx) += num; }

public:
	inline Transition() noexcept = default;

	inline Transition(const std::size_t positions) :
		m_inputs(positions),
		m_outputs(positions)
	{}

	inline Transition(std::vector<size_t>&& inputs, std::vector<size_t>&& outputs) noexcept :
		m_inputs(std::move(inputs)),
		m_outputs(std::move(outputs))
	{}

	[[nodiscard]] inline const auto& getInputs() const noexcept { return m_inputs; }

	[[nodiscard]] inline const auto& getOutputs() const noexcept { return m_outputs; }

	inline void updateInput(const size_t idx, const size_t num) { update(m_inputs, idx, num); }

	inline void updateOutput(const size_t idx, const size_t num) { update(m_outputs, idx, num); }

private:
	std::vector<size_t> m_inputs;
	std::vector<size_t> m_outputs;
};

#endif /* !__TRANSITION_HPP_INCLUDED__ */