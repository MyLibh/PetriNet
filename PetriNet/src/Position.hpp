#ifndef __POSITION_HPP_INCLUDED__
#define __POSITION_HPP_INCLUDED__

#include <vector>

class Position
{
public:
	inline Position() noexcept :
		m_markers{}
	{}

	inline explicit Position(const size_t transitions) :
		m_markers{},
		m_out(transitions)
	{}

	inline Position(std::vector<size_t>&& out) noexcept :
		m_markers{},
		m_out(std::move(out))
	{}

	[[nodiscard]] inline constexpr auto getNumOfMarkers() const noexcept { return m_markers; }

	[[nodiscard]] inline const auto& getOut() const noexcept { return m_out; }

	inline void addMarkers(const size_t num) noexcept { m_markers += num; }

	inline void removeMarkers(const size_t num) noexcept { m_markers -= num; }

	inline void updateTransition(const size_t idx, const size_t num) { m_out.at(idx) += num; }

private:
	size_t              m_markers;
	std::vector<size_t> m_out;
};

#endif /* !__POSITION_HPP_INCLUDED__ */