#ifndef __MARKER_HPP_INCLUDED__
#define __MARKER_HPP_INCLUDED__

#include "Predefs.hpp"

#include <vector>

class Marker
{
private:
	inline static auto ID{ 0LLU };

public:
	inline explicit Marker(const size_t position) noexcept :
		m_blocked{},
		m_id(Marker::ID++),
		m_pos(position),
		m_duration{}
	{}

	[[nodiscard]] inline constexpr auto getId() const noexcept { return m_id; }

	[[nodiscard]] inline constexpr auto getPosition() const noexcept { return m_pos; }

	[[nodiscard]] inline const auto& getTransitions() const noexcept { return m_transitions; }

	[[nodiscard]] inline constexpr auto getTimePoint() const noexcept { return m_timePoint; }

	[[nodiscard]] inline constexpr auto getDuration() const noexcept { return m_duration; }

	inline constexpr bool isBlocked() const noexcept { return m_blocked; }

	void setBlocked(const bool value) noexcept { m_blocked = value; }

	void setPosition(const size_t position) noexcept { m_pos = position; }

	void setTimePoint(const time_point_t& timePoint) noexcept { m_timePoint = timePoint; }

	void setDuration(const duration_t& duration) noexcept { m_duration = duration; }

	inline void addTransition(const size_t idx) { m_transitions.emplace_back(idx); }

	inline void clearTransitions() noexcept { m_transitions.clear(); };

private:
	bool                m_blocked;
	size_t              m_id;
	size_t              m_pos;
	std::vector<size_t> m_transitions;
	time_point_t        m_timePoint;
	duration_t          m_duration;
};

#endif /* !__MARKER_HPP_INCLUDED__ */