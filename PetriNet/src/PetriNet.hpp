#ifndef __PETRI_NET_HPP_INLCLUDED__
#define __PETRI_NET_HPP_INLCLUDED__

#include "Marker.hpp"
#include "Position.hpp"
#include "Transition.hpp"

#include <atomic>
#include <mutex>
#include <iosfwd>

class PetriNet
{
private:
	void generateMarkerDuration(Marker& marker) noexcept;

	void generateMarkersDuration() noexcept;

	void printEndCondition();

	std::vector<size_t> getReadyInputMarkers(const size_t transitionIdx);

	void deleteMarkers(std::vector<size_t>& markers, std::ofstream& ofstr);

	void moveMarkers(const size_t transitionIdx, std::ofstream& ofstr);

	void updateMarkers();

	void updateTransitions(std::ofstream& ofstr);

	void performTransitions(std::ofstream& ofstr);

public:
	inline PetriNet(const size_t positions, const size_t transitions) :
		m_positions(positions, Position(transitions)),
		m_transitions(transitions, Transition(positions)),
		m_duration{}
	{}

	[[nodiscard]] inline constexpr auto getTimePoint() const noexcept { return m_timePoint; }

	[[nodiscard]] inline constexpr auto getDuration() const noexcept { return m_duration; }

	void run(std::chrono::seconds duration, std::ofstream& ofstr);

	inline void addMarkers(const size_t positionIdx, const size_t num)
	{
		m_positions.at(positionIdx).addMarkers(num);
		for (size_t i{}; i < num; ++i)
			m_markers.emplace_back(positionIdx);
	}

	inline void addPTEdge(const size_t positionIdx, const size_t transitionIdx, const size_t num)
	{
		m_positions.at(positionIdx).updateTransition(transitionIdx, num);
		m_transitions.at(transitionIdx).updateInput(positionIdx, num);
	}

	inline void addTPEdge(const size_t transitionIdx, const size_t positionIdx, const size_t num) { m_transitions.at(transitionIdx).updateOutput(positionIdx, num); }

private:
	std::vector<Marker>     m_markers;
	std::vector<Position>   m_positions;
	std::vector<Transition> m_transitions;
	std::vector<size_t>     m_updateQueue;

	std::mutex       m_markers_lock;
	std::mutex       m_queue_lock;
	std::atomic_bool m_all_blocked;

	time_point_t m_timePoint;
	duration_t   m_duration;
};

#endif /* !__PETRI_NET_HPP_INLCLUDED__ */