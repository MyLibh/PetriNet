// This is a personal academic project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com

#include "PetriNet.hpp"
#include "Utility.hpp"

#include <algorithm>
#include <chrono>
#include <thread>
#include <set>
#include <string>
#include <fstream>

namespace detail
{
	inline auto TimeNow() noexcept
	{
		return std::chrono::system_clock::now();
	}

	inline auto TimePassed(const time_point_t& timePoint) noexcept
	{
		return std::chrono::duration_cast<std::chrono::milliseconds>(TimeNow() - timePoint).count();
	}

	template<typename _T>
	concept timed_object  = requires (_T x) { x.getTimePoint(); x.getDuration(); };

	template<timed_object _T>
	inline bool DurationHasPassed(const _T& obj) noexcept
	{
		return TimePassed(obj.getTimePoint()) >= obj.getDuration().count();
	}

	inline std::string MakeLogStr(const time_point_t& timePoint, const size_t markerIdx, const std::string_view msg)
	{
		return '{' + std::to_string(detail::TimePassed(timePoint)) + "ms}<" + std::to_string(markerIdx) + ">[" + msg.data() + "]\n";
	}
} // namespace detail

void PetriNet::generateMarkerDuration(Marker& marker) noexcept
{
	static std::random_device rd;
	static std::mt19937 mt(rd());
	static std::uniform_int_distribution<size_t> dist(0, 1000);

	marker.setTimePoint(detail::TimeNow());
	marker.setDuration(std::chrono::milliseconds(dist(mt)));
}

void PetriNet::generateMarkersDuration() noexcept
{
	std::for_each(std::begin(m_markers), std::end(m_markers), [&](Marker& marker) { generateMarkerDuration(marker); });
}

std::vector<size_t> PetriNet::getReadyInputMarkers(const size_t transitionIdx)
{
	std::vector<size_t> inputMarkers;
	const auto& inputs = m_transitions.at(transitionIdx).getInputs();
	for (size_t i{}; i < std::size(inputs); ++i)
		if (inputs[i])
		{
			std::lock_guard<std::mutex> lock(m_markers_lock);
			std::lock_guard<std::mutex> queue_lock(m_queue_lock);
			for (const auto& marker : m_markers)
				if (marker.getPosition() == i && 
					std::find_if(std::begin(m_updateQueue), std::end(m_updateQueue),
						[&](const size_t markerIdx) { return m_markers.at(markerIdx).getId() == marker.getId(); }) != std::end(m_updateQueue))
					inputMarkers.emplace_back(marker.getId());
		}

	return inputMarkers;
}

void PetriNet::deleteMarkers(std::vector<size_t>& markers, std::ofstream& ofstr)
{
	std::lock_guard<std::mutex> lock(m_markers_lock);
	for (const auto& markerToRemove : markers)
	{
		auto it = std::find_if(std::begin(m_markers), std::end(m_markers), [&](const Marker& marker) { return marker.getId() == markerToRemove; });
		if (it != std::end(m_markers))
		{
			ofstr << detail::MakeLogStr(m_timePoint, markerToRemove, "-");

			m_positions.at(it->getPosition()).removeMarkers(1);

			const size_t pos = std::distance(std::begin(m_markers), it);
			m_updateQueue.erase(std::find(std::begin(m_updateQueue), std::end(m_updateQueue), pos));
			std::for_each(std::begin(m_updateQueue), std::end(m_updateQueue), [&](size_t& markerIdx) { if (markerIdx > pos) --markerIdx; });

			m_markers.erase(it);
		}
	}

	markers.clear();
}

void PetriNet::moveMarkers(const size_t transitionIdx, std::ofstream& ofstr)
{
	auto readyInputMarkers = getReadyInputMarkers(transitionIdx);
	const auto& outputs = m_transitions.at(transitionIdx).getOutputs();

	std::vector<size_t> possiblePositions;
	for (size_t i{}; i < std::size(outputs); ++i)
		if (outputs[i])
			possiblePositions.insert(std::end(possiblePositions), outputs[i], i);

	if (possiblePositions.empty())
		deleteMarkers(readyInputMarkers, ofstr);

	std::lock_guard<std::mutex> lock(m_markers_lock);
	while (!possiblePositions.empty() && !readyInputMarkers.empty())
	{
		const auto outputIt = SelectRandomly(possiblePositions);

		auto it = std::find_if(std::begin(m_markers), std::end(m_markers), [&](const Marker& marker) { return marker.getId() == readyInputMarkers.front(); });
		if (it != std::end(m_markers))
		{
			const auto position = it->getPosition();
			const auto msg = std::to_string(position) + " -> " + std::to_string(*outputIt) + " | " + std::to_string(transitionIdx);
			ofstr << detail::MakeLogStr(m_timePoint, readyInputMarkers.front(), msg);

			m_positions.at(position).removeMarkers(1);
			it->setPosition(*outputIt);
			m_positions.at(*outputIt).addMarkers(1);

			generateMarkerDuration(*it);

			{
				std::lock_guard queue_lock(m_queue_lock);
				m_updateQueue.erase(std::remove_if(std::begin(m_updateQueue), std::end(m_updateQueue),
					[&](const size_t markerIdx) { return m_markers.at(markerIdx).getId() == readyInputMarkers.front(); }), std::end(m_updateQueue));
			}

			readyInputMarkers.erase(std::begin(readyInputMarkers));
			possiblePositions.erase(outputIt);
		}
	}
}

void PetriNet::updateMarker()
{
	while (!m_finished && !m_markers.empty() && !detail::DurationHasPassed(*this))
	{
		while (m_main);
		m_queue_ready = false;
		
		std::lock_guard lock(m_markers_lock);
		std::lock_guard queue_lock(m_queue_lock);
		for (size_t i{}; i < m_markers.size(); ++i)
		{
			if (std::find(std::begin(m_updateQueue), std::end(m_updateQueue), i) != std::end(m_updateQueue))
				continue;

			if (detail::DurationHasPassed(m_markers.at(i)))
				m_updateQueue.push_back(i);
		}

		m_queue_ready = !m_updateQueue.empty();
	}

	m_queue_ready = true;
}

void PetriNet::updateTransitions(std::ofstream& ofstr)
{
	std::lock_guard lock(m_markers_lock);
	std::lock_guard queue_lock(m_queue_lock);

	std::for_each(std::begin(m_markers), std::end(m_markers), [](Marker& marker) { marker.clearTransitions(); });

	for (const auto& markerIdx : m_updateQueue)
	{
		auto& marker = m_markers.at(markerIdx);
		const auto markerPositionIdx = marker.getPosition();
		const auto& position = m_positions.at(markerPositionIdx);

		const auto& out = position.getOut();
		for (size_t i{}; i < std::size(out); ++i)
			if (out[i])
				marker.addTransition(i);

		if (marker.getTransitions().empty())
			ofstr << detail::MakeLogStr(m_timePoint, marker.getId(), "X");
	}
}

void PetriNet::performTransitions(std::ofstream& ofstr)
{
	while (!m_finished && !m_markers.empty() && !detail::DurationHasPassed(*this))
	{
		while (!m_queue_ready);
		m_main = true;

		updateTransitions(ofstr);
		m_markers_lock.lock();
		m_queue_lock.lock();

		std::set<size_t> used;
		std::vector<std::thread> threads;
		for (const auto& markerIdx : m_updateQueue)
		{
			const auto& transitions = m_markers.at(markerIdx).getTransitions();

			std::vector<size_t> possibleTransitions;
			std::copy_if(std::begin(transitions), std::end(transitions), std::back_inserter(possibleTransitions),
				[&](const size_t idx) { return !used.contains(idx); });
			if (!possibleTransitions.empty())
			{
				const auto it = SelectRandomly(possibleTransitions);
				threads.emplace_back([&, transitionIdx = *it]() { moveMarkers(transitionIdx, ofstr); });

				used.insert(*it);
			}

			if (std::size(threads) + 1 == std::thread::hardware_concurrency())
				break;
		}
		m_queue_lock.unlock();
		m_markers_lock.unlock();

		if (used.empty())
			m_finished = true;

		m_main = false;

		for (auto& thread : threads)
			thread.join();
	}

	if (m_finished)
		std::cerr << "All markers are blocked\n";
	else if (detail::DurationHasPassed(*this))
		std::cerr << "Time is over\n";
}

void PetriNet::run(std::chrono::seconds duration, std::ofstream& ofstr)
{
	generateMarkersDuration();

	m_timePoint = detail::TimeNow();
	m_duration = duration;

	std::thread updater([&]() { updateMarker(); });
	std::thread transiter([&]() { performTransitions(ofstr); });

	updater.join();
	transiter.join();
}
