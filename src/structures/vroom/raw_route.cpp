/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/raw_route.h"

// #define TOUR_DEBUG

#ifdef TOUR_DEBUG
  #include <iostream>
#endif

namespace vroom {

RawRoute::RawRoute(const Input& input, Index i, unsigned amount_size)
  : _zero(amount_size),
    _fwd_peaks(2, _zero),
    _bwd_peaks(2, _zero),
    _delivery_margin(input.vehicles[i].capacity),
    _pickup_margin(input.vehicles[i].capacity),
    vehicle_rank(i),
    has_start(input.vehicles[i].has_start()),
    has_end(input.vehicles[i].has_end()),
    capacity(input.vehicles[i].capacity) {
}

void RawRoute::set_route(const Input& input, const std::vector<Index>& r) {
  route = r;
  update_amounts(input);
}

bool RawRoute::empty() const {
  return route.empty();
}

std::size_t RawRoute::size() const {
  return route.size();
}

void RawRoute::update_amounts(const Input& input) {
  auto step_size = route.size() + 2;
  _fwd_pickups.resize(route.size());
  _fwd_deliveries.resize(route.size());
  _bwd_deliveries.resize(route.size());
  _bwd_pickups.resize(route.size());
  _pd_loads.resize(route.size());
  _nb_pickups.resize(route.size());
  _nb_deliveries.resize(route.size());

  _current_loads.resize(step_size);
  _fwd_peaks.resize(step_size);
  _bwd_peaks.resize(step_size);

  _preceding_tour_travel_time.resize(route.size() + 1);
  _following_tour_travel_time.resize(route.size() + 1);

  if (route.empty()) {
    // So that check in is_valid_addition_for_capacity is consistent
    // with empty routes.
    std::fill(_fwd_peaks.begin(), _fwd_peaks.end(), _zero);
    std::fill(_bwd_peaks.begin(), _bwd_peaks.end(), _zero);
    // So that check against break max_load and margins computations
    // are consistent with empty routes.
    std::fill(_current_loads.begin(), _current_loads.end(), _zero);
    std::fill(_preceding_tour_travel_time.begin(), _preceding_tour_travel_time.end(), 0);
    std::fill(_following_tour_travel_time.begin(), _following_tour_travel_time.end(), 0);
    return;
  }

  Amount current_pickups(_zero);
  Amount current_deliveries(_zero);
  Amount current_pd_load(_zero);
  unsigned current_nb_pickups = 0;
  unsigned current_nb_deliveries = 0;

  Vehicle vehicle = input.vehicles[vehicle_rank];

  if (vehicle.depot.has_value()) {
    Index depot_index = vehicle.depot.value().index();

    if (has_start) {
      Index from_index = vehicle.start.value().index();
      Index to_index = input.jobs[route[0]].location.index();
      _preceding_tour_travel_time[1] = vehicle.duration(from_index, to_index);
    }

    for (std::size_t i = 2; i <= route.size(); i++) {
      if (depot_index != input.jobs[route[i - 1]].location.index()) {
        Index from_index = input.jobs[route[i - 2]].location.index();
        Index to_index = input.jobs[route[i - 1]].location.index();

        _preceding_tour_travel_time[i] = _preceding_tour_travel_time[i - 1] + vehicle.duration(from_index, to_index);
      } // else: Reset preceding travel time to zero if we insert after a depot stop
    }

    if (has_end) {
      Index from_index = input.jobs[route[route.size() - 1]].location.index();
      Index to_index = vehicle.end.value().index();

      _following_tour_travel_time[route.size() - 1] = vehicle.duration(from_index, to_index);
    }

    for (std::size_t i = route.size() - 2; i >= 0; i--) {
      if (depot_index != input.jobs[route[i]].location.index()) {
        Index from_index = input.jobs[route[i]].location.index();
        Index to_index = input.jobs[route[i + 1]].location.index();

        _following_tour_travel_time[i] = _following_tour_travel_time[i + 1] + vehicle.duration(from_index, to_index);
      } // else: Reset following travel time to zero if we insert before a depot stop
    
      if (i == 0) break; // Avoid overflow
    }
  }

  for (std::size_t i = 0; i < route.size(); ++i) {
    const auto& job = input.jobs[route[i]];
    switch (job.type) {
    case JOB_TYPE::SINGLE:
      current_pickups += job.pickup;
      current_deliveries += job.delivery;
      break;
    case JOB_TYPE::PICKUP:
      current_pd_load += job.pickup;
      current_nb_pickups += 1;
      break;
    case JOB_TYPE::DELIVERY:
      assert(job.delivery <= current_pd_load);
      current_pd_load -= job.delivery;
      current_nb_deliveries += 1;
      break;
    }
    _fwd_pickups[i] = current_pickups;
    _fwd_deliveries[i] = current_deliveries;
    _pd_loads[i] = current_pd_load;
    assert(current_nb_deliveries <= current_nb_pickups);
    _nb_pickups[i] = current_nb_pickups;
    _nb_deliveries[i] = current_nb_deliveries;
  }
  assert(_pd_loads.back() == _zero);

  current_deliveries = _zero;
  current_pickups = _zero;

  _current_loads.back() = _fwd_pickups.back();
  assert(_current_loads.back() <= capacity);

  for (std::size_t i = 0; i < route.size(); ++i) {
    auto bwd_i = route.size() - i - 1;

    _bwd_deliveries[bwd_i] = current_deliveries;
    _bwd_pickups[bwd_i] = current_pickups;
    _current_loads[bwd_i + 1] =
      _fwd_pickups[bwd_i] + _pd_loads[bwd_i] + current_deliveries;
    assert(_current_loads[bwd_i + 1] <= capacity);
    const auto& job = input.jobs[route[bwd_i]];
    if (job.type == JOB_TYPE::SINGLE) {
      current_deliveries += job.delivery;
      current_pickups += job.pickup;
    }
  }
  _current_loads[0] = current_deliveries;
  assert(_current_loads[0] <= capacity);

  auto peak = _current_loads[0];
  _fwd_peaks[0] = peak;
  for (std::size_t s = 1; s < _fwd_peaks.size(); ++s) {
    // Handle max component-wise.
    for (std::size_t r = 0; r < _zero.size(); ++r) {
      peak[r] = std::max(peak[r], _current_loads[s][r]);
    }
    _fwd_peaks[s] = peak;
  }

  peak = _current_loads.back();
  _bwd_peaks.back() = peak;
  for (std::size_t s = 1; s < _bwd_peaks.size(); ++s) {
    auto bwd_s = _bwd_peaks.size() - s - 1;
    // Handle max component-wise.
    for (std::size_t r = 0; r < _zero.size(); ++r) {
      peak[r] = std::max(peak[r], _current_loads[bwd_s][r]);
    }
    _bwd_peaks[bwd_s] = peak;
  }

  if (route.empty()) {
    _delivery_margin = capacity;
    _pickup_margin = capacity;
  } else {
    assert(!_fwd_pickups.empty());
    const auto& pickups_sum = _fwd_pickups.back();

    for (unsigned i = 0; i < _zero.size(); ++i) {
      _delivery_margin[i] = capacity[i] - _current_loads[0][i];
      _pickup_margin[i] = capacity[i] - pickups_sum[i];
    }
  }
}

bool RawRoute::has_pending_delivery_after_rank(const Index rank) const {
  return _nb_deliveries[rank] < _nb_pickups[rank];
}

bool RawRoute::has_delivery_after_rank(const Index rank) const {
  assert(rank < _nb_deliveries.size());
  return _nb_deliveries[rank] < _nb_deliveries.back();
}

bool RawRoute::is_valid_addition_for_tour(const Input& input,
                                      const Index location,
                                      const Index rank) const {
  #ifdef TOUR_DEBUG
    std::cout << "RawRoute::is_valid_addition_for_tour\n";
  #endif

  Vehicle vehicle = input.vehicles[vehicle_rank];

  if (!vehicle.depot.has_value()) {
    #ifdef TOUR_DEBUG
      std::cout << "  Result: No depot\n";
    #endif
    return true;
  }

  #ifdef TOUR_DEBUG
    std::cout << "  Current route: ";
    std::cout << "(" << vehicle.start.value().index() << ") ";
    for (size_t i = 0; i < route.size(); i++) {
      if (i == rank) {
        std::cout << "* ";
      }

      std::cout << input.jobs[route[i]].location.index() << " ";
    }

    if (rank == route.size()) {
      std::cout << "* ";
    }

    std::cout << "(" << vehicle.end.value().index() << ") ; Size " << route.size() << " \n";

    std::cout << "  New location: " << location << "\n";
  #endif

  assert(rank <= route.size());

  Index preceding_index = location;
  if (rank > 0) {
    preceding_index = input.jobs[route[rank - 1]].location.index();

    #ifdef TOUR_DEBUG
      std::cout << "  Preceding index: " << preceding_index << "\n";
    #endif
  } else if (has_start) {
    preceding_index = vehicle.start.value().index();

    #ifdef TOUR_DEBUG
      std::cout << "  Preceding index: " << preceding_index << " (start)\n";
    #endif
  }

  Index following_index = location;
  if (rank < route.size()) {
    following_index = input.jobs[route[rank]].location.index();

    #ifdef TOUR_DEBUG
      std::cout << "  Following index: " << following_index << "\n";
    #endif
  } else if (has_end) {
    following_index = vehicle.end.value().index();

    #ifdef TOUR_DEBUG
      std::cout << "  Following index: " << following_index << " (end)\n";
    #endif
  }

  Duration additional_to = vehicle.duration(preceding_index, location);
  Duration additional_from = vehicle.duration(location, following_index);
  Duration additional_total = additional_to + additional_from;

  if (route.size() == 0) {
    #ifdef TOUR_DEBUG
      std::cout << "  Trace: " << additional_to << " (to) " << additional_from << " (from)\n";
    #endif

    if (additional_total > vehicle.max_travel_time_per_tour) {
      #ifdef TOUR_DEBUG
        std::cout << "  Result: Exceeds!\n";
      #endif

      return false;
    } else {
      #ifdef TOUR_DEBUG
        std::cout << "  Result: OK!\n";
      #endif

      return true;
    }
  } else {
    Duration preceding = _preceding_tour_travel_time[rank];
    Duration following = _following_tour_travel_time[rank];
    
    #ifdef TOUR_DEBUG
      std::cout << "  Trace: ";
      std::cout << preceding << " (pre) ";
      std::cout << additional_to << " (to) ";
    #endif

    Duration trace = preceding + additional_to;

    if (trace > vehicle.max_travel_time_per_tour) {
      #ifdef TOUR_DEBUG
        std::cout << "\n  Result: Exceeds!\n";
      #endif

      return false;
    }

    if (vehicle.depot.value().index() == location) {
      trace = 0;

      #ifdef TOUR_DEBUG
        std::cout << "/// ";
      #endif
    }

    #ifdef TOUR_DEBUG
      std::cout << additional_from << " (from) ";
      std::cout << following << " (follow) ";
    #endif

    trace += additional_from + following;

    if (trace > vehicle.max_travel_time_per_tour) {
      #ifdef TOUR_DEBUG
        std::cout << "\n  Result: Exceeds!\n";
      #endif

      return false;
    }

    return true;
  }
}

template <class InputIterator>
bool RawRoute::is_valid_addition_for_tour_inclusion(
  const Input& input,
  const InputIterator first_job,
  const InputIterator last_job,
  const Index first_rank,
  const Index last_rank) const {
  #ifdef TOUR_DEBUG
    std::cout << "RawRoute::is_valid_addition_for_tour_inclusion\n";
  #endif

  Vehicle vehicle = input.vehicles[vehicle_rank];

  if (!vehicle.depot.has_value()) {
    #ifdef TOUR_DEBUG
      std::cout << "  Result: No depot\n";
    #endif

    return true;
  }

  if (first_job == last_job) {
    #ifdef TOUR_DEBUG
      std::cout << "  Result: Invalid iterator\n";
    #endif

    return false; // Invalid
  }

  Index depot_index = vehicle.depot.value().index();
  const Duration& max_travel_time_per_tour = vehicle.max_travel_time_per_tour;

  assert(first_rank <= last_rank);
  assert(last_rank <= route.size() + 1);

  #ifdef TOUR_DEBUG
    std::cout << "  Current route: ";
    std::cout << "(" << vehicle.start.value().index() << ") ";
    for (size_t i = 0; i < route.size(); i++) {
      if (i == first_rank) {
        std::cout << "* ";
      }

      if (i == last_rank) {
        std::cout << "* ";
      }

      std::cout << input.jobs[route[i]].location.index() << " ";
    }

    if (route.size() == first_rank) {
      std::cout << "* ";
    }

    if (route.size() == last_rank) {
      std::cout << "* ";
    }

    std::cout << "(" << vehicle.end.value().index() << ") ; Size " << route.size() << " \n";

    std::cout << "  Insertion ranks: " << first_rank << " -> " << last_rank << "\n";
    std::cout << "  Insertion locations: ";
    for (auto job_iter = first_job; job_iter != last_job; ++job_iter) {
      std::cout << input.jobs[*job_iter].location.index() << " ";
    }
    std::cout << "\n";
  #endif

  Index preceding_index = input.jobs[*first_job].location.index();
  if (first_rank > 0) {
    preceding_index = input.jobs[route[first_rank - 1]].location.index();

    #ifdef TOUR_DEBUG
      std::cout << "  Preceding index: " << preceding_index << "\n";
    #endif
  } else if (has_start) {
    preceding_index = vehicle.start.value().index();

    #ifdef TOUR_DEBUG
      std::cout << "  Preceding index: " << preceding_index << " (start)\n";
    #endif
  }

  Index following_index = input.jobs[*(last_job - 1)].location.index();
  if (last_rank < route.size()) {
    following_index = input.jobs[route[last_rank]].location.index();

    #ifdef TOUR_DEBUG
      std::cout << "  Following index: " << following_index << "\n";
    #endif
  } else if (has_end) {
    following_index = vehicle.end.value().index();

    #ifdef TOUR_DEBUG
      std::cout << "  Following index: " << following_index << " (end)\n";
    #endif
  }

  Duration trace = 0;
  Duration addition = 0;

  #ifdef TOUR_DEBUG
    std::cout << "  Trace: ";
  #endif

  if (route.size() > 0) {
    addition = _preceding_tour_travel_time[first_rank];
    trace += addition;
    
    #ifdef TOUR_DEBUG
      std::cout << addition << " (pre) ";
    #endif
  }

  addition = vehicle.duration(preceding_index, input.jobs[*first_job].location.index());
  trace += addition;

  #ifdef TOUR_DEBUG
    std::cout << addition << " (to) ";
  #endif

  if (trace > max_travel_time_per_tour) {
    #ifdef TOUR_DEBUG
      std::cout << "\n  Result: Exceeds!\n";
    #endif

    return false;
  }

  if (first_job != last_job) {
    for (auto job_iter = first_job + 1; job_iter != last_job; ++job_iter) {
      addition = vehicle.duration(
        input.jobs[*(job_iter - 1)].location.index(),
        input.jobs[*(job_iter)].location.index()
      );
      trace += addition;

      #ifdef TOUR_DEBUG
        std::cout << addition << " (move) ";
      #endif

      if (trace > max_travel_time_per_tour) {
        #ifdef TOUR_DEBUG
          std::cout << "\n  Result: Exceeds!\n";
        #endif

        return false;
      }

      if (depot_index == input.jobs[*(job_iter)].location.index()) {
        trace = 0; // Reset because we have returned to depot!

        #ifdef TOUR_DEBUG
          std::cout << "/// ";
        #endif
      }
    }
  }

  addition = vehicle.duration(input.jobs[*(last_job - 1)].location.index(), following_index);
  trace += addition;

  #ifdef TOUR_DEBUG
    std::cout << addition << " (from) ";
  #endif

  if (route.size() > 0) {
    addition = _following_tour_travel_time[last_rank];
    trace += addition;

    #ifdef TOUR_DEBUG
      std::cout << addition << " (follow) ";
    #endif
  }

  if (trace > max_travel_time_per_tour) {
    #ifdef TOUR_DEBUG
      std::cout << "\n  Result: Exceeds!\n";
    #endif

    return false;
  }

  #ifdef TOUR_DEBUG
    std::cout << "\n  Result: OK!\n";
  #endif

  return true;
}

bool RawRoute::has_pickup_up_to_rank(const Index rank) const {
  assert(rank < _nb_pickups.size());
  return 0 < _nb_pickups[rank];
}

bool RawRoute::is_valid_addition_for_capacity(const Input&,
                                              const Amount& pickup,
                                              const Amount& delivery,
                                              const Index rank) const {
  assert(rank <= route.size());

  return (_fwd_peaks[rank] + delivery <= capacity) and
         (_bwd_peaks[rank] + pickup <= capacity);
}

bool RawRoute::is_valid_addition_for_load(const Input&,
                                          const Amount& pickup,
                                          const Index rank) const {
  assert(rank <= route.size());

  const auto& load = route.empty() ? _zero : _current_loads[rank];
  return load + pickup <= capacity;
}

bool RawRoute::is_valid_addition_for_capacity_margins(
  const Input&,
  const Amount& pickup,
  const Amount& delivery,
  const Index first_rank,
  const Index last_rank) const {
  assert(1 <= last_rank);
  assert(last_rank <= route.size() + 1);

  const auto& first_deliveries =
    (first_rank == 0) ? _current_loads[0] : _bwd_deliveries[first_rank - 1];

  const auto& first_pickups =
    (first_rank == 0) ? _zero : _fwd_pickups[first_rank - 1];

  auto replaced_deliveries = first_deliveries - _bwd_deliveries[last_rank - 1];

  return (_fwd_peaks[first_rank] + delivery <=
          capacity + replaced_deliveries) and
         (_bwd_peaks[last_rank] + pickup <=
          capacity + _fwd_pickups[last_rank - 1] - first_pickups);
}

template <class InputIterator>
bool RawRoute::is_valid_addition_for_capacity_inclusion(
  const Input& input,
  Amount delivery,
  const InputIterator first_job,
  const InputIterator last_job,
  const Index first_rank,
  const Index last_rank) const {
  assert(first_rank <= last_rank);
  assert(last_rank <= route.size() + 1);

  const auto& init_load = (route.empty()) ? _zero : _current_loads[0];

  const auto& first_deliveries =
    (first_rank == 0) ? init_load : _bwd_deliveries[first_rank - 1];

  const auto& last_deliveries =
    (last_rank == 0) ? init_load : _bwd_deliveries[last_rank - 1];

  auto replaced_deliveries = first_deliveries - last_deliveries;

  delivery += ((route.empty()) ? _zero : _current_loads[first_rank]) -
              replaced_deliveries;

  bool valid = (delivery <= capacity);

  for (auto job_iter = first_job; valid and job_iter != last_job; ++job_iter) {
    auto& job = input.jobs[*job_iter];
    delivery += job.pickup;
    delivery -= job.delivery;

    valid = (delivery <= capacity);
  }

  return valid;
}

const Amount& RawRoute::job_deliveries_sum() const {
  return route.empty() ? _zero : _current_loads[0];
}

const Amount& RawRoute::job_pickups_sum() const {
  return route.empty() ? _zero : _fwd_pickups.back();
}

const Amount& RawRoute::delivery_margin() const {
  return _delivery_margin;
}

const Amount& RawRoute::pickup_margin() const {
  return _pickup_margin;
}

Amount RawRoute::pickup_in_range(Index i, Index j) const {
  assert(i <= j and j <= _fwd_pickups.size());
  if (i == j or route.empty()) {
    return _zero;
  }
  if (i == 0) {
    return _fwd_pickups[j - 1];
  }
  return _fwd_pickups[j - 1] - _fwd_pickups[i - 1];
}

Amount RawRoute::delivery_in_range(Index i, Index j) const {
  assert(i <= j and j <= _bwd_deliveries.size());
  if (i == j or route.empty()) {
    return _zero;
  }
  const auto& before_deliveries =
    (i == 0) ? _current_loads[0] : _bwd_deliveries[i - 1];
  return before_deliveries - _bwd_deliveries[j - 1];
}

void RawRoute::add(const Input& input, const Index job_rank, const Index rank) {
  route.insert(route.begin() + rank, job_rank);
  update_amounts(input);
}

void RawRoute::remove(const Input& input,
                      const Index rank,
                      const unsigned count) {
  route.erase(route.begin() + rank, route.begin() + rank + count);
  update_amounts(input);
}

template <class InputIterator>
void RawRoute::replace(const Input& input,
                       const InputIterator first_job,
                       const InputIterator last_job,
                       const Index first_rank,
                       const Index last_rank) {
  assert(first_rank <= last_rank);

  route.erase(route.begin() + first_rank, route.begin() + last_rank);
  route.insert(route.begin() + first_rank, first_job, last_job);

  update_amounts(input);
}

template bool RawRoute::is_valid_addition_for_capacity_inclusion(
  const Input& input,
  Amount delivery,
  const std::vector<Index>::iterator first_job,
  const std::vector<Index>::iterator last_job,
  const Index first_rank,
  const Index last_rank) const;
template bool RawRoute::is_valid_addition_for_capacity_inclusion(
  const Input& input,
  Amount delivery,
  const std::vector<Index>::const_iterator first_job,
  const std::vector<Index>::const_iterator last_job,
  const Index first_rank,
  const Index last_rank) const;
template bool RawRoute::is_valid_addition_for_capacity_inclusion(
  const Input& input,
  Amount delivery,
  const std::vector<Index>::reverse_iterator first_job,
  const std::vector<Index>::reverse_iterator last_job,
  const Index first_rank,
  const Index last_rank) const;

template bool RawRoute::is_valid_addition_for_tour_inclusion(
  const Input& input,
  const std::vector<Index>::iterator first_job,
  const std::vector<Index>::iterator last_job,
  const Index first_rank,
  const Index last_rank) const;
template bool RawRoute::is_valid_addition_for_tour_inclusion(
  const Input& input,
  const std::vector<Index>::const_iterator first_job,
  const std::vector<Index>::const_iterator last_job,
  const Index first_rank,
  const Index last_rank) const;
template bool RawRoute::is_valid_addition_for_tour_inclusion(
  const Input& input,
  const std::vector<Index>::reverse_iterator first_job,
  const std::vector<Index>::reverse_iterator last_job,
  const Index first_rank,
  const Index last_rank) const;

template void RawRoute::replace(const Input& input,
                                std::vector<Index>::iterator first_job,
                                std::vector<Index>::iterator last_job,
                                const Index first_rank,
                                const Index last_rank);

} // namespace vroom
