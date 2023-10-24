/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <algorithm>

#include "problems/vrptw/operators/priority_replace.h"

namespace vroom::vrptw {

PriorityReplace::PriorityReplace(const Input& input,
                                 const utils::SolutionState& sol_state,
                                 std::unordered_set<Index>& unassigned,
                                 TWRoute& tw_s_route,
                                 Index s_vehicle,
                                 Index s_rank,
                                 Index u)
  : cvrp::PriorityReplace(input,
                          sol_state,
                          unassigned,
                          static_cast<RawRoute&>(tw_s_route),
                          s_vehicle,
                          s_rank,
                          u),
    _tw_s_route(tw_s_route) {
}

bool PriorityReplace::is_valid() {
  bool valid = cvrp::PriorityReplace::is_valid();

  if (valid) {
    std::vector<Index> job_ranks({_u});
    valid = _tw_s_route.is_valid_addition_for_tw(_input,
                                                 _input.jobs[_u].delivery,
                                                 job_ranks.begin(),
                                                 job_ranks.end(),
                                                 0,
                                                 s_rank + 1);
  }

  return valid;
}

void PriorityReplace::apply() {
  assert(_unassigned.contains(_u));
  _unassigned.erase(_u);

  assert(
    std::all_of(s_route.cbegin(),
                s_route.cbegin() + s_rank + 1,
                [this](const auto j) { return !_unassigned.contains(j); }));
  _unassigned.insert(s_route.cbegin(), s_route.cbegin() + s_rank + 1);

  const std::vector<Index> addition({_u});
  _tw_s_route.replace(_input,
                      _input.jobs[_u].delivery,
                      addition.begin(),
                      addition.end(),
                      0,
                      s_rank + 1);
}

} // namespace vroom::vrptw
