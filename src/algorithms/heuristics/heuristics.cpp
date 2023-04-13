/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <set>

#include "algorithms/heuristics/heuristics.h"
#include "utils/helpers.h"

namespace vroom::heuristics {

std::vector<std::vector<Eval>> get_jobs_vehicles_evals(const Input& input) {
  // For a single job j, evals[j][v] evaluates fetching job j in an
  // empty route from vehicle at rank v. For a pickup job j,
  // evals[j][v] evaluates fetching job j **and** associated delivery
  // in an empty route from vehicle at rank v.
  std::vector<std::vector<Eval>> evals(input.jobs.size(),
                                       std::vector<Eval>(
                                         input.vehicles.size()));
  for (std::size_t j = 0; j < input.jobs.size(); ++j) {
    Index j_index = input.jobs[j].index();
    bool is_pickup = (input.jobs[j].type == JOB_TYPE::PICKUP);

    Index last_job_index = j_index;
    if (is_pickup) {
      assert((j + 1 < input.jobs.size()) and
             (input.jobs[j + 1].type == JOB_TYPE::DELIVERY));
      last_job_index = input.jobs[j + 1].index();
    }

    for (std::size_t v = 0; v < input.vehicles.size(); ++v) {
      const auto& vehicle = input.vehicles[v];
      Eval current_eval =
        is_pickup ? vehicle.eval(j_index, last_job_index) : Eval();
      if (vehicle.has_start()) {
        current_eval += vehicle.eval(vehicle.start.value().index(), j_index);
      }
      if (vehicle.has_end()) {
        current_eval +=
          vehicle.eval(last_job_index, vehicle.end.value().index());
      }
      evals[j][v] = current_eval;
      if (is_pickup) {
        // Assign same eval to delivery.
        evals[j + 1][v] = current_eval;
      }
    }

    if (is_pickup) {
      // Skip delivery.
      ++j;
    }
  }

  return evals;
}

template <class T>
T basic(const Input& input, INIT init, double lambda, SORT sort) {
  const auto nb_vehicles = input.vehicles.size();
  T routes;
  for (Index v = 0; v < nb_vehicles; ++v) {
    routes.emplace_back(input, v, input.zero_amount().size());
  }

  std::set<Index> unassigned;
  for (Index j = 0; j < input.jobs.size(); ++j) {
    unassigned.insert(j);
  }

  // One level of indirection to allow easy ordering of the vehicles
  // within the heuristic.
  std::vector<Index> vehicles_ranks(nb_vehicles);
  std::iota(vehicles_ranks.begin(), vehicles_ranks.end(), 0);

  switch (sort) {
  case SORT::CAPACITY:
    // Sort vehicles by decreasing max number of tasks allowed, then
    // capacity (not a total order), then working hours length.
    std::stable_sort(vehicles_ranks.begin(),
                     vehicles_ranks.end(),
                     [&](const auto lhs, const auto rhs) {
                       auto& v_lhs = input.vehicles[lhs];
                       auto& v_rhs = input.vehicles[rhs];
                       return v_lhs.max_tasks > v_rhs.max_tasks or
                              (v_lhs.max_tasks == v_rhs.max_tasks and
                               (v_rhs.capacity << v_lhs.capacity or
                                (v_lhs.capacity == v_rhs.capacity and
                                 v_lhs.tw.length > v_rhs.tw.length)));
                     });
    break;
  case SORT::COST:
    // Sort vehicles by increasing fixed cost, then same as above.
    std::stable_sort(vehicles_ranks.begin(),
                     vehicles_ranks.end(),
                     [&](const auto lhs, const auto rhs) {
                       auto& v_lhs = input.vehicles[lhs];
                       auto& v_rhs = input.vehicles[rhs];
                       return v_lhs.costs < v_rhs.costs or
                              (v_lhs.costs == v_rhs.costs and
                               (v_lhs.max_tasks > v_rhs.max_tasks or
                                (v_lhs.max_tasks == v_rhs.max_tasks and
                                 (v_rhs.capacity << v_lhs.capacity or
                                  (v_lhs.capacity == v_rhs.capacity and
                                   v_lhs.tw.length > v_rhs.tw.length)))));
                     });
    break;
  }

  auto evals = get_jobs_vehicles_evals(input);

  // regrets[v][j] holds the min cost for reaching job j in an empty
  // route across all remaining vehicles **after** vehicle at rank v
  // in vehicle_ranks.
  std::vector<std::vector<Cost>> regrets(nb_vehicles,
                                         std::vector<Cost>(input.jobs.size()));

  // Use own cost for last vehicle regret values.
  auto& last_regrets = regrets.back();
  for (Index j = 0; j < input.jobs.size(); ++j) {
    last_regrets[j] = evals[j][vehicles_ranks.back()].cost;
  }

  for (Index rev_v = 0; rev_v < nb_vehicles - 1; ++rev_v) {
    // Going trough vehicles backward from second to last.
    const auto v = nb_vehicles - 2 - rev_v;
    for (Index j = 0; j < input.jobs.size(); ++j) {
      regrets[v][j] =
        std::min(regrets[v + 1][j], (evals[j][vehicles_ranks[v + 1]]).cost);
    }
  }

  for (Index v = 0; v < nb_vehicles; ++v) {
    auto v_rank = vehicles_ranks[v];
    auto& current_r = routes[v_rank];

    const auto& vehicle = input.vehicles[v_rank];

    Duration current_route_duration = 0;

    if (init != INIT::NONE) {
      // Initialize current route with the "best" valid job.
      bool init_ok = false;

      Amount higher_amount(input.zero_amount());
      Cost furthest_cost = 0;
      Cost nearest_cost = std::numeric_limits<Cost>::max();
      Duration earliest_deadline = std::numeric_limits<Duration>::max();
      Index best_job_rank = 0;
      for (const auto job_rank : unassigned) {
        if (!input.vehicle_ok_with_job(v_rank, job_rank) or
            input.jobs[job_rank].type == JOB_TYPE::DELIVERY) {
          continue;
        }

        bool is_pickup = (input.jobs[job_rank].type == JOB_TYPE::PICKUP);

        if (current_r.size() + (is_pickup ? 2 : 1) > vehicle.max_tasks) {
          continue;
        }

        bool try_validity = false;

        if (init == INIT::HIGHER_AMOUNT) {
          try_validity |= (higher_amount << input.jobs[job_rank].pickup or
                           higher_amount << input.jobs[job_rank].delivery);
        }
        if (init == INIT::EARLIEST_DEADLINE) {
          Duration current_deadline =
            (is_pickup) ? input.jobs[job_rank + 1].tws.back().end
                        : input.jobs[job_rank].tws.back().end;
          try_validity |= (current_deadline < earliest_deadline);
        }
        if (init == INIT::FURTHEST) {
          try_validity |= (furthest_cost < evals[job_rank][v_rank].cost);
        }
        if (init == INIT::NEAREST) {
          try_validity |= (evals[job_rank][v_rank].cost < nearest_cost);
        }

        if (!try_validity) {
          continue;
        }

        bool is_valid =
          (vehicle.ok_for_travel_time(evals[job_rank][v_rank].duration)) &&
          current_r
            .is_valid_addition_for_capacity(input,
                                            input.jobs[job_rank].pickup,
                                            input.jobs[job_rank].delivery,
                                            0) &&
          current_r.is_valid_addition_for_tour(input,
                                               input.jobs[job_rank].location.index(),
                                               0);
        if (is_pickup) {
          std::vector<Index> p_d({job_rank, static_cast<Index>(job_rank + 1)});
          is_valid =
            is_valid && current_r.is_valid_addition_for_tw(input,
                                                           input.zero_amount(),
                                                           p_d.begin(),
                                                           p_d.end(),
                                                           0,
                                                           0);
        } else {
          assert(input.jobs[job_rank].type == JOB_TYPE::SINGLE);
          is_valid =
            is_valid && current_r.is_valid_addition_for_tw(input, job_rank, 0);
        }

        if (is_valid) {
          init_ok = true;
          best_job_rank = job_rank;

          switch (init) {
          case INIT::NONE:
            assert(false);
            break;
          case INIT::HIGHER_AMOUNT:
            if (higher_amount << input.jobs[job_rank].pickup) {
              higher_amount = input.jobs[job_rank].pickup;
            }
            if (higher_amount << input.jobs[job_rank].delivery) {
              higher_amount = input.jobs[job_rank].delivery;
            }
            break;
          case INIT::EARLIEST_DEADLINE:
            earliest_deadline = (is_pickup)
                                  ? input.jobs[job_rank + 1].tws.back().end
                                  : input.jobs[job_rank].tws.back().end;
            break;
          case INIT::FURTHEST:
            furthest_cost = evals[job_rank][v_rank].cost;
            break;
          case INIT::NEAREST:
            nearest_cost = evals[job_rank][v_rank].cost;
            break;
          }
        }
      }

      if (init_ok) {
        if (input.jobs[best_job_rank].type == JOB_TYPE::SINGLE) {
          current_r.add(input, best_job_rank, 0);
          unassigned.erase(best_job_rank);
        }
        if (input.jobs[best_job_rank].type == JOB_TYPE::PICKUP) {
          std::vector<Index> p_d(
            {best_job_rank, static_cast<Index>(best_job_rank + 1)});
          current_r
            .replace(input, input.zero_amount(), p_d.begin(), p_d.end(), 0, 0);
          unassigned.erase(best_job_rank);
          unassigned.erase(best_job_rank + 1);
        }
        current_route_duration += evals[best_job_rank][v_rank].duration;
      }
    }

    bool keep_going = true;
    while (keep_going) {
      keep_going = false;
      double best_cost = std::numeric_limits<double>::max();
      Index best_job_rank = 0;
      Index best_r = 0;
      Index best_pickup_r = 0;
      Index best_delivery_r = 0;
      Amount best_modified_delivery = input.zero_amount();
      Duration best_duration_addition = 0;

      for (const auto job_rank : unassigned) {
        if (!input.vehicle_ok_with_job(v_rank, job_rank)) {
          continue;
        }

        if (input.jobs[job_rank].type == JOB_TYPE::DELIVERY) {
          continue;
        }

        if (input.jobs[job_rank].type == JOB_TYPE::SINGLE and
            current_r.size() + 1 <= vehicle.max_tasks) {
          for (Index r = 0; r <= current_r.size(); ++r) {
            const auto current_add = utils::addition_cost(input,
                                                          job_rank,
                                                          vehicle,
                                                          current_r.route,
                                                          r);

            double current_cost =
              static_cast<double>(current_add.cost) -
              lambda * static_cast<double>(regrets[v][job_rank]);

            if (current_cost < best_cost and
                (vehicle.ok_for_travel_time(current_route_duration +
                                            current_add.duration)) and
                current_r
                  .is_valid_addition_for_capacity(input,
                                                  input.jobs[job_rank].pickup,
                                                  input.jobs[job_rank].delivery,
                                                  r) and
                current_r.is_valid_addition_for_tw(input, job_rank, r) and
                current_r.is_valid_addition_for_tour(input,
                                                    input.jobs[job_rank].location.index(),
                                                    r)) {
              best_cost = current_cost;
              best_job_rank = job_rank;
              best_r = r;
              best_duration_addition = current_add.duration;
            }
          }
        }

        if (input.jobs[job_rank].type == JOB_TYPE::PICKUP and
            current_r.size() + 2 <= vehicle.max_tasks) {
          // Pre-compute cost of addition for matching delivery.
          std::vector<Eval> d_adds(current_r.route.size() + 1);
          std::vector<unsigned char> valid_delivery_insertions(
            current_r.route.size() + 1);

          for (unsigned d_rank = 0; d_rank <= current_r.route.size();
               ++d_rank) {
            d_adds[d_rank] = utils::addition_cost(input,
                                                  job_rank + 1,
                                                  vehicle,
                                                  current_r.route,
                                                  d_rank);
            valid_delivery_insertions[d_rank] =
              current_r.is_valid_addition_for_tw_without_max_load(input,
                                                                  job_rank + 1,
                                                                  d_rank);
          }

          for (Index pickup_r = 0; pickup_r <= current_r.size(); ++pickup_r) {
            const auto p_add = utils::addition_cost(input,
                                                    job_rank,
                                                    vehicle,
                                                    current_r.route,
                                                    pickup_r);

            if (!current_r
                   .is_valid_addition_for_load(input,
                                               input.jobs[job_rank].pickup,
                                               pickup_r) or
                !current_r
                   .is_valid_addition_for_tw_without_max_load(input,
                                                              job_rank,
                                                              pickup_r)) {
              continue;
            }

            // Build replacement sequence for current insertion.
            std::vector<Index> modified_with_pd({job_rank});
            Amount modified_delivery = input.zero_amount();

            for (Index delivery_r = pickup_r; delivery_r <= current_r.size();
                 ++delivery_r) {
              // Update state variables along the way before potential
              // early abort.
              if (pickup_r < delivery_r) {
                modified_with_pd.push_back(current_r.route[delivery_r - 1]);
                const auto& new_modified_job =
                  input.jobs[current_r.route[delivery_r - 1]];
                if (new_modified_job.type == JOB_TYPE::SINGLE) {
                  modified_delivery += new_modified_job.delivery;
                }
              }

              if (!static_cast<bool>(valid_delivery_insertions[delivery_r])) {
                continue;
              }

              Eval current_add;
              if (pickup_r == delivery_r) {
                current_add = utils::addition_cost(input,
                                                   job_rank,
                                                   vehicle,
                                                   current_r.route,
                                                   pickup_r,
                                                   pickup_r + 1);
              } else {
                current_add = p_add + d_adds[delivery_r];
              }

              double current_cost =
                current_add.cost -
                lambda * static_cast<double>(regrets[v][job_rank]);

              if (current_cost < best_cost) {
                modified_with_pd.push_back(job_rank + 1);

                // Update best cost depending on validity.
                bool valid =
                  (vehicle.ok_for_travel_time(current_route_duration +
                                              current_add.duration)) &&
                  current_r
                    .is_valid_addition_for_capacity_inclusion(input,
                                                              modified_delivery,
                                                              modified_with_pd
                                                                .begin(),
                                                              modified_with_pd
                                                                .end(),
                                                              pickup_r,
                                                              delivery_r) &&
                  current_r
                    .is_valid_addition_for_tour_inclusion(input,
                                                modified_with_pd
                                                  .begin(),
                                                modified_with_pd
                                                  .end(),
                                                pickup_r,
                                                delivery_r);

                valid =
                  valid &&
                  current_r.is_valid_addition_for_tw(input,
                                                     modified_delivery,
                                                     modified_with_pd.begin(),
                                                     modified_with_pd.end(),
                                                     pickup_r,
                                                     delivery_r);

                modified_with_pd.pop_back();

                if (valid) {
                  best_cost = current_cost;
                  best_job_rank = job_rank;
                  best_pickup_r = pickup_r;
                  best_delivery_r = delivery_r;
                  best_modified_delivery = modified_delivery;
                  best_duration_addition = current_add.duration;
                }
              }
            }
          }
        }
      }

      if (best_cost < std::numeric_limits<double>::max()) {
        if (input.jobs[best_job_rank].type == JOB_TYPE::SINGLE) {
          current_r.add(input, best_job_rank, best_r);
          unassigned.erase(best_job_rank);
          keep_going = true;
        }
        if (input.jobs[best_job_rank].type == JOB_TYPE::PICKUP) {
          std::vector<Index> modified_with_pd({best_job_rank});
          std::copy(current_r.route.begin() + best_pickup_r,
                    current_r.route.begin() + best_delivery_r,
                    std::back_inserter(modified_with_pd));
          modified_with_pd.push_back(best_job_rank + 1);

          current_r.replace(input,
                            best_modified_delivery,
                            modified_with_pd.begin(),
                            modified_with_pd.end(),
                            best_pickup_r,
                            best_delivery_r);
          unassigned.erase(best_job_rank);
          unassigned.erase(best_job_rank + 1);
          keep_going = true;
        }

        current_route_duration += best_duration_addition;
      }
    }
  }

  return routes;
}

template <class T>
T dynamic_vehicle_choice(const Input& input,
                         INIT init,
                         double lambda,
                         SORT sort) {
  const auto nb_vehicles = input.vehicles.size();
  T routes;
  for (Index v = 0; v < nb_vehicles; ++v) {
    routes.emplace_back(input, v, input.zero_amount().size());
  }

  std::set<Index> unassigned;
  for (Index j = 0; j < input.jobs.size(); ++j) {
    unassigned.insert(j);
  }

  std::vector<Index> vehicles_ranks(nb_vehicles);
  std::iota(vehicles_ranks.begin(), vehicles_ranks.end(), 0);

  auto evals = get_jobs_vehicles_evals(input);

  while (!vehicles_ranks.empty() and !unassigned.empty()) {
    // For any unassigned job at j, jobs_min_costs[j]
    // (resp. jobs_second_min_costs[j]) holds the min cost
    // (resp. second min cost) of picking the job in an empty route
    // for any remaining vehicle.
    std::vector<Cost> jobs_min_costs(input.jobs.size(),
                                     std::numeric_limits<Cost>::max());
    std::vector<Cost> jobs_second_min_costs(input.jobs.size(),
                                            std::numeric_limits<Cost>::max());
    for (const auto job_rank : unassigned) {
      for (const auto v_rank : vehicles_ranks) {
        if (evals[job_rank][v_rank].cost <= jobs_min_costs[job_rank]) {
          jobs_second_min_costs[job_rank] = jobs_min_costs[job_rank];
          jobs_min_costs[job_rank] = evals[job_rank][v_rank].cost;
        } else {
          if (evals[job_rank][v_rank].cost < jobs_second_min_costs[job_rank]) {
            jobs_second_min_costs[job_rank] = evals[job_rank][v_rank].cost;
          }
        }
      }
    }

    // Pick vehicle that has the biggest number of compatible jobs
    // closest to him than to any other different vehicle.
    std::vector<unsigned> closest_jobs_count(nb_vehicles, 0);
    for (const auto job_rank : unassigned) {
      for (const auto v_rank : vehicles_ranks) {
        if (evals[job_rank][v_rank].cost == jobs_min_costs[job_rank]) {
          ++closest_jobs_count[v_rank];
        }
      }
    }

    Index v_rank;

    if (sort == SORT::CAPACITY) {
      const auto chosen_vehicle =
        std::min_element(vehicles_ranks.begin(),
                         vehicles_ranks.end(),
                         [&](const auto lhs, const auto rhs) {
                           auto& v_lhs = input.vehicles[lhs];
                           auto& v_rhs = input.vehicles[rhs];
                           return closest_jobs_count[lhs] >
                                    closest_jobs_count[rhs] or
                                  (closest_jobs_count[lhs] ==
                                     closest_jobs_count[rhs] and
                                   (v_rhs.capacity << v_lhs.capacity or
                                    (v_lhs.capacity == v_rhs.capacity and
                                     v_lhs.tw.length > v_rhs.tw.length)));
                         });
      v_rank = *chosen_vehicle;
      vehicles_ranks.erase(chosen_vehicle);
    } else {
      assert(sort == SORT::COST);

      const auto chosen_vehicle =
        std::min_element(vehicles_ranks.begin(),
                         vehicles_ranks.end(),
                         [&](const auto lhs, const auto rhs) {
                           auto& v_lhs = input.vehicles[lhs];
                           auto& v_rhs = input.vehicles[rhs];
                           return closest_jobs_count[lhs] >
                                    closest_jobs_count[rhs] or
                                  (closest_jobs_count[lhs] ==
                                     closest_jobs_count[rhs] and
                                   (v_lhs.costs < v_rhs.costs or
                                    (v_lhs.costs == v_rhs.costs and
                                     (v_rhs.capacity << v_lhs.capacity or
                                      (v_lhs.capacity == v_rhs.capacity and
                                       v_lhs.tw.length > v_rhs.tw.length)))));
                         });
      v_rank = *chosen_vehicle;
      vehicles_ranks.erase(chosen_vehicle);
    }

    // Once current vehicle is decided, regrets[j] holds the min cost
    // of picking the job in an empty route for other remaining
    // vehicles.
    std::vector<Cost> regrets(input.jobs.size(), input.get_cost_upper_bound());
    for (const auto job_rank : unassigned) {
      if (jobs_min_costs[job_rank] < evals[job_rank][v_rank].cost) {
        regrets[job_rank] = jobs_min_costs[job_rank];
      } else {
        regrets[job_rank] = jobs_second_min_costs[job_rank];
      }
    }

    const auto& vehicle = input.vehicles[v_rank];
    auto& current_r = routes[v_rank];

    Duration current_route_duration = 0;

    if (init != INIT::NONE) {
      // Initialize current route with the "best" valid job that is
      // closest for current vehicle than to any other remaining
      // vehicle.
      bool init_ok = false;

      Amount higher_amount(input.zero_amount());
      Cost furthest_cost = 0;
      Cost nearest_cost = std::numeric_limits<Cost>::max();
      Duration earliest_deadline = std::numeric_limits<Duration>::max();
      Index best_job_rank = 0;
      for (const auto job_rank : unassigned) {
        if (jobs_min_costs[job_rank] < evals[job_rank][v_rank].cost or
            // One of the remaining vehicles is closest to that job.
            !input.vehicle_ok_with_job(v_rank, job_rank) or
            input.jobs[job_rank].type == JOB_TYPE::DELIVERY) {
          continue;
        }

        bool is_pickup = (input.jobs[job_rank].type == JOB_TYPE::PICKUP);

        if (current_r.size() + (is_pickup ? 2 : 1) > vehicle.max_tasks) {
          continue;
        }

        bool try_validity = false;

        if (init == INIT::HIGHER_AMOUNT) {
          try_validity |= (higher_amount << input.jobs[job_rank].pickup or
                           higher_amount << input.jobs[job_rank].delivery);
        }
        if (init == INIT::EARLIEST_DEADLINE) {
          Duration current_deadline =
            (is_pickup) ? input.jobs[job_rank + 1].tws.back().end
                        : input.jobs[job_rank].tws.back().end;
          try_validity |= (current_deadline < earliest_deadline);
        }
        if (init == INIT::FURTHEST) {
          try_validity |= (furthest_cost < evals[job_rank][v_rank].cost);
        }
        if (init == INIT::NEAREST) {
          try_validity |= (evals[job_rank][v_rank].cost < nearest_cost);
        }

        if (!try_validity) {
          continue;
        }

        bool is_valid =
          (vehicle.ok_for_travel_time(evals[job_rank][v_rank].duration)) &&
          current_r
            .is_valid_addition_for_capacity(input,
                                            input.jobs[job_rank].pickup,
                                            input.jobs[job_rank].delivery,
                                            0)  &&
          current_r.is_valid_addition_for_tour(input,
                                               input.jobs[job_rank].location.index(),
                                               0);

        if (is_pickup) {
          std::vector<Index> p_d({job_rank, static_cast<Index>(job_rank + 1)});
          is_valid =
            is_valid && current_r.is_valid_addition_for_tw(input,
                                                           input.zero_amount(),
                                                           p_d.begin(),
                                                           p_d.end(),
                                                           0,
                                                           0);
        } else {
          assert(input.jobs[job_rank].type == JOB_TYPE::SINGLE);
          is_valid =
            is_valid && current_r.is_valid_addition_for_tw(input, job_rank, 0);
        }

        if (is_valid) {
          init_ok = true;
          best_job_rank = job_rank;

          switch (init) {
          case INIT::NONE:
            assert(false);
            break;
          case INIT::HIGHER_AMOUNT:
            if (higher_amount << input.jobs[job_rank].pickup) {
              higher_amount = input.jobs[job_rank].pickup;
            }
            if (higher_amount << input.jobs[job_rank].delivery) {
              higher_amount = input.jobs[job_rank].delivery;
            }
            break;
          case INIT::EARLIEST_DEADLINE:
            earliest_deadline = (is_pickup)
                                  ? input.jobs[job_rank + 1].tws.back().end
                                  : input.jobs[job_rank].tws.back().end;
            break;
          case INIT::FURTHEST:
            furthest_cost = evals[job_rank][v_rank].cost;
            break;
          case INIT::NEAREST:
            nearest_cost = evals[job_rank][v_rank].cost;
            break;
          }
        }
      }

      if (init_ok) {
        if (input.jobs[best_job_rank].type == JOB_TYPE::SINGLE) {
          current_r.add(input, best_job_rank, 0);
          unassigned.erase(best_job_rank);
        }
        if (input.jobs[best_job_rank].type == JOB_TYPE::PICKUP) {
          std::vector<Index> p_d(
            {best_job_rank, static_cast<Index>(best_job_rank + 1)});
          current_r
            .replace(input, input.zero_amount(), p_d.begin(), p_d.end(), 0, 0);
          unassigned.erase(best_job_rank);
          unassigned.erase(best_job_rank + 1);
        }
        current_route_duration += evals[best_job_rank][v_rank].duration;
      }
    }

    bool keep_going = true;
    while (keep_going) {
      keep_going = false;
      double best_cost = std::numeric_limits<double>::max();
      Index best_job_rank = 0;
      Index best_r = 0;
      Index best_pickup_r = 0;
      Index best_delivery_r = 0;
      Amount best_modified_delivery = input.zero_amount();
      Duration best_duration_addition = 0;

      for (const auto job_rank : unassigned) {
        if (!input.vehicle_ok_with_job(v_rank, job_rank)) {
          continue;
        }

        if (input.jobs[job_rank].type == JOB_TYPE::DELIVERY) {
          continue;
        }

        if (input.jobs[job_rank].type == JOB_TYPE::SINGLE and
            current_r.size() + 1 <= vehicle.max_tasks) {
          for (Index r = 0; r <= current_r.size(); ++r) {
            const auto current_add = utils::addition_cost(input,
                                                          job_rank,
                                                          vehicle,
                                                          current_r.route,
                                                          r);

            double current_cost =
              static_cast<double>(current_add.cost) -
              lambda * static_cast<double>(regrets[job_rank]);

            if (current_cost < best_cost and
                (vehicle.ok_for_travel_time(current_route_duration +
                                            current_add.duration)) and
                current_r
                  .is_valid_addition_for_capacity(input,
                                                  input.jobs[job_rank].pickup,
                                                  input.jobs[job_rank].delivery,
                                                  r) and
                current_r.is_valid_addition_for_tw(input, job_rank, r)  &&
                current_r.is_valid_addition_for_tour(input,
                                               input.jobs[job_rank].location.index(),
                                               r)) {
              best_cost = current_cost;
              best_job_rank = job_rank;
              best_r = r;
              best_duration_addition = current_add.duration;
            }
          }
        }

        if (input.jobs[job_rank].type == JOB_TYPE::PICKUP and
            current_r.size() + 2 <= vehicle.max_tasks) {
          // Pre-compute cost of addition for matching delivery.
          std::vector<Eval> d_adds(current_r.route.size() + 1);
          std::vector<unsigned char> valid_delivery_insertions(
            current_r.route.size() + 1);

          for (unsigned d_rank = 0; d_rank <= current_r.route.size();
               ++d_rank) {
            d_adds[d_rank] = utils::addition_cost(input,
                                                  job_rank + 1,
                                                  vehicle,
                                                  current_r.route,
                                                  d_rank);
            valid_delivery_insertions[d_rank] =
              current_r.is_valid_addition_for_tw_without_max_load(input,
                                                                  job_rank + 1,
                                                                  d_rank);
          }

          for (Index pickup_r = 0; pickup_r <= current_r.size(); ++pickup_r) {
            const auto p_add = utils::addition_cost(input,
                                                    job_rank,
                                                    vehicle,
                                                    current_r.route,
                                                    pickup_r);

            if (!current_r
                   .is_valid_addition_for_load(input,
                                               input.jobs[job_rank].pickup,
                                               pickup_r) or
                !current_r
                   .is_valid_addition_for_tw_without_max_load(input,
                                                              job_rank,
                                                              pickup_r)) {
              continue;
            }

            // Build replacement sequence for current insertion.
            std::vector<Index> modified_with_pd({job_rank});
            Amount modified_delivery = input.zero_amount();

            for (Index delivery_r = pickup_r; delivery_r <= current_r.size();
                 ++delivery_r) {
              // Update state variables along the way before potential
              // early abort.
              if (pickup_r < delivery_r) {
                modified_with_pd.push_back(current_r.route[delivery_r - 1]);
                const auto& new_modified_job =
                  input.jobs[current_r.route[delivery_r - 1]];
                if (new_modified_job.type == JOB_TYPE::SINGLE) {
                  modified_delivery += new_modified_job.delivery;
                }
              }

              if (!static_cast<bool>(valid_delivery_insertions[delivery_r])) {
                continue;
              }

              Eval current_add;
              if (pickup_r == delivery_r) {
                current_add = utils::addition_cost(input,
                                                   job_rank,
                                                   vehicle,
                                                   current_r.route,
                                                   pickup_r,
                                                   pickup_r + 1);
              } else {
                current_add = p_add + d_adds[delivery_r];
              }

              double current_cost =
                current_add.cost -
                lambda * static_cast<double>(regrets[job_rank]);

              if (current_cost < best_cost) {
                modified_with_pd.push_back(job_rank + 1);

                // Update best cost depending on validity.
                bool valid =
                  (vehicle.ok_for_travel_time(current_route_duration +
                                              current_add.duration)) &&
                  current_r
                    .is_valid_addition_for_capacity_inclusion(input,
                                                              modified_delivery,
                                                              modified_with_pd
                                                                .begin(),
                                                              modified_with_pd
                                                                .end(),
                                                              pickup_r,
                                                              delivery_r) &&
                  current_r
                    .is_valid_addition_for_tour_inclusion(input,
                                                              modified_with_pd
                                                                .begin(),
                                                              modified_with_pd
                                                                .end(),
                                                              pickup_r,
                                                              delivery_r) &&
                  current_r.is_valid_addition_for_tw(input,
                                                     modified_delivery,
                                                     modified_with_pd.begin(),
                                                     modified_with_pd.end(),
                                                     pickup_r,
                                                     delivery_r);

                modified_with_pd.pop_back();

                if (valid) {
                  best_cost = current_cost;
                  best_job_rank = job_rank;
                  best_pickup_r = pickup_r;
                  best_delivery_r = delivery_r;
                  best_modified_delivery = modified_delivery;
                  best_duration_addition = current_add.duration;
                }
              }
            }
          }
        }
      }

      if (best_cost < std::numeric_limits<double>::max()) {
        if (input.jobs[best_job_rank].type == JOB_TYPE::SINGLE) {
          current_r.add(input, best_job_rank, best_r);
          unassigned.erase(best_job_rank);
          keep_going = true;
        }
        if (input.jobs[best_job_rank].type == JOB_TYPE::PICKUP) {
          std::vector<Index> modified_with_pd({best_job_rank});
          std::copy(current_r.route.begin() + best_pickup_r,
                    current_r.route.begin() + best_delivery_r,
                    std::back_inserter(modified_with_pd));
          modified_with_pd.push_back(best_job_rank + 1);

          current_r.replace(input,
                            best_modified_delivery,
                            modified_with_pd.begin(),
                            modified_with_pd.end(),
                            best_pickup_r,
                            best_delivery_r);
          unassigned.erase(best_job_rank);
          unassigned.erase(best_job_rank + 1);
          keep_going = true;
        }

        current_route_duration += best_duration_addition;
      }
    }
  }

  return routes;
}

template <class T> T initial_routes(const Input& input) {
  T routes;
  for (Index v = 0; v < input.vehicles.size(); ++v) {
    routes.emplace_back(input, v, input.zero_amount().size());

    const auto& vehicle = input.vehicles[v];
    auto& current_r = routes.back();

    // Startup load is the sum of deliveries for (single) jobs.
    Amount single_jobs_deliveries(input.zero_amount());
    for (const auto& step : vehicle.steps) {
      if (step.type == STEP_TYPE::JOB and step.job_type == JOB_TYPE::SINGLE) {
        single_jobs_deliveries += input.jobs[step.rank].delivery;
      }
    }
    if (!(single_jobs_deliveries <= vehicle.capacity)) {
      throw InputException("Route over capacity for vehicle " +
                           std::to_string(vehicle.id) + ".");
    }

    // Startup load is the sum of deliveries for (single) jobs.
    Amount current_load = single_jobs_deliveries;

    std::vector<Index> job_ranks;
    std::unordered_set<Index> expected_delivery_ranks;
    for (const auto& step : vehicle.steps) {
      if (step.type != STEP_TYPE::JOB) {
        continue;
      }

      const auto job_rank = step.rank;
      const auto& job = input.jobs[job_rank];
      job_ranks.push_back(job_rank);

      if (!input.vehicle_ok_with_job(v, job_rank)) {
        throw InputException("Missing skill or step out of reach for vehicle " +
                             std::to_string(vehicle.id) + ".");
      }

      switch (step.job_type) {
      case JOB_TYPE::SINGLE: {
        current_load += job.pickup;
        current_load -= job.delivery;
        break;
      }
      case JOB_TYPE::PICKUP: {
        expected_delivery_ranks.insert(job_rank + 1);

        current_load += job.pickup;
        break;
      }
      case JOB_TYPE::DELIVERY: {
        auto search = expected_delivery_ranks.find(job_rank);
        if (search == expected_delivery_ranks.end()) {
          throw InputException("Invalid shipment in route for vehicle " +
                               std::to_string(vehicle.id) + ".");
        }
        expected_delivery_ranks.erase(search);

        current_load -= job.delivery;
        break;
      }
      }

      // Check validity after this step wrt capacity.
      if (!(current_load <= vehicle.capacity)) {
        throw InputException("Route over capacity for vehicle " +
                             std::to_string(vehicle.id) + ".");
      }
    }

    if (vehicle.max_tasks < job_ranks.size()) {
      throw InputException("Too many tasks for vehicle " +
                           std::to_string(vehicle.id) + ".");
    }

    if (!expected_delivery_ranks.empty()) {
      throw InputException("Invalid shipment in route for vehicle " +
                           std::to_string(vehicle.id) + ".");
    }

    // Now route is OK with regard to capacity, precedence and skills
    // constraints.
    if (!job_ranks.empty()) {
      if (!current_r.is_valid_addition_for_tw(input,
                                              single_jobs_deliveries,
                                              job_ranks.begin(),
                                              job_ranks.end(),
                                              0,
                                              0)) {
        throw InputException("Infeasible route for vehicle " +
                             std::to_string(vehicle.id) + ".");
      }

      current_r.replace(input,
                        single_jobs_deliveries,
                        job_ranks.begin(),
                        job_ranks.end(),
                        0,
                        0);
    }
  }

  return routes;
}

using RawSolution = std::vector<RawRoute>;
using TWSolution = std::vector<TWRoute>;

template RawSolution
basic(const Input& input, INIT init, double lambda, SORT sort);

template RawSolution
dynamic_vehicle_choice(const Input& input, INIT init, double lambda, SORT sort);

template RawSolution initial_routes(const Input& input);

template TWSolution
basic(const Input& input, INIT init, double lambda, SORT sort);

template TWSolution
dynamic_vehicle_choice(const Input& input, INIT init, double lambda, SORT sort);

template TWSolution initial_routes(const Input& input);

} // namespace vroom::heuristics
