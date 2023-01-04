#ifndef COST_WRAPPER_H
#define COST_WRAPPER_H

/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/generic/matrix.h"
#include "structures/typedefs.h"

namespace vroom {

class CostWrapper {
private:
  const Duration discrete_duration_factor;
  std::size_t duration_matrix_size;
  const UserDuration* duration_data;

  Cost discrete_cost_factor;
  std::size_t cost_matrix_size;
  const UserCost* cost_data;

  Cost _per_hour;
  bool _cost_based_on_duration;

  Energy discrete_energy_factor;
  std::size_t energy_matrix_size;
  const UserEnergy* energy_data;

public:
  CostWrapper(double speed_factor, Cost per_hour);

  void set_durations_matrix(const Matrix<UserDuration>* matrix);

  void set_costs_matrix(const Matrix<UserCost>* matrix,
                        bool reset_cost_factor = false);

  void set_energy_matrix(const Matrix<UserEnergy>* matrix);

  Duration get_discrete_duration_factor() const {
    return discrete_duration_factor;
  }

  bool cost_based_on_duration() const {
    return _cost_based_on_duration;
  }

  Duration duration(Index i, Index j) const {
    return discrete_duration_factor *
           static_cast<Duration>(duration_data[i * duration_matrix_size + j]);
  }

  Cost cost(Index i, Index j) const {
    return discrete_cost_factor *
           static_cast<Cost>(cost_data[i * cost_matrix_size + j]);
  }

  Energy energy(Index i, Index j) const {
    if (energy_matrix_size == 0) {
      return 0;
    } else {
      return -discrete_energy_factor *
            static_cast<Energy>(energy_data[i * energy_matrix_size + j]);
    }
  }

  UserCost user_cost_from_user_duration(UserDuration d) const;
};

} // namespace vroom

#endif
