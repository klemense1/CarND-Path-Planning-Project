// Copyright [2017] Klemens Esterle

#ifndef SRC_PARAMETERS_H_
#define SRC_PARAMETERS_H_

namespace Parameters {
  // The max s value before wrapping around the track back to 0
  const double max_s = 6945.554;
  
  const double velocity_max = 21;
  
  const double dt = 0.02;
  
  const int n_steps = 50;
  
  size_t n_steps_react = 5;
}  // namespace Parameters

#endif  // SRC_PARAMETERS_H_
