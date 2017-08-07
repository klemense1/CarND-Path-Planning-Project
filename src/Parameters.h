//
//  Parameter.h
//  Path_Planning
//
//  Created by Klemens on 06.08.17.
//
//

#ifndef Parameter_h
#define Parameter_h

namespace Parameters {
  // The max s value before wrapping around the track back to 0
  const double max_s = 6945.554;
  
  const double velocity_max = 21;
  
  const double dt = 0.02;
  
  const int n_steps = 50;
  
  size_t n_steps_react = 5;
}

#endif /* Parameter_h */
