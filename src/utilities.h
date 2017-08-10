// Copyright [2017] Klemens Esterle

#ifndef SRC_UTILITIES_H_
#define SRC_UTILITIES_H_

#include <vector>

namespace utilities {
  
  
  // For converting back and forth between radians and degrees.
  constexpr double pi() { return M_PI; }
  double deg2rad(double x) { return x * pi() / 180; }
  double rad2deg(double x) { return x * 180 / pi(); }
  
  /*
   from https://stackoverflow.com/questions/11734322/matlab-type-arrays-in-c
   */
  
  std::vector<double> linspace(double a, double b, int n) {
    std::vector<double> array;
    if ((n == 0) || (n == 1) || (a == b)) {
      array.push_back(b);
    } else if (n > 1) {
      double step = (b - a) / (n - 1);
      int count = 0;
      while (count < n) {
        array.push_back(a + count*step);
        ++count;
      }
    }
    return array;
  }
  
  double bound_s(double s) {
    const double max_s = 6945.554;
    double bounded_s;
    if (s < 0 || s > max_s) {
      bounded_s = fmod(max_s + s, max_s);
    } else {
      bounded_s = s;
    }
    return bounded_s;
  }
}  // namespace utilities

#endif  // SRC_UTILITIES_H_
