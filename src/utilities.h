// Copyright [2017] Klemens Esterle

#ifndef SRC_UTILITIES_H_
#define SRC_UTILITIES_H_

#include <vector>

namespace utilities {
  
  const double max_s = 6945.554;

  // For converting back and forth between radians and degrees.
  constexpr double pi() { return M_PI; }
  inline double deg2rad(double x) { return x * pi() / 180; }
  inline double rad2deg(double x) { return x * 180 / pi(); }
  
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
  
  inline double bound_s(double s) {
    double bounded_s;
    if (s < 0 || s > max_s) {
      bounded_s = fmod(max_s + s, max_s);
    } else {
      bounded_s = s;
    }
    return bounded_s;
  }
  
  inline double bound_s_difference(double s1, double s2) {
    auto a = bound_s(s1);
    auto b = bound_s(s2);
    auto diff = a - b;
    if (diff < -max_s / 2) diff += max_s;
    if (diff > max_s / 2) diff -= max_s;
    return diff;
  }

}  // namespace utilities

#endif  // SRC_UTILITIES_H_
