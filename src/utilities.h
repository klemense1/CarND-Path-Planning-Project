//
//  utilities.h
//  Path_Planning
//
//  Created by Klemens on 06.08.17.
//
//

#ifndef utilities_h
#define utilities_h

#include <vector>

using namespace std;

namespace utilities {
  
  
  // For converting back and forth between radians and degrees.
  constexpr double pi() { return M_PI; }
  double deg2rad(double x) { return x * pi() / 180; }
  double rad2deg(double x) { return x * 180 / pi(); }
  
  /*
   from https://stackoverflow.com/questions/11734322/matlab-type-arrays-in-c
   */
  vector<double> linspace(double a, double b, int n) {
    vector<double> array;
    if ((n == 0) || (n == 1) || (a == b))
      array.push_back(b);
    else if (n > 1) {
      double step = (b - a) / (n - 1);
      int count = 0;
      while(count < n) {
        array.push_back(a + count*step);
        ++count;
      }
    }
    return array;
  }
  
}
#endif /* utilities_h */
