#include <cstdio>
#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <cmath>
#include <vector>

using namespace std;
// class MotionProfileController {
// public:
//   MotionProfileController();
//   ~MotionProfileController();
//
//   void SetupMotion(float max_speed,
//                    float distance,
//                    float t1,
//                    float t2,
//                    float itp,
//                  );
//
// private:
//   float t4;
//   float fl1_sum;
//   float fl2_sum;
//   float N;
//   int step;
// }

#define DEBUG 0

int RoundUp(float num) {
  float x = num;
  return ceil(num);
}

template <typename T>
void DumpVector(std::vector<T> v, ostream & out) {
#if DEBUG
  out << "[ ";
  for (int i = 0; i < v.size(); ++i) {
    out << v[i] << " ";
  }
  out << "]" << endl;
#endif
}

int main(int argc, char **argv) {
  int max_samples = 30;
  if (argc > 1) {
    max_samples = atoi(argv[1]);
  }
  if (max_samples <= 0) {
    cerr << "If you're going to provide a maximum output, make it positive! :)" << endl;
    return 1;
  }
  // inputs!
  float vprog = 43.0; // ft/s
  float dist  = 35; // ft
  float t1    = 200; // ms
  float t2    = 100; // ms
  float itp   = 45; // ms

  // derived!
  float t4    = dist / vprog * 1000.0; // time in ms to get to destination
  float n     = RoundUp(t4 / itp); // total number of inputs to the filter
  int   fl1   = RoundUp(t1 / itp);
  int   fl2   = RoundUp(t2 / itp);

  cout << "vprog = " << vprog << endl;
  cout << "dist  = " << dist << endl;
  cout << "t1    = " << t1 << endl;
  cout << "t2    = " << t2 << endl;
  cout << "itp   = " << itp << endl;
  cout << endl;
  cout << "t4  = " << t4 << endl;
  cout << "n   = " << n << endl;
  cout << "fl1 = " << fl1 << endl;
  cout << "fl2 = " << fl2 << endl;

  vector<float> fl1_array;
  for (int i = 0; i < fl2; ++i) {
    fl1_array.push_back(0);
  }
  int fl1_array_idx = 0;

  // intermediate!
  int   input   = 0;
  float fl1_sum = 0;
  float fl2_sum = 0;

  // output!
  float vel = 0;
  float pos = 0;
  float acc = 0;

  // my stuff!
  int step = 1;
  float the_time = 0;

  cout << setw(4) << "Step" << ' ';
  cout << setw(6) << "Time" << ' ';
  cout << setw(5) << "Input" << ' ';
  cout << setw(5) << "F1Sum" << ' ';
  cout << setw(5) << "F2Sum" << ' ';
  cout << setw(6) << "Vel"   << ' ';
  cout << setw(6) << "Pos"   << ' ';
  cout << setw(6) << "Acc"   << endl;

  do {
    cout.setf( std::ios::fixed, std::ios::floatfield );
    cout << setw(4) << step << ' ';
    cout << setw(6) << setprecision(4) << the_time << ' ';
    cout << setw(5) << input << ' ';
    cout << setw(5) << setprecision(3) << fl1_sum << ' ';
    cout << setw(5) << setprecision(3) << fl2_sum << ' ';
    cout << setw(6) << setprecision(4) << vel << ' ';
    cout << setw(6) << setprecision(4) << pos << ' ';
    cout << setw(6) << setprecision(4) << acc << endl;
    cout.unsetf( std::ios::floatfield );

    ++step;
    the_time += itp / 1000.0;

    input = (step < n + 2) ? 1 : 0;
    float fl1_add = (input == 1) ? (1.0 / fl1) : (-1.0 / fl1);

    // sum filter 1 and divide by number of steps in filter 1
    // as input to filter 2.

    DumpVector(fl1_array, cout);
    fl1_sum = max(0.0f, min(1.0f, fl1_sum + fl1_add));
    fl1_array[fl1_array_idx++] = fl1_sum;
    if (fl1_array_idx >= fl2) {
      fl1_array_idx = 0;
    }

    fl2_sum = 0;
    for (int i = 0; i < fl2; ++i) {
      fl2_sum += fl1_array[i];
    }

    float oldvel = vel;
    float oldpos = pos;
    vel = ((fl1_sum + fl2_sum) / (1.0 + fl2)) * vprog;
    pos = (vel + oldvel)/2.0 * itp / 1000.0 + oldpos;
    acc = (vel - oldvel)/(itp / 1000);
  } while ((fl1_sum > 0 || fl2_sum > 0) && step < max_samples);
  return 0;
}
