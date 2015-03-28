#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <vector>

using namespace std;

#define DEBUG 1

#if DEBUG
#define DPRINTF(...) do { fprintf( stderr, __VA_ARGS__); } while (false)
#else
#define DPRINTF(...) do { } while (false)
#endif

#define DEXPR(expr) do { DPRINTF("%s -> %g\n", #expr, (expr)); } while (false)

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


int RoundUp(float num) {
  float x = (num + .5);
  DEXPR(num);
  DEXPR(x);
  DEXPR(static_cast<int>(ceil(x)));
  return static_cast<int>(ceil(x));
}

int main(int argc, char **argv) {
  // inputs!
  float vprog = 43.0; // ft/s
  float dist  = 35; // ft
  float t1    = 200; // ms
  float t2    = 100; // ms
  float itp   = 45; // ms

  // derived!
  float t4    = dist / vprog * 1000.0; // time in ms to get to destination
  DEXPR(t4);
  float n     = RoundUp(t4 / itp); // total number of inputs to the filter
  DEXPR(n);
  int   fl1   = RoundUp(t1 / itp);
  DEXPR(fl1);
  int   fl2   = RoundUp(t2 / itp);
  DEXPR(fl2);

  vector<float> fl1_array(fl2);
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

  printf("%.4s %8s %.5s %.5s %.5s %6s %6s %6s\n",
    "Step",
    "Time",
    "Input",
    "F1sum",
    "F2sum",
    "Vel",
    "Pos",
    "Acc"
  );
  do {
    printf("%.4d %3.5f     %d %.3f %.3f %.4f %.4f %.4f\n",
      step,
      the_time,
      input,
      fl1_sum,
      fl2_sum,
      vel,
      pos,
      acc);

    ++step;
    the_time += itp / 1000.0;

    input = (step < n + 2) ? 1 : 0;
    float fl1_add = (input == 1) ? (1.0 / fl1) : (-1.0 / fl1);

    // sum filter 1 and divide by number of steps in filter 1
    // as input to filter 2.
    fl1_sum += max(0.0f, min(1.0f,fl1_add));
    fl1_array[fl1_array_idx++] = fl1_sum;
    if (fl1_array_idx >= fl1) {
      fl1_array_idx = 0;
    }

    for (int i = 0, fl2_sum = 0; i < fl1; ++i) {
      fl2_sum += fl1_array[i];
    }

    float oldvel = vel;
    float oldpos = pos;
    vel = fl1_sum + fl2_sum / (1.0 + fl2) * vprog;
    pos = (vel + oldvel)/2.0 * itp / 1000.0 + oldpos;
    acc = (vel - oldvel)/(itp / 1000);
  } while ((fl1_sum > 0 || fl2_sum > 0) && step < 30);
  return 0;
}
