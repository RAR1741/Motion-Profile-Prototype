/*
MotionProfileController.h

Set of related classes for doing motion profile planning
and execution.

P. Frampton, 2015
*/

#ifndef MOTION_PROFILE_H__
#define MOTION_PROFILE_H__

#include <vector>


// An individual step to complete, part of a full motion profile
class MotionProfileStep {
public:
  MotionProfileStep(float the_time, float position, float velocity, float acceleration = 0.0f);
  ~MotionProfileStep();

  float Time(); // Time from beginning where this step occurs
  float Position(); // Position from start where the robot should be at Time()
  float Velocity(); // Velocity of the robot system at Time()
  float Acceleration(); // Acceleration at this time (optional)
private:
  float the_time;
  float position;
  float velocity;
  float acceleration;
};

class MotionProfile {
public:
  MotionProfile();
  ~MotionProfile();

  int Steps();
  void AppendStep(MotionProfileStep step);

  MotionProfileStep operator[](int idx);
private:
  std::vector<MotionProfileStep> steps;
};

class MotionProfileController {
public:
  MotionProfileController();
  ~MotionProfileController();

  MotionProfile ComputeMotion(
                     float max_speed, // maximum rate of change in units/s
                     float distance,  // distance to move in units (+/-)
                     float t1,        // filter 1 time
                     float t2,        // filter 2 time
                     float itp);       // iteration period, in Hz

private:
};

#endif
