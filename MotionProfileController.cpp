#include "MotionProfileController.h"
#include <iostream>
#include <cmath>
#include <algorithm>

MotionProfileStep::MotionProfileStep(float the_time, float position, float velocity, float acceleration) {
  this->the_time = the_time;
  this->position = position;
  this->velocity = velocity;
  this->acceleration = acceleration;
}

MotionProfileStep::~MotionProfileStep() {

}

float MotionProfileStep::Time() {
  return the_time;
}

float MotionProfileStep::Position() {
  return position;
}

float MotionProfileStep::Velocity() {
  return velocity;
}

float MotionProfileStep::Acceleration() {
  return acceleration;
}

MotionProfile::MotionProfile() {

}

MotionProfile::~MotionProfile() {

}

int MotionProfile::Steps() {
  return steps.size();
}

void MotionProfile::AppendStep(MotionProfileStep step) {
  steps.push_back(step);
}

MotionProfileStep MotionProfile::operator[](int idx) {
  return steps[idx]; // range checking is the user's problem
}

MotionProfileController::MotionProfileController() {
}

MotionProfileController::~MotionProfileController() {

}

MotionProfile MotionProfileController::ComputeMotion(
                                            float max_speed,
                                            float distance,
                                            float t1,
                                            float t2,
                                            float itp
                                          )
{
  float vprog = max_speed;
  float t4   = distance / vprog * 1000.0;
  float N    = ::ceil(t4 / itp);
  float fl1  = ::ceil(t1 / itp);
  float fl2  = ::ceil(t2 / itp);

  float position = 0.0f;
  float velocity = 0.0f;
  float acceleration = 0.0f;

  int step = 1;
  float the_time = 0.0f;

  std::vector<float> previous_fl1_sums;
  for (int i = 0; i < fl2; ++i) {
    previous_fl1_sums.push_back(0);
  }
  int fl1_array_idx = 0;
  float fl1_sum = 0.0f;
  float fl2_sum = 0.0f;

  MotionProfile profile;


  do {
    MotionProfileStep the_step(the_time, position, velocity, acceleration);
    profile.AppendStep(the_step);

    ++step;
    the_time += itp;

    int input = (step < N + 2) ? 1 : 0; // Input to the filterz

    // Incremental amount to be used for fl1
    float fl1_add = (input==1) ? (1.0 / fl1) : (-1.0 / fl1);

    fl1_sum = std::max(0.0f, std::min(1.0f, fl1_sum + fl1_add));

    previous_fl1_sums[fl1_array_idx++] = fl1_sum;
    if (fl1_array_idx >= fl2) {
      fl1_array_idx = 0;
    }

    fl2_sum = 0;
    for (int i = 0; i < fl2; ++i) {
      fl2_sum += previous_fl1_sums[i];
    }

    float oldvel = velocity;
    float oldpos = position;
    velocity     = ((fl1_sum + fl2_sum) / (1.0 + fl2)) * vprog;
    position     = (velocity + oldvel)/2.0 * itp / 1000.0 + oldpos;
    acceleration = (velocity - oldvel)/(itp / 1000);
  } while ((fl1_sum > 0 || fl2_sum > 0));
  return profile;
}
