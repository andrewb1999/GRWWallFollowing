#include "ConstrainedIntegrator.h"
#include <Arduino.h>

ConstrainedIntegrator::ConstrainedIntegrator(double init, double int_windup_min, 
double int_windup_max, double frame_length) :
sum(init),
max_value(int_windup_max),
min_value(int_windup_min),
frame(frame_length)
{
  prev_value = 0;
}

void ConstrainedIntegrator::new_value(double value) {
  sum = constrain(sum + ((value + prev_value)/2)*frame, min_value, max_value);
  prev_value = value;
}

double ConstrainedIntegrator::get_sum() {
  return sum;
}
