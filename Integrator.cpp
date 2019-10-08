#include "Integrator.h"
#include <Arduino.h>

Integrator::Integrator(double init, double frame_length) :
frame(frame_length)
{
  prev_value = 0;
  sum = 0;
}

void Integrator::new_value(double value) {
  sum += ((value + prev_value)/2)*frame;
  prev_value = value;
}

double Integrator::get_sum() {
  return sum;
}

void Integrator::reset(double init) {
  sum = init;  
}
