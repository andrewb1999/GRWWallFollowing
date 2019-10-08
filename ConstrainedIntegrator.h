#ifndef CONSTRAINEDINTERGRATOR_H
#define CONSTRAINEDINTEGRATOR_H

using namespace std;

class ConstrainedIntegrator
{
  private:
    double sum;
    double max_value;
    double min_value;
    double frame;
    double prev_value;
  
  public:
    ConstrainedIntegrator(double init, double int_windup_min, 
    double int_windup_max, double frame_length);
    void new_value(double value);
    double get_sum();
};

#endif
