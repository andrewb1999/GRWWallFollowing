#ifndef DISTANCEINTERGRATOR_H
#define DISTANCEINTEGRATOR_H

using namespace std;

class Integrator
{
  private:
    double sum;
    double frame;
    double prev_value;
  
  public:
    Integrator(double init, double frame_length);
    void new_value(double value);
    double get_sum();
    void reset(double init);
};

#endif
