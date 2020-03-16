#include "include/PIDFF.hh"

PIDFF::PIDFF()
{

}

PIDFF::~PIDFF()
{

}

void PIDFF::init(double kp, double ki, double kd, double kff, double imin, double imax, double min, double max)
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->kff = kff;
    this->imin = imin;
    this->imax = imax;
    this->min = min;
    this->max = max;
}

double PIDFF::update(double target, double error, double dt)
{
    double p, i, d, ff;
    p = i = d = ff = 0;

    p = this->kp * error;
    
}

void PIDFF::resetIntegrator()
{
    this->integrator = 0.0;
}