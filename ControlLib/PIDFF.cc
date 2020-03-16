#include "include/PIDFF.hh"

PIDFF::PIDFF(double p, double i, double d, double ff, double iMin, double iMax, double Min, double Max)
    : kp(p), ki(i), kd(d), kff(ff), imin(iMin), imax(iMax), min(Min), max(Max)
{
    resetIntegrator();
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
    if(dt <= 0.0){
        return -1;
    }

    double p, d, ff;
    p = d = ff = 0;

    // Calculate proportional term
    p = this->kp * error; 
    
    // Calculate intergal term
    this->integrator += this->ki * error * dt;
    this->integrator = this->integrator > this->imax ? this->imax : this->integrator < this->imin ? this->imin : this->integrator; // anti-windup

    // Calculate derivative term
    d = this->kd * (error - lastError) / dt;
    
    // Calculate feedforward term
    ff = this->kff * target;

    return p + integrator + d + ff;
}

void PIDFF::resetIntegrator()
{
    this->integrator = 0.0;
}