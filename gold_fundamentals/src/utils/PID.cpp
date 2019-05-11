#include "PID.h"

PID::PID() {

}

PID::PID(double max, double min, double Kp, double Ki, double Kd) :
        _max(max),
        _min(min),
        _Kp(Kp),
        _Ki(Ki),
        _Kd(Kd),
        _pre_error(0),
        _integral(0) {
}

double PID::calculate(double setpoint, double pv, double dt) {
    // Calculate error
    double error = setpoint - pv;

    // Proportional term
    double Pout = _Kp * error;

    // Integral term
    _integral += error * dt;
    double Iout = _Ki * _integral;

    // avoid division by zero
    if (dt < 0.001) dt = 0.001;

    // Derivative term
    double derivative = (error - _pre_error) / dt;
    double Dout = _Kd * derivative;

    // Calculate total output
    double output = Pout + Iout + Dout;

    // Restrict to max/min
    if (output > _max) {
        output = _max;
    } else if (output < _min) {
        output = _min;
    }

    // Save error to previous error
    _pre_error = error;

    return output;
}

void PID::reset() {
    this->_pre_error = 0;
    this->_integral = 0;
}

PID::~PID() {
}

