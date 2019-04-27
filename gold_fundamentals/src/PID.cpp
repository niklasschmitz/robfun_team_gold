class PID {
public:
    PID(double dt, double max, double min, double Kp, double Ki, double Kd);

    ~PID();

    double calculate(double setpoint, double pv);
    double _dt;
private:

    double _max;
    double _min;
    double _Kp;
    double _Ki;
    double _Kd;
    double _pre_error;
    double _integral;
};


PID::PID(double dt, double max, double min, double Kp, double Ki, double Kd) :
        _dt(dt),
        _max(max),
        _min(min),
        _Kp(Kp),
        _Ki(Ki),
        _Kd(Kd),
        _pre_error(0),
        _integral(0) {
}

double PID::calculate(double setpoint, double pv) {

    // Calculate error
    double error = setpoint - pv;


    // Proportional term
    double Pout = _Kp * error;

    // Integral term
    _integral += error * _dt;
    double Iout = _Ki * _integral;

    // Derivative term
    double derivative = (error - _pre_error) / _dt;
    double Dout = _Kd * derivative;

    // Calculate total output
    double output = Pout + Iout + Dout;

    // Restrict to max/min
    if (output > _max)
        output = _max;
    else if (output < _min)
        output = _min;

    // Save error to previous error
    _pre_error = error;

    return output;
}

PID::~PID() {
}

