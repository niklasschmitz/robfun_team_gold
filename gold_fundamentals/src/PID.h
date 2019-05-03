#ifndef SRC_PID_H
#define SRC_PID_H

class PID {
public:
    PID();
    PID(double max, double min, double Kp, double Ki, double Kd);

    ~PID();

    double calculate(double setpoint, double pv, double dt);
    void reset();
private:

    double _max;
    double _min;
    double _Kp;
    double _Ki;
    double _Kd;
    double _pre_error;
    double _integral;
};

#endif //SRC_PID_H
