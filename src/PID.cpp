#include "PID.h"

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_in, double Ki_in, double Kd_in) {
    Kp = Kp_in;
    Ki = Ki_in;
    Kd = Kd_in;

    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;

    total_error = 0.0;
}

void PID::UpdateError(double cte) {
    double temp_d_error = cte - p_error;
    if (temp_d_error != 0){
        d_error = cte - p_error;
        p_error = cte;
        i_error += cte;
    }

}

double PID::TotalError() {
    total_error += (p_error * p_error);
    return total_error;
}

