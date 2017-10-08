#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>

#include "Eigen-3.3/Eigen/Dense"
#include "jmt.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

std::vector<double> getMinJerk(const std::vector<double>& start, 
	const std::vector<double>& end, double T)
{
    /*
     * Calculates the Jerk Minimizing Trajectory that connects the initial state
     * to the final state in Time T
     *
     * Inputs
     * start - the vehicle start location given as a length three array
     *         corresponding to initial values of [s, s_dot, s_double_dot]
     * end   - the desired end state for vehicle. Like "start" this is a 
     *         length three array
     * T     - The duration, in seconds, over which this maneuver should occur.
     *
     * Output
     * an array of length 6, each value corresponding to a coefficient in the polynomial
     * s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
     */
    MatrixXd A(3,3);
    double T2 = T*T, T3 = T2*T, T4 = T3*T, T5 = T4*T;
    A << T3, T4, T5,
         3*T2, 4*T3, 5*T4,
         6*T, 12*T2, 20*T3;
    
    MatrixXd B(3,1);
    B << end[0] - (start[0] + start[1]*T + .5*start[2]*T2),
         end[1] - (start[1] + start[2]*T),
         end[2] - start[2];

    MatrixXd Ai = A.inverse();
    MatrixXd C = Ai*B;

    std::vector<double> result = {start[0], start[1], 0.5*start[2]};
    for (int i = 0; i < C.size(); ++i)
    {
	result.push_back(C.data()[i]);
    }
    return result;
}

double
evaluate(const std::vector<double>& coeffs, double x)
{
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); ++i)
    {
	result += coeffs[i] * std::pow(x, i);
    }
    return result;
}
