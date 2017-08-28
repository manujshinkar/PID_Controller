#include "PID.h"
#include <iostream>
//using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {

	Kp = Kp_;
	Ki = Ki_;
	Kd = Kd_;

    prev_cte = -999;
}

void PID::UpdateError(double cte) {

    double cte_filtered = cte;

    if(prev_cte != -999) {
        cte_filtered = prev_cte + 0.8 * (cte - prev_cte);
    } else {
        prev_cte = cte;
    }

	p_error = cte_filtered;
	d_error = cte_filtered - prev_cte;
	i_error += cte_filtered;

	prev_cte = cte_filtered;

    //std::cout << cte << "," << cte_filtered << std::endl;
}

double PID::TotalError() {

	return 0;
}

double PID::calculateSteering() {

	double steering = - Kp * p_error - Ki * i_error - Kd * d_error; 
	return steering;
}

void twiddle()	{

// 	double tol = 0.02; 
//     double p[3];
//     double dp[3] = {0.01, 0.0001, 0.1};
    
//     int it = 0;
//     double dum
//     while sum(dp) > tol:
//         //print("Iteration {}, best error = {}".format(it, best_err))
//         for i in range(len(p)):
//             p[i] += dp[i]
//             robot = make_robot()
//             x_trajectory, y_trajectory, err = run(robot, p)
    
//             if err < best_err:
//                 best_err = err
//                 dp[i] *= 1.1
//             else:
//                 p[i] -= 2 * dp[i]
//                 robot = make_robot()
//                 x_trajectory, y_trajectory, err = run(robot, p)
    
//                 if err < best_err:
//                     best_err = err
//                     dp[i] *= 1.1
//                 else:
//                     p[i] += dp[i]
//                     dp[i] *= 0.9
//         it += 1
//     return p, best_err
}

