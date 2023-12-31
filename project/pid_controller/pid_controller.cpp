/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
   Kp = Kpi;
   Ki = Kii;
   Kd = Kdi;

   output_lim_max = output_lim_maxi;
   output_lim_min = output_lim_mini;

   cte_der = 0.0;
   cte_int = 0.0;
   cte_prev = 0.0;

   dt = 0.0;
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
   cte_der = (dt > 0.0) ? (cte - cte_prev) / dt : 0.0;
   cte_int += cte * dt;
   cte_prev = cte;
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
    double control = Kp * cte_prev + Kd * cte_der + Ki * cte_int;
    if (control < output_lim_min) {
      control = output_lim_min;
    } else if (control > output_lim_max) {
      control = output_lim_max;
    }
    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
   dt = new_delta_time;
   return dt;
}