/*
  control_pid.hpp

  Created on: Feb 25, 2019

  Author: Biggs
*/

double Control_PID(double  L_DesiredSpeed,
                   double  L_CurrentSpeed,
                   double *L_ErrorPrev,
                   double *L_IntegralPrev,
                   double  L_ProportionalGx,
                   double  L_IntegralGx,
                   double  L_DerivativeGx,
                   double  L_ProportionalUpperLimit,
                   double  L_ProportionalLowerLimit,
                   double  L_IntegralUpperLimit,
                   double  L_IntegralLowerLimit,
                   double  L_DerivativeUpperLimit,
                   double  L_DerivativeLowerLimit,
                   double  L_OutputUpperLimit,
                   double  L_OutputLowerLimit);