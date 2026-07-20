#pragma once 

#include <array> 
#include "rocketMath.h"

void pidControl(std::array<double, 3>& pidArray, const std::array<double, 3>& pidGains, double& prevError, double& desiredAngle, double rocketAngle, double dt, int axis);
void slewServo(double& currentServoAngle, double desiredAngle, double maxRate, double dt);