#pragma once 
#include <array> 
#include <cmath> 

//rocket global constants
inline const double tBurn = 3.45;

std::array<double, 3> forceThrustRf(double gimbalAngleX, double gimbalAngleY, double t);
double magnitudeThrust(double t);
double mass(double t); 