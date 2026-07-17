#pragma once 
#include <array> 
#include <cmath> 

double clamp(double val, double min, double max);
std::array<double, 4> vectorToPureQuaternion(const std::array<double, 3>& vec);
std::array<double, 4> multiplyQP(const std::array<double, 4>& q, const std::array<double, 4>& p); 
std::array<double, 4> conjugateQuaternion(const std::array<double, 4>&q);
std::array<double, 3> rotateRfToWf(const std::array<double, 4>& stateQuaternion, const std::array<double, 3>& vectorRf); 
std::array<double, 3> rotateWfToRf(const std::array<double, 4>& stateQuaternion, const std::array<double, 3>& vectorWf); 
std::array<double, 3> crossProduct(const std::array<double, 3>& a, const std::array<double, 3>& b);
std::array<double, 2> quaternionToEuler(const std::array<double, 4>& stateQuaternion);
void quatToMat(const std::array<double,4>& q, float m[16]);
void normalizeQuaternion(std::array<double, 4>& q);  