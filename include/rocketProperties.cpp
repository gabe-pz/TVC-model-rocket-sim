#include "rocketProperties.h" 

std::array<double, 3> forceThrustRf(double gimbalAngleX, double gimbalAngleY, double t){    
    if(t > tBurn){
        std::array<double, 3> forceThrustVectorRf = {0.0, 0.0, 0.0}; 
        return forceThrustVectorRf;
    }
    
    else {
        std::array<double, 3> forceThrustVectorRf = {magnitudeThrust(t)*std::sin(gimbalAngleX), magnitudeThrust(t)*std::sin(gimbalAngleY), magnitudeThrust(t)*std::cos(gimbalAngleX)*std::cos(gimbalAngleY)};   
        return forceThrustVectorRf;
    }
}

double magnitudeThrust(double t){
    return (-2.32*t+17.76);
}
double mass(double t){
    const double massRocketEmpty = 0.95;
    if(t > tBurn) return massRocketEmpty;
    else return ((-17.571*t + 60.62)/1000.0 + massRocketEmpty);
}
