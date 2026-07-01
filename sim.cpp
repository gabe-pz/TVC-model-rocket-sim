#include <iostream> 
#include <array>
#include <numbers>
#define _USE_MATH_DEFINES
#include <cmath> 

int g_magnitudeThrustVector= 14;

//rocket properties, in SI units
double mass = 1.01074445935;
double gravity = 9.81;
double centerOfPressure = 0.08775954;
double centerOfGravity = 0.4059174;
double distanceToThrustVector = 6477;

double Ixx = 0.0249899588;
double Iyy = 0.0249868814;


double dt = 0.00001; 
int simTime = 5;

std::array<double, 3> forceThrustRf(double gimbalAngleX, double gimbalAngleY, double t);
std::array<double, 4> vectorToPureQuaternion(const std::array<double, 3>& vec);
std::array<double, 4> multiplyQP(const std::array<double, 4>& q, const std::array<double, 4>& p); 
std::array<double, 4> conjugateQuaternion(const std::array<double, 4>&q);
std::array<double, 3> rotateRfToWf(const std::array<double, 4>& stateQuaternion, const std::array<double, 3>& vectorRf); 
double degreesToRads(double angleDegrees);

int main(void){     
    double initalGimbalAngleX = 2;
    double initalGimbalAngleY = 3; 


    //quaterion initaliztion
    std::array<double, 4> stateQ = {1.0, 0.0, 0.0, 0.0};

    //vector initalization
    std::array<double, 3> thrustRf = {0.0, 0.0, 0.0};
    std::array<double, 3> thrustWf = {0.0, 0.0, 0.0};
    std::array<double, 3> sumOfForcesWf = {0.0, 0.0, 0.0};

    for(int i = 0; i < simTime/dt; i++){
        double t = i * dt; 

        thrustRf = forceThrustRf(degreesToRads(initalGimbalAngleX), degreesToRads(initalGimbalAngleY), t);
        thrustWf = rotateRfToWf(stateQ, thrustRf); 

        sumOfForcesWf = {thrustWf[0], thrustWf[1], thrustWf[2]-mass*gravity}; 

        std::cout << "F_x = " << sumOfForcesWf[0] << std::endl;
        
    }



    return 0;
}


std::array<double, 3> forceThrustRf(double gimbalAngleX, double gimbalAngleY, double t){

    if(t > 3){
        std::array<double, 3> forceThrustVectorRf = {0.0, 0.0, 0.0}; 
        return forceThrustVectorRf;
    }
    
    else {
        std::array<double, 3> forceThrustVectorRf = {g_magnitudeThrustVector*std::sin(gimbalAngleY), g_magnitudeThrustVector*std::sin(gimbalAngleX), g_magnitudeThrustVector*std::cos(gimbalAngleY)*std::cos(gimbalAngleX)};   
        return forceThrustVectorRf;
    }
}
std::array<double, 4> vectorToPureQuaternion(const std::array<double, 3>& vec){
    std::array<double, 4> vecToQuaternion = {0.0, vec[0], vec[1], vec[2]} ;

    return vecToQuaternion;
}
std::array<double, 4> multiplyQP(const std::array<double, 4>& q, const std::array<double, 4>& p){
    double qP0 = q[0]*p[0] - q[1]*p[1] - q[2]*p[2] - q[3]*p[3];
    double qP1 = q[0]*p[1] + q[1]*p[0] + q[2]*p[3] - q[3]*p[2];
    double qP2 = q[0]*p[2] - q[1]*p[3] + q[2]*p[0] + q[3]*p[1];
    double qP3 = q[0]*p[3] + q[1]*p[2] - q[2]*p[1] + q[3]*p[0];
    
    
    std::array<double, 4> QP = {qP0, qP1, qP2, qP3};

    return QP; 
}
std::array<double, 4> conjugateQuaternion(const std::array<double, 4>&q){
    std::array<double, 4> qStar = {q[0], -q[1], -q[2], -q[3]};

    return qStar;
}
std::array<double, 3> rotateRfToWf(const std::array<double, 4>& stateQuaternion, const std::array<double, 3>& vectorRf){
    std::array<double, 4> vectorRfToQ = vectorToPureQuaternion(vectorRf);
    std::array<double, 4> q1 = multiplyQP(stateQuaternion, vectorRfToQ);
    std::array<double, 4> conjugateStateQuaternion = conjugateQuaternion(stateQuaternion); 
    std::array<double, 4> vectorWfQ = multiplyQP(q1, conjugateStateQuaternion);
    
    std::array<double, 3> vectorWf = {vectorWfQ[1], vectorWfQ[2], vectorWfQ[3]}; 

    return vectorWf;
}
double degreesToRads(double angleDegrees){
    return (angleDegrees * (M_PI/180.0));
}