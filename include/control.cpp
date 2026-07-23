#include "control.h"

void pidControl(std::array<double, 3>& pidArray, const std::array<double, 3>& pidGains, double& prevError, double& desiredAngle, double rocketAngle, double dt, int axis){
    const double setPoint = 0.0;
    const double maxOut = 5.0;
    double error = setPoint - rad2deg(rocketAngle); 
    
    //p
    double pTerm = pidGains[0]*error; 

    //i
    pidArray[1] =  clamp(pidArray[1] + dt*pidGains[1]*error, -5.0, 5.0); //write to pidArray iTerm
    double iTerm = pidArray[1]; //update iTerm to be used in total output

    //d
    double dTerm = pidGains[2]*((error - prevError) / dt);

    double output = pTerm+iTerm+dTerm;

    prevError = error;

    //computing angles need to move for each axis(0 is x, 1 is y) and clamping
    if(axis == 0){ 
        desiredAngle = clamp(-output, -maxOut, maxOut);
    }
    else{
        desiredAngle = clamp(output, -maxOut, maxOut);
    }
}
void slewServo(double& currentServoAngle, double desiredAngle, double maxRate, double dt){
    //d(theta) = omega*dt, max angular movement servo can do in dt
    double maxUpdate = maxRate*dt;

    //essentially moving to desired angle within steps, and ensuring that each step is within the physical limits
    currentServoAngle += clamp(desiredAngle - currentServoAngle, -maxUpdate, maxUpdate);
}