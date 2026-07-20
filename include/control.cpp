#include "control.h"

void pidControl(std::array<double, 3>& pidArray, const std::array<double, 3>& pidGains, double& prevError, double& desiredAngle, double rocketAngle, double dt, int axis){
    const double setPoint = 0.0;
    const double maxOut = 5.0;
    double error = setPoint - rad2deg(rocketAngle); 
    
    double pTerm = pidGains[0]*error; 
    double iTerm = clamp(pidArray[1] + dt*pidGains[1]*error, -maxOut, maxOut);
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

    //essentially is breaking the movement up to where it needs to go into small steps it can take to get there
    //where those steps are characterized by the max angular v of the servos
    currentServoAngle += clamp(desiredAngle - currentServoAngle, -maxUpdate, maxUpdate);
}