#include "rocketMath.h"


double clamp(double val, double min, double max){
    if(val < min) return min;
    else if(val > max) return max;
    else return val;

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
std::array<double, 3> rotateWfToRf(const std::array<double, 4>& stateQuaternion, const std::array<double, 3>& vectorWf){
    std::array<double, 4> vectorWfToQ = vectorToPureQuaternion(vectorWf);
    std::array<double, 4> conjugateStateQuaternion = conjugateQuaternion(stateQuaternion); 
    std::array<double, 4> q1 = multiplyQP(conjugateStateQuaternion, vectorWfToQ);

    std::array<double, 4> vectorRfQ = multiplyQP(q1, stateQuaternion);
    std::array<double, 3> vectorRf = {vectorRfQ[1], vectorRfQ[2], vectorRfQ[3]}; 

    return vectorRf;
}
std::array<double, 3> crossProduct(const std::array<double, 3>& a, const std::array<double, 3>& b){
    std::array<double, 3> aCrossb = {a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]};

    return aCrossb;
}
std::array<double, 2> quaternionToEuler(const std::array<double, 4>& stateQuaternion){

    double aSinArg = clamp(2*(stateQuaternion[0]*stateQuaternion[2] - stateQuaternion[1]*stateQuaternion[3]), -1.0, 1.0);

    double aTan2Arg1 = 2*(stateQuaternion[0]*stateQuaternion[1]+stateQuaternion[2]*stateQuaternion[3]);
    double aTan2Arg2 = 1-2*(stateQuaternion[1]*stateQuaternion[1]+stateQuaternion[2]*stateQuaternion[2]); 

    std::array<double, 2> angles = {std::asin(aSinArg), std::atan2(aTan2Arg1, aTan2Arg2)};

    //returned as psi = (phi, theta)
    return angles;
}
void normalizeQuaternion(std::array<double, 4>& q){
    double w = q[0]*q[0];
    double x = q[1]*q[1];
    double y = q[2]*q[2];
    double z = q[3]*q[3]; 

    double magnitude = std::sqrt(w+x+y+z); 

    q[0] = (q[0] / magnitude);
    q[1] = (q[1] / magnitude);
    q[2] = (q[2] / magnitude);
    q[3] = (q[3] / magnitude);
}
void quatToMat(const std::array<double,4>& q, float m[16]) {
    double w=q[0], x=q[1], y=q[2], z=q[3];

    double r00=1-2*(y*y+z*z), r01=2*(x*y-w*z),   r02=2*(x*z+w*y);
    double r10=2*(x*y+w*z),   r11=1-2*(x*x+z*z), r12=2*(y*z-w*x);
    double r20=2*(x*z-w*y),   r21=2*(y*z+w*x),   r22=1-2*(x*x+y*y);

    m[0]=r00; m[1]=r10; m[2]=r20; m[3]=0;
    m[4]=r01; m[5]=r11; m[6]=r21; m[7]=0;  
    m[8]=r02; m[9]=r12; m[10]=r22;m[11]=0;
    m[12]=0;  m[13]=0;  m[14]=0;  m[15]=1; 
}