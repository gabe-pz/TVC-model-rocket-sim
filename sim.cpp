#include <iostream> 
#include <array>
#include <cmath> 
#include <raylib.h> 
#include <rlgl.h> 

std::array<double, 3> forceThrustRf(double gimbalAngleX, double gimbalAngleY, double t, double magnitudeThrustVector);
std::array<double, 4> vectorToPureQuaternion(const std::array<double, 3>& vec);
std::array<double, 4> multiplyQP(const std::array<double, 4>& q, const std::array<double, 4>& p); 
std::array<double, 4> conjugateQuaternion(const std::array<double, 4>&q);
std::array<double, 3> rotateRfToWf(const std::array<double, 4>& stateQuaternion, const std::array<double, 3>& vectorRf); 
std::array<double, 3> crossProduct(const std::array<double, 3>& a, const std::array<double, 3>& b);
std::array<double, 2> quaternionToEuler(const std::array<double, 4>& stateQuaternion);
void quatToMat(const std::array<double,4>& q, float m[16]);
void normalizeQuaternion(std::array<double, 4>& q); 

int main(void){     
    //rocket properties, in SI units
    const double magnitudeThrustVector= 14.44;
    //const double centerOfPressure = 0.0877;
    const double distanceToThrustVector = 0.6477;

    double centerOfGravity = 0.405;
    double mass = 1.01;

    long double Ixx = 0.0249899588;
    long double Iyy = 0.0249868814;

    //physical constants
    const double gravity = 9.81;

    //simulation constants
    double dt = 0.000001; 
    int simTime = 15;

    //gimbal angle initalization
    double gimbalAngleX = 0.02;
    double gimbalAngleY = 0.2; 

    //raylib initalization
    const int screenWidth  = 800;
    const int screenHeight = 450;
    bool landed = false;
    InitWindow(screenWidth, screenHeight, "tvc-model-rocket-sim");

    //Define the camera to look into our 3d world
    Camera3D camera = { 0 };
    camera.position   = (Vector3){ 10.0f, 10.0f, 10.0f };  // Camera position
    camera.target     = (Vector3){ 0.0f, 0.0f, 0.0f };     // Camera looking-at point
    camera.up         = (Vector3){ 0.0f, 1.0f, 0.0f };     // Camera up vector
    camera.fovy       = 45.0f;                             // Field-of-view Y
    camera.projection = CAMERA_PERSPECTIVE;
    DisableCursor();    

    //raylib sync 
    int FPS = 60;
    SetTargetFPS(FPS); 
    int iterationsPerFrame = (1/(dt*FPS)); 
    double t = 0.0;

    //Raylib rocket Dimensions
    float bodyRadius = 0.25f;
    float bodyHeight = 2.5f;
    float coneHeight = 0.7;
    float coneTopRadius = 0.05f; 
    float cgFrac = (distanceToThrustVector - centerOfGravity) / distanceToThrustVector;                             


    //coordinate transformation matrix between raylib vecs and my own vecs
    float cf[16] = {
        1,0,0,0,
        0,0,-1,0,
        0,1,0,0,
        0,0,0,1
    };

    //quaterion initaliztion
    std::array<double, 4> stateQ = {1.0, 0.0, 0.0, 0.0};
    std::array<double, 4> stateQTimeDerivative = {1.0, 0.0, 0.0, 0.0};
    std::array<double, 4> angularVelocityQ = {1.0, 0.0, 0.0, 0.0};

    //forces initalization
    std::array<double, 3> thrustRf = {0.0, 0.0, 0.0};
    std::array<double, 3> thrustWf = {0.0, 0.0, 0.0};
    std::array<double, 3> sumOfForcesWf = {0.0, 0.0, 0.0}; 

    //torques initalization
    std::array<double, 3> torqueThrust = {0.0, 0.0, 0.0}; 

    //position and its derivatives initalization
    std::array<double, 3> accleration = {0.0, 0.0, 0.0};
    std::array<double, 3> velocity = {0.0, 0.0, 0.0};
    std::array<double, 3> position = {0.0, 0.0, 0.0}; 

    //rotation and its derivatives initalization. Note psi = (theta, phi)
    std::array<double, 3> angularAccleration = {0.0, 0.0, 0.0};
    std::array<double, 3> angularVelocity = {0.0, 0.0, 0.0};
    std::array<double, 2> psi = {0.0, 0.0};
    
    //moment arm
    std::array<double, 3> r = {0.0, 0.0, centerOfGravity-distanceToThrustVector};

    // Main game loop
    while (!WindowShouldClose()){

        if(!landed){
            //update loop
            for(int i = 0; i < iterationsPerFrame; i++){
                t += dt;

                //compute forces
                thrustRf = forceThrustRf(gimbalAngleX*DEG2RAD, gimbalAngleY*DEG2RAD, t, magnitudeThrustVector);
                thrustWf = rotateRfToWf(stateQ, thrustRf); 

                //sum forces
                sumOfForcesWf = {thrustWf[0], thrustWf[1], thrustWf[2]-mass*gravity}; 

                //compute accleration
                accleration[0] = (sumOfForcesWf[0] / mass);
                accleration[1] = (sumOfForcesWf[1] / mass);
                accleration[2] = (sumOfForcesWf[2] / mass);

                //integrate accleration for velocity
                velocity[0] += dt*accleration[0];
                velocity[1] += dt*accleration[1];
                velocity[2] += dt*accleration[2];

                //integrate velocity for position 
                position[0] += dt*velocity[0];
                position[1] += dt*velocity[1];
                position[2] += dt*velocity[2]; 


                //compute torques   
                torqueThrust = crossProduct(r, thrustRf);


                //compute angular accleration
                angularAccleration[0] = (torqueThrust[0] / Ixx);
                angularAccleration[1] = (torqueThrust[1] / Iyy);

                //integrate angular accleration for angular velocity 
                angularVelocity[0] += dt*angularAccleration[0];
                angularVelocity[1] += dt*angularAccleration[1];

                //convert angular velocity to pure quaternion
                angularVelocityQ = vectorToPureQuaternion(angularVelocity);

                //compute first derivative of quaternion
                stateQTimeDerivative = multiplyQP(stateQ, angularVelocityQ);
                stateQTimeDerivative[0] = stateQTimeDerivative[0]*0.5;
                stateQTimeDerivative[1] = stateQTimeDerivative[1]*0.5;
                stateQTimeDerivative[2] = stateQTimeDerivative[2]*0.5;
                stateQTimeDerivative[3] = stateQTimeDerivative[3]*0.5;

                //integrate time derivative of q to update state quaternion
                stateQ[0] += dt*stateQTimeDerivative[0];
                stateQ[1] += dt*stateQTimeDerivative[1];
                stateQ[2] += dt*stateQTimeDerivative[2];
                stateQ[3] += dt*stateQTimeDerivative[3];

                //normalize
                normalizeQuaternion(stateQ);

                //compute euler angles
                psi = quaternionToEuler(stateQ); 

                if(position[2] < 0 && t > 0.5){
                    landed = true; 
                    break;
                }
            }
        }

        //rotation matrix creation
        float rf[16];
        quatToMat(stateQ, rf);

        //Apply Update
        UpdateCamera(&camera, CAMERA_FREE);

        //focus on vectorisZ
        if (IsKeyPressed(KEY_Z)){
            camera.target = (Vector3){position[0], position[2], -position[1]};
        }
        

        BeginDrawing();
            ClearBackground(RAYWHITE);
            BeginMode3D(camera);

                rlPushMatrix();

                    //transform into raylibs coords and move into rocket frame to then get rotation
                    rlMultMatrixf(cf);                                   
                    rlTranslatef(position[0], position[1], position[2]); 
                    rlMultMatrixf(rf);                                   

                    //rotate rocket to be facing correct way
                    rlRotatef(90.0f, 1.0f, 0.0f, 0.0f);                

                    //shift cyclinder down, such that c.g is point rotate about. i.e make it the origin
                    rlTranslatef(0.0f, -cgFrac * bodyHeight, 0.0f);
                    
                    //main rocket body
                    DrawCylinder((Vector3){0,0,0}, bodyRadius, bodyRadius, bodyHeight, 64, BLUE);

                    //nose cone
                    rlPushMatrix();
                        rlTranslatef(0.0f, bodyHeight, 0.0f);
                        DrawCylinder((Vector3){0,0,0}, coneTopRadius, bodyRadius, coneHeight, 64, RED);
                    rlPopMatrix();

                rlPopMatrix();

                DrawGrid(100, 1.0f);
            
            EndMode3D();
            
            DrawRectangle(10, 10, 200, 90, Fade(SKYBLUE, 0.5f));
            DrawRectangleLines(10, 10, 200, 90, BLUE);
            DrawText("Free camera default controls:", 20, 20, 10, BLACK);
            DrawText("- Mouse Wheel to Zoom in&out", 40, 40, 10, DARKGRAY);
            DrawText("- WASD to move around", 40, 60, 10, DARKGRAY);
            DrawText("- Press Z to focus on rocket", 40, 80, 10, DARKGRAY);
        EndDrawing();
    }

    CloseWindow();    

    //print
    std::cout << "Final position: (" << position[0] << "x, " << position[1] << "y, " << position[2] << "z)" << std::endl;

    return 0;
}


std::array<double, 3> forceThrustRf(double gimbalAngleX, double gimbalAngleY, double t, double magnitudeThrustVector){

    if(t > 3){
        std::array<double, 3> forceThrustVectorRf = {0.0, 0.0, 0.0}; 
        return forceThrustVectorRf;
    }
    
    else {
        std::array<double, 3> forceThrustVectorRf = {magnitudeThrustVector*std::sin(gimbalAngleX), magnitudeThrustVector*std::sin(gimbalAngleY), magnitudeThrustVector*std::cos(gimbalAngleX)*std::cos(gimbalAngleY)};   
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
std::array<double, 3> crossProduct(const std::array<double, 3>& a, const std::array<double, 3>& b){
    std::array<double, 3> aCrossb = {a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]};

    return aCrossb;
}
std::array<double, 2> quaternionToEuler(const std::array<double, 4>& stateQuaternion){

    std::array<double, 2> angles = {std::atan2(2*(stateQuaternion[0]*stateQuaternion[1]+stateQuaternion[2]*stateQuaternion[3]),
         1-2*(stateQuaternion[1]*stateQuaternion[1]+stateQuaternion[2]*stateQuaternion[2])), 
         std::asin(2*(stateQuaternion[0]*stateQuaternion[2] - stateQuaternion[1]*stateQuaternion[3]))};

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