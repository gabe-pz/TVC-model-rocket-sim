#include <iostream>
#include <array>
#include <cmath> 
#include <raylib.h> 
#include <rlgl.h> 

#include "../include/windGeneration.h" 
#include "../include/rocketMath.h"
#include "../include/rocketProperties.h"


int main(void){     
    //*****ROCKET PROPERTIES*****
    const double centerOfPressure = 0.0877;
    const double aRef = 0.00456; 
    const double cD = 0.291;
    const int cNa = 2;

    double centerOfGravity = 0.405;
    const double distanceToThrustVector = 0.6477;

    long double Ixx = 0.0249899588;
    long double Iyy = 0.0249868814;

    //inital gimbal angles(in degrees)
    double gimbalAngleX = 0.0;
    double gimbalAngleY = 0.0; 

    
    //*****SIMULATION SETTINGS*****
    const double dt = 0.000001; 
    const int simTime = 15;
    bool landed = false;
    double currentAltitude = 0.0;
    double prevAltitude = 0.0;
    bool printed = false; 
    bool coastingOver = false;
    bool printedCoasted = false;

    //*****WORLD THINGS***** 
    const double gravity = 9.81; 
    const float rho = 1.187f;

    //****PID SETTINGS*****
    double errorX, errorY;
    double prevErrorX = 0.0;
    double prevErrorY = 0.0;

    double setPoint = 0.0; 
    
    //pid gains
    double kPy = 0.3;
    double kIy = 0.07; 
    double kDy = 0.1;

    double kPx = 0.3;
    double kIx = 0.07; 
    double kDx = 0.1;

    //pid terms 
    double pTermY = 0.0;
    double iTermY = 0.0;
    double dTermY = 0.0;

    double pTermX = 0.0;
    double iTermX = 0.0;
    double dTermX = 0.0;


    //*****WIND SETTINGS*****
    //wind generation constants
    unsigned int seed = 12345;
    int n = (int)(simTime * GEN_FREQ) + 2;
    double U         = 5.0;  //average wind velocity
    double intensity = 0.20; //turbulence intensity 
    double sigmaU    = intensity * U;
    
    //wind turbulence buffers
    std::vector<double> pinkU = generatePinkNoise(n, seed);
    std::vector<double> pinkV = generatePinkNoise(n, seed + 1);
    std::vector<double> pinkW = generatePinkNoise(n, seed + 2);

    //mean-wind heading in world frame
    double theta = 0.0;
    double ux = std::cos(theta);
    double uy = std::sin(theta);

    //*****RAYLIB INITALIZATION*****
    const int screenWidth  = 1800;
    const int screenHeight = 1200;
    InitWindow(screenWidth, screenHeight, "tvc-model-rocket-sim");
    
    //raylib camera 
    Camera3D camera = { 0 };
    camera.position   = (Vector3){ 10.0f, 10.0f, 10.0f };  // Camera position
    camera.target     = (Vector3){ 0.0f, 0.0f, 0.0f };     // Camera looking-at point
    camera.up         = (Vector3){ 0.0f, 1.0f, 0.0f };     // Camera up vector
    camera.fovy       = 45.0f;                             // Field-of-view Y
    camera.projection = CAMERA_PERSPECTIVE;
    DisableCursor();    

    //raylib syncing 
    int FPS = 60;
    SetTargetFPS(FPS); 
    int iterationsPerFrame = (1/(dt*FPS)); 
    double t = 0.0;

    //raylib rocket Dimensions
    float bodyRadius = 0.25f;
    float bodyHeight = 2.5f;
    float coneHeight = 0.7;
    float coneTopRadius = 0.05f; 
    float cgFrac = (distanceToThrustVector - centerOfGravity) / distanceToThrustVector;                             

    //coordinate transformation matrix between raylib vecs and my own vecs. 
    //Form for 4x4 matrix is: r1 u, r2 v, r3 w, r4 t, where u, v, and w is my x, y, and z hat vector mapped into raylibs coordinate system, and t is origin offset
    float cf[16] = {
        0,0,1,0,
        1,0,0,0,
        0,1,0,0,
        0,0,0,1
    };


    //*******STD ARRAY INITALIZATIONS*****
    //quaterion initaliztion
    std::array<double, 4> stateQ = {1.0, 0.0, 0.0, 0.0};
    std::array<double, 4> stateQTimeDerivative = {1.0, 0.0, 0.0, 0.0};
    std::array<double, 4> angularVelocityQ = {1.0, 0.0, 0.0, 0.0};

    //forces initalization
    std::array<double, 3> thrustRf = {0.0, 0.0, 0.0};
    std::array<double, 3> thrustWf = {0.0, 0.0, 0.0};
    std::array<double, 3> aerodynamicForcesRf = {0.0, 0.0, 0.0}; 
    std::array<double, 3> aerodynamicForceswf = {0.0, 0.0, 0.0}; 
    std::array<double, 3> sumOfForcesWf = {0.0, 0.0, 0.0}; 

    //torques initalization
    std::array<double, 3> torqueThrust = {0.0, 0.0, 0.0}; 
    std::array<double, 3> torqueAero = {0.0, 0.0, 0.0}; 
    
    //position and its derivatives initalization
    std::array<double, 3> accleration = {0.0, 0.0, 0.0};
    std::array<double, 3> velocity = {0.0, 0.0, 0.0};
    std::array<double, 3> position = {0.0, 0.0, 0.0}; 
    std::array<double, 3> relativeVelocityWf = {0.0, 0.0, 0.0};
    std::array<double, 3> relativeVelocityRf = {0.0, 0.0, 0.0};

    //rotation and its derivatives initalization. Note psi = (theta, phi)
    std::array<double, 3> angularAccleration = {0.0, 0.0, 0.0};
    std::array<double, 3> angularVelocity = {0.0, 0.0, 0.0};
    std::array<double, 2> psi = {0.0, 0.0};
    
    //moment arms
    std::array<double, 3> r = {0.0, 0.0, centerOfGravity-distanceToThrustVector};
    std::array<double, 3> r_aero = {0.0, 0.0, centerOfGravity-centerOfPressure};

    //wind initalization
    std::array<double, 3> windVelocityWf = {0.0, 0.0, 0.0};

    //main physics loop
    while (!WindowShouldClose()){

        if(!landed){
            //state update
            for(int i = 0; i < iterationsPerFrame; i++){
                t += dt;
                
                //******PID CONTROL*****
                errorY = setPoint - RAD2DEG*psi[0]; 
                errorX = setPoint - RAD2DEG*psi[1];
                
                pTermY = kPy*errorY; 
                iTermY += dt*kIy*errorY;
                dTermY = kDy*((errorY - prevErrorY) / dt);

                pTermX = kPx*errorX; 
                iTermX += dt*kIx*errorX;
                dTermX = kDx*((errorX - prevErrorX) / dt);
                
                prevErrorY = errorY;
                prevErrorX = errorX;
                
                gimbalAngleY = clamp(pTermY+iTermY+dTermY, -5.0, 5.0);
                gimbalAngleX = clamp(-(pTermX+iTermX+dTermX), -5.0, 5.0);


                //*****WIND*****
                //sampling three independent turbulence streams at time t
                double u = windVelocity(t, U,   sigmaU,       pinkU);   
                double v = windVelocity(t, 0.0, 0.8 * sigmaU, pinkV);   
                double w = windVelocity(t, 0.0, 0.5 * sigmaU, pinkW);  

                //rotate wind frame -> world frame
                windVelocityWf = {u*ux - v*uy, u*uy + v*ux, w};
                
                //velocity of rocket wrt to wind in world frame, then into rocket frame
                relativeVelocityWf[0] = velocity[0] - windVelocityWf[0];
                relativeVelocityWf[1] = velocity[1] - windVelocityWf[1];
                relativeVelocityWf[2] = velocity[2] - windVelocityWf[2];
                relativeVelocityRf = rotateWfToRf(stateQ, relativeVelocityWf);
                
                //magnitude of relative v in RF
                double relativeVelMag = std::sqrt((relativeVelocityRf[0]*relativeVelocityRf[0]) + (relativeVelocityRf[1]*relativeVelocityRf[1]) + (relativeVelocityRf[2]*relativeVelocityRf[2]));
                
                //*****COMPUTE FORCES*****
                //force due to thrust
                thrustRf = forceThrustRf(gimbalAngleX*DEG2RAD, gimbalAngleY*DEG2RAD, t);
                thrustWf = rotateRfToWf(stateQ, thrustRf); 

                //aero forces
                aerodynamicForcesRf[0] = -0.5*rho*cNa*aRef*relativeVelocityRf[0]*relativeVelMag;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
                aerodynamicForcesRf[1] = -0.5*rho*cNa*aRef*relativeVelocityRf[1]*relativeVelMag;
                aerodynamicForcesRf[2] = -0.5*rho*cD*aRef*(std::abs(relativeVelocityRf[2]))*relativeVelocityRf[2]; 
                aerodynamicForceswf = rotateRfToWf(stateQ, aerodynamicForcesRf);
            
                //sum forces
                sumOfForcesWf = {thrustWf[0]+aerodynamicForceswf[0], thrustWf[1]+aerodynamicForceswf[1], thrustWf[2]-mass(t)*gravity+aerodynamicForceswf[2]}; 
                
                //****GET POSITION THROUGH ITS DERIVATIVES AND SUCH*****
                //compute accleration
                accleration[0] = (sumOfForcesWf[0] / mass(t));
                accleration[1] = (sumOfForcesWf[1] / mass(t));
                accleration[2] = (sumOfForcesWf[2] / mass(t));

                //integrate accleration for velocity
                velocity[0] += dt*accleration[0];
                velocity[1] += dt*accleration[1];
                velocity[2] += dt*accleration[2];

                //integrate velocity for position 
                position[0] += dt*velocity[0];
                position[1] += dt*velocity[1];
                position[2] += dt*velocity[2]; 

                //****GET ROATION THROUGH ITS DERIVATIVES*****
                //compute torques   
                torqueThrust = crossProduct(r, thrustRf);
                torqueAero = crossProduct(r_aero, aerodynamicForcesRf);

                //compute angular accleration
                angularAccleration[0] = ((torqueThrust[0] + torqueAero[0]) / Ixx);
                angularAccleration[1] = ((torqueThrust[1] + torqueAero[1]) / Iyy);

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

                //normalize quaterinon 
                normalizeQuaternion(stateQ);

                //get euler angles
                psi = quaternionToEuler(stateQ); 
                
                //*****LANDING CHECK*****
                if(position[2] < 0 && t > 0.5){
                    landed = true; 
                    break;
                }
                
                //******COASTING*****
                if(t > 3.45 && coastingOver != true && printedCoasted != true){
                    std::cout << "COASTING PHASE" << std::endl;
                    printedCoasted = true;
                }

                //*****APOGEE CHECK*****
                currentAltitude = position[2];
                if(currentAltitude < prevAltitude && printed != true){
                    std::cout << "APOGEE at t = " << t << "s" << std::endl; 
                    std::cout << "APOGEE ALTITUDE = " << position[2]*3.281 << "ft" << std::endl;
                    printed = true;
                    coastingOver = true;
                }
                prevAltitude = currentAltitude;
            
            }
        }

        //rotation matrix creation
        float rf[16];
        quatToMat(stateQ, rf);

        //Apply Update
        UpdateCamera(&camera, CAMERA_FREE);

        //focus on vectorisZ
        if (IsKeyPressed(KEY_Z)){
            //my x, i.e position[0] is mapped into raylibs z, and so on
            camera.target = (Vector3){(float)position[1], (float)position[2], (float)position[0]};
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

    //final print
    std::cout << "Final position: (" << position[0] << "x, " << position[1] << "y, " << position[2] << "z)" << std::endl;

    return 0;
}
