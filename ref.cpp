#include "raylib.h"
#include "rlgl.h"

//------------------------------------------------------------------------------------
// Physics step (placeholder). Called once per dt.
//------------------------------------------------------------------------------------
void updateMyPhysics(float *x, float *z, float *y, float *rotX, float *rotY, float dt)
{
    // Replace with actual rocket kinematics.
    // NOTE: once these values grow past ~1.0, float grid spacing (1.19e-7) exceeds
    // 0.1f*dt = 1e-7, so each increment rounds high. Use double state for real physics.
    *z    += 0.1f * dt;
    *rotX += 0.1f * dt;
    *rotY += 0.2f * dt;
}

//------------------------------------------------------------------------------------
// Program main entry point
//------------------------------------------------------------------------------------
int main(void)
{
    // Initialization
    //--------------------------------------------------------------------------------
    const int screenWidth  = 800;
    const int screenHeight = 450;
    InitWindow(screenWidth, screenHeight, "tvc-model-rocket-sim");

    // Define the camera to look into our 3d world
    Camera3D camera = { 0 };
    camera.position   = (Vector3){ 10.0f, 10.0f, 10.0f };  // Camera position
    camera.target     = (Vector3){ 0.0f, 0.0f, 0.0f };     // Camera looking-at point
    camera.up         = (Vector3){ 0.0f, 1.0f, 0.0f };     // Camera up vector
    camera.fovy       = 45.0f;                             // Field-of-view Y
    camera.projection = CAMERA_PERSPECTIVE;

    DisableCursor();    // Limit cursor to relative movement inside the window

    // Rocket state: raylib (rx, ry, rz) -> physics (x, z, y)
    float myX = 0.0f, myY = 0.0f, myZ = 0.0f;
    float myRotX = 0.0f, myRotY = 0.0f;

    // Simulation clock --------------------------------------------------------------
    const int   STEPS_PER_SEC = 1000000;                    // 1 MHz physics rate
    const float dt            = 1.0f / (float)STEPS_PER_SEC; // 1 us per step
    const int   stepsPerFrame = STEPS_PER_SEC / 60;          // sim time per rendered frame
    const float SIM_END_TIME  = 15.0f;                       // total sim duration [s]
    const long  totalSteps    = (long)(SIM_END_TIME * STEPS_PER_SEC + 0.5f); // 15,000,000

    long stepCount   = 0;       // integer clock: exact, immune to float rounding
    bool simFinished = false;
    //--------------------------------------------------------------------------------

    // Main game loop
    while (!WindowShouldClose())    // Detect window close button or ESC key
    {
        // Update
        //----------------------------------------------------------------------------
        UpdateCamera(&camera, CAMERA_FREE);

        // Run this frame's chunk of physics, but never step past totalSteps.
        // The second condition trims the final chunk so we land on t = 15 s exactly.
        for (int i = 0; i < stepsPerFrame && stepCount < totalSteps; i++)
        {
            updateMyPhysics(&myX, &myZ, &myY, &myRotX, &myRotY, dt);
            stepCount++;
        }
        if (stepCount >= totalSteps) simFinished = true;

        // Reconstruct time from the step count: one multiply -> one rounding, no drift.
        double simTime = (double)stepCount * (double)dt;

        // Draw the resulting state once per frame
        //----------------------------------------------------------------------------
        BeginDrawing();

            ClearBackground(RAYWHITE);

            BeginMode3D(camera);

                rlPushMatrix();
                    rlTranslatef(myX, myZ, myY);
                    rlRotatef(myRotX, 1.0f, 0.0f, 0.0f);
                    rlRotatef(myRotY, 0.0f, 0.0f, 1.0f);
                    DrawCylinder((Vector3){ 0, 0, 0 }, 0.15f, 1.0f, 5.0f, 64, RED);
                rlPopMatrix();

                DrawGrid(100, 1.0f);

            EndMode3D();

            DrawRectangle(10, 10, 320, 133, Fade(SKYBLUE, 0.5f));
            DrawRectangleLines(10, 10, 320, 133, BLUE);
            DrawText("Free camera default controls:", 20, 20, 10, BLACK);
            DrawText("- Mouse Wheel to Zoom in-out", 40, 40, 10, DARKGRAY);
            DrawText("- Mouse Wheel Pressed to Pan", 40, 60, 10, DARKGRAY);
            DrawText("- Z to zoom to (0, 0, 0)", 40, 80, 10, DARKGRAY);
            DrawText(TextFormat("t = %.6f s / %.0f s", simTime, SIM_END_TIME), 20, 100, 10, BLACK);
            if (simFinished) DrawText("SIM COMPLETE - state frozen at t = 15 s", 20, 120, 10, MAROON);

        EndDrawing();
        //----------------------------------------------------------------------------
    }

    // De-Initialization
    //--------------------------------------------------------------------------------
    CloseWindow();    // Close window and OpenGL context
    //--------------------------------------------------------------------------------

    return 0;
}