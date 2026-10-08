/// Entry point for the 2D drone flight simulator.
/// Owns the raylib window and drives the input -> physics -> render loop.

#include "raylib.h"

#include "Drone.h"

namespace {

constexpr int kScreenWidth = 800;
constexpr int kScreenHeight = 600;
constexpr int kTargetFps = 60;
constexpr int kGroundStripHeightPx = 10;
constexpr const char* kWindowTitle = "2D Drone Flight Simulator";

}  // namespace

int main() {
    InitWindow(kScreenWidth, kScreenHeight, kWindowTitle);
    SetTargetFPS(kTargetFps);

    Drone drone(static_cast<float>(kScreenWidth) / 2.0f, static_cast<float>(kScreenHeight) / 2.0f);

    while (!WindowShouldClose()) {
        const float dt = GetFrameTime();

        // 1. Update simulation state.
        drone.HandleInput(dt);
        drone.UpdatePhysics(dt, kScreenWidth, kScreenHeight);

        // 2. Render the frame.
        BeginDrawing();
        ClearBackground(SKYBLUE);
        DrawRectangle(0, kScreenHeight - kGroundStripHeightPx, kScreenWidth, kGroundStripHeightPx, DARKGREEN);
        drone.Draw();
        EndDrawing();
    }

    CloseWindow();
    return 0;
}