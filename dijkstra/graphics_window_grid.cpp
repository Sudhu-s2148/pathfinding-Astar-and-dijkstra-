#include "raylib.h"

int main()
{
    // Initialization
    const int screenWidth = 800;
    const int screenHeight = 450;
    InitWindow(screenWidth, screenHeight, "Raylib Line Animation Example");

    Vector2 startPoint = { 100, 200 };
    Vector2 endPoint = { 700, 200 }; // Target end-point
    float t = 0.0f;                  // Animation progress (0.0f to 1.0f)
    float speed = 1.0f;              // Speed of the line animation

    SetTargetFPS(100);

    // Main game loop
    while (!WindowShouldClose())
    {
        // Animate line progress
        if (t < 1.0f) t += speed * GetFrameTime();

        // Calculate current endpoint
        Vector2 currentPoint = {
            startPoint.x + t * (endPoint.x - startPoint.x),
            startPoint.y + t * (endPoint.y - startPoint.y)
        };

        BeginDrawing();
        ClearBackground(BLACK);

        DrawLineEx(startPoint, currentPoint,2.5f,GREEN);

        EndDrawing();
    }

    // De-Initialization
    CloseWindow();

    return 0;
}