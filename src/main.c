#include "raylib/raylib.h"
#include "../include/common.h" // Includes raymath.h indirectly
#include <stdio.h>   // For printf (debugging)


// Prototypes for functions in objects.c (if not covered by common.h for external use)
GameObject* createRectangleObject(Vector2 position, Vector2 velocity, float width, float height, Color color, bool isStatic);
GameObject* createDiamondObject(Vector2 position, Vector2 velocity, float diagWidth, float diagHeight, Color color, bool isStatic);
void addObjectToList(GameObject** head, GameObject* newObject);
void freeObjectList(GameObject** head);
void updateObjectList(GameObject* head, float dt);
void renderObjectList(GameObject* head);

// Simple screen boundary collision for the ball
static void applyScreenBoundaryCollisions(Ball* ball) {
    bool reflected = false;
    if (ball->position.x - ball->radius < 0) {
        ball->position.x = ball->radius + EPSILON2; // Push out
        if (ball->velocity.x < 0) ball->velocity.x *= -1; // Reflect
        reflected = true;
    } else if (ball->position.x + ball->radius > SCREEN_WIDTH) {
        ball->position.x = SCREEN_WIDTH - ball->radius - EPSILON2; // Push out
        if (ball->velocity.x > 0) ball->velocity.x *= -1; // Reflect
        reflected = true;
    }
    if (ball->position.y - ball->radius < 0) {
        ball->position.y = ball->radius + EPSILON2; // Push out
        if (ball->velocity.y < 0) ball->velocity.y *= -1; // Reflect
        reflected = true;
    } else if (ball->position.y + ball->radius > SCREEN_HEIGHT) {
        ball->position.y = SCREEN_HEIGHT - ball->radius - EPSILON2; // Push out
        if (ball->velocity.y > 0) ball->velocity.y *= -1; // Reflect
        reflected = true;
    }
    if (reflected) { // Apply slight damping on wall hit
        ball->velocity = Vector2Scale(ball->velocity, 0.99f);
    }
}

int main(void) {
    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Raylib Advanced Bouncing Ball");
    SetTargetFPS(60);

    Ball ball = {
        .position = { SCREEN_WIDTH / 5.0f, SCREEN_HEIGHT / 3.0f },
        .velocity = { 250.0f, 180.0f },
        .radius = 12.0f,
        .color = RED
    };

    GameObject* objectList = NULL;
    addObjectToList(&objectList, createRectangleObject((Vector2){SCREEN_WIDTH*0.7f, SCREEN_HEIGHT*0.5f}, (Vector2){0, 60.0f}, 120, 60, BLUE, false));
    addObjectToList(&objectList, createDiamondObject((Vector2){SCREEN_WIDTH*0.3f, SCREEN_HEIGHT*0.7f}, (Vector2){40, -30}, 100, 150, GREEN, false));
    addObjectToList(&objectList, createRectangleObject((Vector2){SCREEN_WIDTH*0.5f, SCREEN_HEIGHT*0.2f}, (Vector2){-50, 20}, 80, 80, PURPLE, false));
    addObjectToList(&objectList, createRectangleObject((Vector2){SCREEN_WIDTH*0.5f, SCREEN_HEIGHT - 10.0f}, (Vector2){0,0}, SCREEN_WIDTH*0.9f, 20, GRAY, true)); // Static Floor
    addObjectToList(&objectList, createRectangleObject((Vector2){10.0f, SCREEN_HEIGHT*0.5f}, (Vector2){0,0}, 20, SCREEN_HEIGHT*0.9f, GRAY, true)); // Static Left Wall


    unsigned int frameCounter = 0;

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();
        frameCounter++;

        // --- Physics Update with Sub-stepping for Multiple Collisions ---
        float remainingTimeThisFrame = dt;
        int substeps = 0;
        const int MAX_SUBSTEPS = 10000; // Safety break for too many collisions

        // Check for initial overlap with any object and resolve it before starting simulation
        for (GameObject* obj = objectList; obj != NULL; obj = obj->next) {
            float dummy_toi;
            Vector2 normal;
            // If already colliding (collision with time=0), push the ball out
            if (obj->checkCollision(obj, &ball, EPSILON2, &dummy_toi, &normal) && dummy_toi < EPSILON2) {
                if (Vector2LengthSqr(normal) > EPSILON2) {
                    // Push ball out along collision normal to resolve overlap
                    ball.position = Vector2Add(ball.position, Vector2Scale(normal, ball.radius * 0.1f));
                }
            }
        }

        while (remainingTimeThisFrame > EPSILON2 && substeps < MAX_SUBSTEPS) {
            float timeToFirstCollision = remainingTimeThisFrame; // Assume no collision initially
            GameObject* firstCollidingObject = NULL;
            Vector2 firstCollisionNormal = {0,0};

            // 1. Find the earliest collision time with any object
            for (GameObject* obj = objectList; obj != NULL; obj = obj->next) {
                float toi_candidate;
                Vector2 normal_candidate;
                
                // Check collision for the current remaining time slice
                if (obj->checkCollision(obj, &ball, remainingTimeThisFrame, &toi_candidate, &normal_candidate)) {
                    // Ensure toi_candidate is valid (non-negative and less than current earliest)
                    if (toi_candidate >= -EPSILON2 && toi_candidate < timeToFirstCollision) {
                        timeToFirstCollision = toi_candidate;
                        firstCollidingObject = obj;
                        firstCollisionNormal = normal_candidate;
                    }
                }
            }
            
            timeToFirstCollision = fmaxf(0.0f, timeToFirstCollision); // Ensure non-negative time step

            // 2. Advance ball and all objects by timeToFirstCollision
            ball.position = Vector2Add(ball.position, Vector2Scale(ball.velocity, timeToFirstCollision));
            // For objects, their update function already uses their own velocity.
            // The collision check used relative velocity, so we advance each independently.
            updateObjectList(objectList, timeToFirstCollision);


            // 3. Reduce remaining time for this frame
            remainingTimeThisFrame -= timeToFirstCollision;

            // 4. If a collision occurred (i.e., firstCollidingObject is not NULL)
            if (firstCollidingObject != NULL) {
                // Resolve ball's collision
                if (Vector2LengthSqr(firstCollisionNormal) > EPSILON2) {
                    ball.velocity = Vector2Reflect(ball.velocity, firstCollisionNormal);
                    // Apply a stronger separation to avoid immediate re-collision due to precision issues
                    // Increased from EPSILON2 * 10.0f to a more significant value
                    ball.position = Vector2Add(ball.position, Vector2Scale(firstCollisionNormal, ball.radius * 0.05f));
                    
                    // Apply a small damping on bounce to make physics more realistic
                    ball.velocity = Vector2Scale(ball.velocity, 0.98f);
                } else {
                    // Emergency fallback for invalid normal - push away from object center
                    Vector2 pushDir = Vector2Normalize(Vector2Subtract(ball.position, firstCollidingObject->position));
                    if (Vector2LengthSqr(pushDir) > EPSILON2) {
                        ball.position = Vector2Add(ball.position, Vector2Scale(pushDir, ball.radius * 0.1f));
                        // Simple reflection based on direction to object center
                        ball.velocity = Vector2Reflect(ball.velocity, pushDir);
                    }
                }
                
                // Note: Object's reaction to collision (e.g., bouncing off ball) is not implemented here for simplicity.
                // If objects were to react, their velocities would also change.
            }
            substeps++;
        }

        // Apply simple screen boundary collisions after all object interactions
        applyScreenBoundaryCollisions(&ball);

        // --- Drawing ---
        BeginDrawing();
        ClearBackground(DARKGRAY); // Changed background for better contrast

        renderObjectList(objectList);
        DrawCircleV(ball.position, ball.radius, ball.color);

        DrawText(TextFormat("Substeps: %d", substeps), 10, 10, 20, LIGHTGRAY);
        DrawFPS(SCREEN_WIDTH - 100, 10);
        EndDrawing();
    }

    freeObjectList(&objectList);
    CloseWindow();
    return 0;
}
