#include "../include/raylib.h"
#include "../include/common.h" // Includes raymath.h indirectly
#include <stdio.h>  // For debug prints
#include <stdlib.h> // For malloc, free

// Prototypes for functions in objects.c
// GameObject related
GameObject* createRectangleObject(Vector2 position, Vector2 velocity, float width, float height, Color color, bool isStatic);
GameObject* createDiamondObject(Vector2 position, Vector2 velocity, float diagWidth, float diagHeight, Color color, bool isStatic);
GameObject* createArcCircleObject(Vector2 position, Vector2 velocity, float radius, float startAngle, float endAngle, float thickness, Color color, bool isStatic, float rotationSpeed, bool removeEscapedBalls);
void addObjectToList(GameObject** head, GameObject* newObject);
void freeObjectList(GameObject** head);
void updateObjectList(GameObject* head, float dt);
void renderObjectList(GameObject* head);
GameObject* createGameObjectWithEffects(GameObject* baseObject, CollisionEffect* effectsList);
void addCollisionEffectsToGameObject(GameObject* obj, CollisionEffect* effectsList);
void addCollisionEffectsToBouncingObject(BouncingObject* obj, CollisionEffect* effectsList);
int Count_BouncingObjects(BouncingObject* head);
int Count_GameObjects(GameObject* head);

// Collision effects
CollisionEffect* createColorChangeEffect(Color newColor, bool continuous);
CollisionEffect* createVelocityBoostEffect(float factor, bool continuous);
CollisionEffect* createVelocityDampenEffect(float factor, bool continuous);
CollisionEffect* createSizeChangeEffect(float factor, bool continuous);
CollisionEffect* createSoundPlayEffect(Sound sound, bool continuous);
void addEffectToList(CollisionEffect** head, CollisionEffect* newEffect);
void applyEffects(BouncingObject* bouncingObj, GameObject* gameObj, bool isOngoingCollision);

// Handle collisions between bouncing objects
void handleBallToBallCollisions(BouncingObject* bouncingObjectList, float dt);

// Screen boundary collision for a bouncing object
static void applyScreenBoundaryCollisions(BouncingObject* obj) {
    bool reflected = false;
    
    if (obj->position.x - obj->radius < 0) {
        obj->position.x = obj->radius + EPSILON2; // Push out
        if (obj->velocity.x < 0) obj->velocity.x *= -1; // Reflect
        reflected = true;
    } else if (obj->position.x + obj->radius > SCREEN_WIDTH) {
        obj->position.x = SCREEN_WIDTH - obj->radius - EPSILON2; // Push out
        if (obj->velocity.x > 0) obj->velocity.x *= -1; // Reflect
        reflected = true;
    }
    
    if (obj->position.y - obj->radius < 0) {
        obj->position.y = obj->radius + EPSILON2; // Push out
        if (obj->velocity.y < 0) obj->velocity.y *= -1; // Reflect
        reflected = true;
    } else if (obj->position.y + obj->radius > SCREEN_HEIGHT) {
        obj->position.y = SCREEN_HEIGHT - obj->radius - EPSILON2; // Push out
        if (obj->velocity.y > 0) obj->velocity.y *= -1; // Reflect
        reflected = true;
    }
    
    if (reflected) { // Apply slight damping on wall hit
        obj->velocity = Vector2Scale(obj->velocity, 0.99f);
    }
}

// Handle collisions between bouncing objects
void handleBallToBallCollisions(BouncingObject* bouncingObjectList, float dt) {
    // For each pair of balls, check for collisions
    for (BouncingObject* ball1 = bouncingObjectList; ball1 != NULL; ball1 = ball1->next) {
        // Skip if this ball shouldn't interact with other bouncing objects
        if (!ball1->interactWithOtherBouncingObjects) continue;
        
        for (BouncingObject* ball2 = ball1->next; ball2 != NULL; ball2 = ball2->next) {
            // Skip if the second ball shouldn't interact with other bouncing objects
            if (!ball2->interactWithOtherBouncingObjects) continue;
            
            // Calculate distance between centers
            float distance = Vector2Distance(ball1->position, ball2->position);
            float minDistance = ball1->radius + ball2->radius;
            
            // Check for collision (overlap)
            if (distance < minDistance) {
                // Calculate normal vector from ball1 to ball2
                Vector2 normal = Vector2Normalize(Vector2Subtract(ball2->position, ball1->position));
                
                // Calculate overlap amount
                float overlap = minDistance - distance;
                
                // Separate the balls to avoid persistent collision
                // Distribute movement based on masses (heavier ball moves less)
                float totalMass = ball1->mass + ball2->mass;
                float ball1Ratio = ball2->mass / totalMass;
                float ball2Ratio = ball1->mass / totalMass;
                
                // Push balls apart
                ball1->position = Vector2Subtract(ball1->position, Vector2Scale(normal, overlap * ball1Ratio));
                ball2->position = Vector2Add(ball2->position, Vector2Scale(normal, overlap * ball2Ratio));
                
                // Collision response (elastic collision formula)
                // Calculate relative velocity
                Vector2 relativeVelocity = Vector2Subtract(ball1->velocity, ball2->velocity);
                
                // Calculate impulse strength
                float impulseMagnitude = (-(1 + ball1->restitution * ball2->restitution) * 
                                         Vector2DotProduct(relativeVelocity, normal)) / 
                                         (1/ball1->mass + 1/ball2->mass);
                
                // Apply impulse to velocities
                ball1->velocity = Vector2Add(ball1->velocity, 
                                           Vector2Scale(normal, impulseMagnitude / ball1->mass));
                                           
                ball2->velocity = Vector2Subtract(ball2->velocity, 
                                                Vector2Scale(normal, impulseMagnitude / ball2->mass));
            }
        }
    }
}

// Find and handle all collisions for a single bouncing object with all game objects
// Returns the number of collisions handled
int handleBouncingObjectCollisions(BouncingObject* bouncingObj, GameObject* objectList, float dt, int maxSubsteps) {
    float remainingTimeThisFrame = dt;
    int substeps = 0;
    
    // Check for initial overlap with any object and resolve it before starting simulation
    for (GameObject* obj = objectList; obj != NULL; obj = obj->next) {
        float dummy_toi;
        Vector2 normal;
        // If already colliding (collision with time=0), push the bouncing object out
        if (obj->checkCollision(obj, bouncingObj, EPSILON2, &dummy_toi, &normal) && dummy_toi < EPSILON2) {
            if (Vector2LengthSqr(normal) > EPSILON2) {
                // Push bouncing object out along collision normal to resolve overlap
                bouncingObj->position = Vector2Add(bouncingObj->position, 
                                                  Vector2Scale(normal, bouncingObj->radius * 0.1f));
            }
        }
    }
    
    while (remainingTimeThisFrame > EPSILON2 && substeps < maxSubsteps) {
        float timeToFirstCollision = remainingTimeThisFrame; // Assume no collision initially
        GameObject* firstCollidingObject = NULL;
        Vector2 firstCollisionNormal = {0,0};
        
        // 1. Find the earliest collision time with any object
        for (GameObject* obj = objectList; obj != NULL; obj = obj->next) {
            float toi_candidate;
            Vector2 normal_candidate;
            
            // Check collision for the current remaining time slice
            if (obj->checkCollision(obj, bouncingObj, remainingTimeThisFrame, 
                                   &toi_candidate, &normal_candidate)) {
                // Ensure toi_candidate is valid and the earliest
                if (toi_candidate >= -EPSILON2 && toi_candidate < timeToFirstCollision) {
                    timeToFirstCollision = toi_candidate;
                    firstCollidingObject = obj;
                    firstCollisionNormal = normal_candidate;
                }
            }
        }
        
        // Ensure non-negative time step
        timeToFirstCollision = fmaxf(0.0f, timeToFirstCollision);

        // 2. Advance bouncing object by timeToFirstCollision
        bouncingObj->position = Vector2Add(bouncingObj->position, 
                                          Vector2Scale(bouncingObj->velocity, timeToFirstCollision));
        
        // 3. Update the objects (they move independently)
        //updateObjectList(objectList, timeToFirstCollision);
        
        // 4. Reduce remaining time for this frame
        remainingTimeThisFrame -= timeToFirstCollision;
        
        // 5. If a collision occurred, resolve it
        if (firstCollidingObject != NULL) {
            bool isValidNormal = (Vector2LengthSqr(firstCollisionNormal) > EPSILON2);
            
            // Collision response - velocity reflection with restitution
            if (isValidNormal) {
                // Calculate reflected velocity with restitution factor
                bouncingObj->velocity = Vector2Scale(
                    Vector2Reflect(bouncingObj->velocity, firstCollisionNormal), 
                    bouncingObj->restitution
                );
                
                // Avoid precision issues by nudging away from collision surface
                bouncingObj->position = Vector2Add(
                    bouncingObj->position, 
                    Vector2Scale(firstCollisionNormal, bouncingObj->radius * 0.05f)
                );
            } else {
                // Fallback for invalid normal - push away from object center
                Vector2 pushDir = Vector2Normalize(
                    Vector2Subtract(bouncingObj->position, firstCollidingObject->position)
                );
                
                if (Vector2LengthSqr(pushDir) > EPSILON2) {
                    // Push away from object
                    bouncingObj->position = Vector2Add(
                        bouncingObj->position, 
                        Vector2Scale(pushDir, bouncingObj->radius * 0.1f)
                    );
                    
                    // Simple reflection based on direction to object center
                    bouncingObj->velocity = Vector2Scale(
                        Vector2Reflect(bouncingObj->velocity, pushDir), 
                        bouncingObj->restitution
                    );
                }
            }
            
            // Apply any collision effects (on initial collision only)
            applyEffects(bouncingObj, firstCollidingObject, false);
        }
        
        substeps++;
    }
    
    return substeps;
}

#if 0
#define SONG 1
#else
#define SONG 0
#endif

int main(void) {
    // Initialize window and set target FPS
    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Multi-Object Physics Simulation");
    SetTargetFPS(120);
    
    // Speed controller setup
    const float speedValues[] = {
        0.00f, 0.01f, 0.02f, 0.05f, 0.10f, 0.20f, 0.30f, 0.40f, 0.50f, 0.60f, 
        0.70f, 0.80f, 0.90f, 1.00f, 1.10f, 1.20f, 1.30f, 1.40f, 1.50f, 
        1.60f, 1.70f, 1.80f, 1.90f, 2.00f, 2.20f, 2.40f, 2.60f, 2.80f, 
        3.00f, 4.00f, 5.00f, 6.00f, 7.00f, 8.00f, 9.00f, 10.00f
    };
    const int numSpeedValues = sizeof(speedValues) / sizeof(speedValues[0]);
    int currentSpeedIndex = 13; // Default to 1.00 (index 13)
    float timeMultiplier = speedValues[currentSpeedIndex];
    
    // Speed controller UI elements
    Rectangle decreaseButton = { SCREEN_WIDTH/2 - 100, SCREEN_HEIGHT - 40, 30, 30 };
    Rectangle increaseButton = { SCREEN_WIDTH/2 + 70, SCREEN_HEIGHT - 40, 30, 30 };
    Rectangle speedDisplay = { SCREEN_WIDTH/2 - 65, SCREEN_HEIGHT - 40, 130, 30 };
    
    // Initialize audio device for sound effects
    #if SONG
    InitAudioDevice();
    // Create some sounds for collision effects
    Sound bounceSound = LoadSound("resources/bounce.wav"); // Ensure this file exists or handle failure
    #endif
    // --- Create the static and moving objects ---
    GameObject* staticObjectList = NULL; // Objects that don't bounce but can be collided with
    BouncingObject* bouncingObjectList = NULL; // Objects that bounce around
    
    // --- Create static objects with different effects ---
    
    // Create a rotating arc circle that removes balls when they escape through it
    GameObject* arcCircle = createArcCircleObject(
        (Vector2){ SCREEN_WIDTH * 0.5f, SCREEN_HEIGHT * 0.5f }, // Center position
        (Vector2){ 0, 0 }, // No movement
        100.0f,            // Radius
        60.0f,              // Start angle
        120.0f,            // End angle (120 degrees arc)
        20.0f,             // Thickness
        (Color){ 230, 41, 55, 255 }, // Red
        true,              // Static
        30.0f,             // Rotation speed (degrees per second)
        true               // Remove balls that escape through the arc
    );
    
    // Add the arc circle to the static objects list
    addObjectToList(&staticObjectList, arcCircle);
    
    // Main game loop
    while (!WindowShouldClose()) {        // Get the elapsed time for this frame
        float dt = GetFrameTime() * timeMultiplier;  // Apply time multiplier to control simulation speed
        
        // Update all static objects (especially important for rotating objects like arcCircle)
        updateObjectList(staticObjectList, dt);
        
        // Handle speed controller buttons
        Vector2 mousePoint = GetMousePosition();
        
        // Check decrease button
        if ((CheckCollisionPointRec(mousePoint, decreaseButton) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) || IsKeyPressed(KEY_LEFT)) {
            if (currentSpeedIndex > 0) {
                currentSpeedIndex--;
                timeMultiplier = speedValues[currentSpeedIndex];
            }
        }
        
        // Check increase button
        if ((CheckCollisionPointRec(mousePoint, increaseButton) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) || IsKeyPressed(KEY_RIGHT)) {
            if (currentSpeedIndex < numSpeedValues - 1) {
                currentSpeedIndex++;
                timeMultiplier = speedValues[currentSpeedIndex];
            }
        }
        
        // Handle keyboard input - add new bouncing objects with mouse click
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) || IsKeyDown(KEY_SPACE)) {
            // Don't create a ball if clicking on speed controls
            if (!CheckCollisionPointRec(mousePoint, decreaseButton) && 
                !CheckCollisionPointRec(mousePoint, increaseButton) &&
                !CheckCollisionPointRec(mousePoint, speedDisplay)){
                int repetition = 1;
                    if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)){
                        repetition = 25;
                }
                Vector2 mousePos = GetMousePosition();
                do {
                    Vector2 randomVelocity = {
                        (float)(100 + rand() % 200) * (rand() % 2 == 0 ? 1 : -1),
                        (float)(100 + rand() % 200) * (rand() % 2 == 0 ? 1 : -1)
                    };                BouncingObject* newBall = createBouncingObject(
                        mousePos, 
                        randomVelocity, 
                        10 + (rand() % 20), // Random size
                        (Color){ rand() % 200 + 55, rand() % 200 + 55, rand() % 200 + 55, 255 }, // Random color
                        0.5f + ((float)rand() / RAND_MAX) * 2.5f, // Random mass between 0.5 and 3.0
                        //0.6f + ((float)rand() / RAND_MAX) * 0.35f, // Random restitution between 0.6 and 0.95
                        1.0f, // Restitution (bounciness)
                        true // By default, allow interaction with other bouncing objects
                    );
                    addBouncingObjectToList(&bouncingObjectList, newBall);
                    repetition--;
                } while (repetition > 0);
            }
        }
        
        // Process physics for all bouncing objects
        for (BouncingObject* ball = bouncingObjectList; ball != NULL; ball = ball->next) {
            // Handle collisions with all static and moving non-bouncing objects
            handleBouncingObjectCollisions(ball, staticObjectList, dt, 10);
            
            // Apply simple screen boundary collisions
            applyScreenBoundaryCollisions(ball);
        }
        
        // Handle collisions between bouncing objects
        handleBallToBallCollisions(bouncingObjectList, dt);
        
        // Remove any balls marked for deletion (e.g. those that have escaped through arcs)
        removeMarkedBouncingObjects(&bouncingObjectList);
        
        // Begin drawing
        BeginDrawing();
        ClearBackground(DARKGRAY);
        
        // Render all objects
        renderObjectList(staticObjectList);
        renderBouncingObjectList(bouncingObjectList);
          // Display instructions
        DrawText("Left click: Add new random bouncing ball", 10, 10, 20, WHITE);
        DrawText("Right click + Left click: Add 50 balls at once", 10, 40, 20, WHITE);
        DrawText("ESC: Quit", 10, 70, 20, WHITE);
        
        // Display FPS
        DrawFPS(SCREEN_WIDTH - 100, 10);
        
        // Displays the number of bouncing objects
        int bouncingCount = Count_BouncingObjects(bouncingObjectList);
        DrawText(TextFormat("Bouncing Objects: %d", bouncingCount), 10, 70, 20, WHITE);
        // Display the number of static objects
        int staticCount = Count_GameObjects(staticObjectList);
        DrawText(TextFormat("Static Objects: %d", staticCount), 10, 100, 20, WHITE);

        // Render speed controller UI
        DrawRectangleRec(decreaseButton, LIGHTGRAY);
        DrawRectangleRec(increaseButton, LIGHTGRAY);
        DrawRectangleRec(speedDisplay, GRAY);
        DrawText("<", decreaseButton.x + 10, decreaseButton.y + 5, 20, BLACK);
        DrawText(">", increaseButton.x + 10, increaseButton.y + 5, 20, BLACK);
        DrawText(TextFormat("x%.2f", timeMultiplier), speedDisplay.x + 10, speedDisplay.y + 5, 20, WHITE);

        EndDrawing();
    }
    
    // Cleanup
    #if SONG
    UnloadSound(bounceSound);
    CloseAudioDevice();
    #endif
    
    freeObjectList(&staticObjectList);
    freeBouncingObjectList(&bouncingObjectList);
    
    CloseWindow();
    return 0;
}