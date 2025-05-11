#include "raylib/raylib.h"
#include "../include/common.h" // Includes raymath.h indirectly
#include <stdio.h>  // For debug prints
#include <stdlib.h> // For malloc, free

// Prototypes for functions in objects.c
// GameObject related
GameObject* createRectangleObject(Vector2 position, Vector2 velocity, float width, float height, Color color, bool isStatic);
GameObject* createDiamondObject(Vector2 position, Vector2 velocity, float diagWidth, float diagHeight, Color color, bool isStatic);
void addObjectToList(GameObject** head, GameObject* newObject);
void freeObjectList(GameObject** head);
void updateObjectList(GameObject* head, float dt);
void renderObjectList(GameObject* head);
GameObject* createGameObjectWithEffects(GameObject* baseObject, CollisionEffect* effectsList);
void addCollisionEffectsToGameObject(GameObject* obj, CollisionEffect* effectsList);
void addCollisionEffectsToBouncingObject(BouncingObject* obj, CollisionEffect* effectsList);

// Collision effects
CollisionEffect* createColorChangeEffect(Color newColor, bool continuous);
CollisionEffect* createVelocityBoostEffect(float factor, bool continuous);
CollisionEffect* createVelocityDampenEffect(float factor, bool continuous);
CollisionEffect* createSizeChangeEffect(float factor, bool continuous);
CollisionEffect* createSoundPlayEffect(Sound sound, bool continuous);
void addEffectToList(CollisionEffect** head, CollisionEffect* newEffect);
void applyEffects(BouncingObject* bouncingObj, GameObject* gameObj, bool isOngoingCollision);

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
        updateObjectList(objectList, timeToFirstCollision);

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

#if 1
#define SONG 1
#else
#define SONG 0
#endif

int main(void) {
    // Initialize window and set target FPS
    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Multi-Object Physics Simulation");
    SetTargetFPS(60);
    
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
    
    // Rectangle that changes the color of bouncing objects to BLUE
    CollisionEffect* blueColorEffect = createColorChangeEffect(BLUE, false);
    GameObject* blueRect = createRectangleObject(
        (Vector2){SCREEN_WIDTH * 0.2f, SCREEN_HEIGHT - 50}, 
        (Vector2){0, 0}, 
        120, 20, 
        SKYBLUE, 
        true // Static
    );
    addCollisionEffectsToGameObject(blueRect, blueColorEffect);
    addObjectToList(&staticObjectList, blueRect);
    
    // Rectangle that boosts velocity
    CollisionEffect* speedBoostEffect = createVelocityBoostEffect(1.2f, false);
    GameObject* boostRect = createRectangleObject(
        (Vector2){SCREEN_WIDTH * 0.5f, SCREEN_HEIGHT - 50}, 
        (Vector2){0, 0}, 
        120, 20, 
        LIME, 
        true // Static
    );
    addCollisionEffectsToGameObject(boostRect, speedBoostEffect);
    addObjectToList(&staticObjectList, boostRect);
    
    // Rectangle that dampens velocity
    CollisionEffect* slowEffect = createVelocityDampenEffect(0.8f, false);
    GameObject* slowRect = createRectangleObject(
        (Vector2){SCREEN_WIDTH * 0.8f, SCREEN_HEIGHT - 50}, 
        (Vector2){0, 0}, 
        120, 20, 
        PINK, 
        true // Static
    );
    addCollisionEffectsToGameObject(slowRect, slowEffect);
    addObjectToList(&staticObjectList, slowRect);
    
    // Diamond with sound effect
    #if SONG
    CollisionEffect* soundEffect = createSoundPlayEffect(bounceSound, false);
    #endif
    GameObject* soundDiamond = createDiamondObject(
        (Vector2){SCREEN_WIDTH * 0.5f, SCREEN_HEIGHT * 0.5f}, 
        (Vector2){0, 0}, 
        100, 100, 
        YELLOW, 
        true // Static
    );
    #if SONG
    addCollisionEffectsToGameObject(soundDiamond, soundEffect);
    #endif
    addObjectToList(&staticObjectList, soundDiamond);
    
    // Rectangle that changes size
    CollisionEffect* growEffect = createSizeChangeEffect(1.1f, false);
    GameObject* growRect = createRectangleObject(
        (Vector2){SCREEN_WIDTH * 0.3f, SCREEN_HEIGHT * 0.3f}, 
        (Vector2){0, 30}, 
        80, 80, 
        ORANGE, 
        false // Moving
    );
    addCollisionEffectsToGameObject(growRect, growEffect);
    addObjectToList(&staticObjectList, growRect);

    
    // Create boundaries (walls)
    addObjectToList(&staticObjectList, createRectangleObject(
        (Vector2){SCREEN_WIDTH * 0.5f, 5}, 
        (Vector2){0, 0}, 
        SCREEN_WIDTH - 10, 10, 
        DARKGRAY, 
        true // Static
    )); // Top wall
    
    addObjectToList(&staticObjectList, createRectangleObject(
        (Vector2){5, SCREEN_HEIGHT * 0.5f}, 
        (Vector2){0, 0}, 
        10, SCREEN_HEIGHT - 10, 
        DARKGRAY, 
        true // Static
    )); // Left wall
    
    addObjectToList(&staticObjectList, createRectangleObject(
        (Vector2){SCREEN_WIDTH - 5, SCREEN_HEIGHT * 0.5f}, 
        (Vector2){0, 0}, 
        10, SCREEN_HEIGHT - 10, 
        DARKGRAY, 
        true // Static
    )); // Right wall
    
    // Create bouncing objects
    BouncingObject* ball1 = createBouncingObject(
        (Vector2){SCREEN_WIDTH * 0.3f, SCREEN_HEIGHT * 0.7f}, 
        (Vector2){220, -180}, 
        15, 
        RED, 
        1.0f, // Mass
        0.95f  // High restitution (bounciness)
    );
    addBouncingObjectToList(&bouncingObjectList, ball1);
    
    BouncingObject* ball2 = createBouncingObject(
        (Vector2){SCREEN_WIDTH * 0.7f, SCREEN_HEIGHT * 0.7f}, 
        (Vector2){-180, -150}, 
        20, 
        GREEN, 
        2.0f, // Heavier
        0.8f  // Medium restitution
    );
    addBouncingObjectToList(&bouncingObjectList, ball2);
    
    BouncingObject* ball3 = createBouncingObject(
        (Vector2){SCREEN_WIDTH * 0.5f, SCREEN_HEIGHT * 0.5f}, 
        (Vector2){120, 120}, 
        25, 
        GOLD, 
        3.0f, // Heavy
        0.6f  // Low restitution
    );
    addBouncingObjectToList(&bouncingObjectList, ball3);
    
    // Main game loop
    while (!WindowShouldClose()) {
        // Get the elapsed time for this frame
        float dt = GetFrameTime();
        
        // Handle keyboard input - add new bouncing objects with mouse click
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            Vector2 mousePos = GetMousePosition();
            Vector2 randomVelocity = {
                (float)(100 + rand() % 200) * (rand() % 2 == 0 ? 1 : -1),
                (float)(100 + rand() % 200) * (rand() % 2 == 0 ? 1 : -1)
            };
            
            BouncingObject* newBall = createBouncingObject(
                mousePos, 
                randomVelocity, 
                10 + (rand() % 20), // Random size
                (Color){ rand() % 200 + 55, rand() % 200 + 55, rand() % 200 + 55, 255 }, // Random color
                0.5f + ((float)rand() / RAND_MAX) * 2.5f, // Random mass between 0.5 and 3.0
                0.6f + ((float)rand() / RAND_MAX) * 0.35f // Random restitution between 0.6 and 0.95
            );
            addBouncingObjectToList(&bouncingObjectList, newBall);
        }
        
        // Process physics for all bouncing objects
        for (BouncingObject* ball = bouncingObjectList; ball != NULL; ball = ball->next) {
            // Handle collisions with all static and moving non-bouncing objects
            handleBouncingObjectCollisions(ball, staticObjectList, dt, 10);
            
            // Apply simple screen boundary collisions
            applyScreenBoundaryCollisions(ball);
        }
        
        // Begin drawing
        BeginDrawing();
        ClearBackground(DARKGRAY);
        
        // Render all objects
        renderObjectList(staticObjectList);
        renderBouncingObjectList(bouncingObjectList);
        
        // Display instructions
        DrawText("Left click: Add new random bouncing ball", 10, 10, 20, WHITE);
        DrawText("ESC: Quit", 10, 40, 20, WHITE);
        
        // Display FPS
        DrawFPS(SCREEN_WIDTH - 100, 10);
        
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