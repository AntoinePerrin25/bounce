#ifndef COMMON_H
#define COMMON_H

#include "raylib/raylib.h"
#include "raylib/raymath.h" // For Vector2 math functions
#include <stdbool.h>
#include <float.h>   // For FLT_MAX

#define SCREEN_WIDTH 1280
#define SCREEN_HEIGHT 720

#define EPSILON2 0.0001f

// Forward declarations
typedef struct Ball Ball;
typedef struct GameObject GameObject;

// Shape types
typedef enum {
    SHAPE_RECTANGLE,
    SHAPE_DIAMOND,
    // SHAPE_CIRCLE_ARC, // Placeholder for "cercle non fermé"
} ShapeType;

// --- Shape-specific data structures ---
typedef struct {
    float width;
    float height;
    Color color;
} ShapeDataRectangle;

typedef struct {
    float halfWidth;  // Half of horizontal diagonal for diamond
    float halfHeight; // Half of vertical diagonal for diamond
    Color color;
} ShapeDataDiamond;

// --- Ball structure ---
struct Ball {
    Vector2 position;
    Vector2 velocity;
    float radius;
    Color color;
};

// --- Generic Game Object structure ---
struct GameObject {
    ShapeType type;
    Vector2 position;    // Center of the shape
    Vector2 velocity;
    void* shapeData;     // Pointer to shape-specific data (e.g., ShapeDataRectangle)
    bool isStatic;       // If true, velocity is ignored, object doesn't move

    // Function pointers for polymorphism
    void (*render)(GameObject* self);
    // dt_step: the maximum time interval for this collision check (e.g., remaining frame time)
    // timeOfImpact (OUT): calculated time until collision occurs within dt_step
    // collisionNormal (OUT): normal of the surface at the point of impact (pointing away from object surface)
    bool (*checkCollision)(GameObject* self, Ball* ball, float dt_step, float* timeOfImpact, Vector2* collisionNormal);
    void (*update)(GameObject* self, float dt); // For moving objects
    void (*destroy)(GameObject* self);          // To free shapeData and other resources

    GameObject* next; // For linked list
};

// --- Function Prototypes for Physics Helpers (implemented in objects.c or a dedicated physics.c) ---
bool sweptBallToStaticPointCollision(Vector2 point,
                                     Vector2 ballPos, Vector2 ballVel, float ballRadius,
                                     float dt_max, float* toi, Vector2* normal);

bool sweptBallToStaticSegmentCollision(Vector2 segP1, Vector2 segP2,
                                       Vector2 ballPos, Vector2 ballVel, float ballRadius,
                                       float dt_max, float* toi, Vector2* normal);

#endif // COMMON_H