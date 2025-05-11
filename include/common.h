#ifndef COMMON_H
#define COMMON_H

#include "../include/raylib.h"
#include "../include/raymath.h" // For Vector2 math functions
#include <stdbool.h>
#include <float.h>   // For FLT_MAX

#define SCREEN_WIDTH 1080
#define SCREEN_HEIGHT 720

#define EPSILON2 0.0001f

// Forward declarations
typedef struct Ball Ball;
typedef struct GameObject GameObject;
typedef struct BouncingObject BouncingObject;
typedef struct CollisionEffect CollisionEffect;

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

// --- Type of collision effect ---
typedef enum {
    EFFECT_COLOR_CHANGE,      // Change color of the bouncing object
    EFFECT_VELOCITY_BOOST,    // Increase speed of the bouncing object
    EFFECT_VELOCITY_DAMPEN,   // Decrease speed of the bouncing object
    EFFECT_SIZE_CHANGE,       // Change size of the bouncing object
    EFFECT_SOUND_PLAY         // Play a sound effect
} EffectType;

// --- Collision effect structure ---
struct CollisionEffect {
    EffectType type;
    bool continuous;         // If true, apply on every frame of contact; if false, only on initial bounce
    union {
        struct {
            Color color;      // New color for COLOR_CHANGE
        } colorEffect;
        struct {
            float factor;     // Multiplication factor for VELOCITY_BOOST or VELOCITY_DAMPEN
        } velocityEffect;
        struct {
            float factor;     // Multiplication factor for SIZE_CHANGE
        } sizeEffect;
        struct {
            Sound sound;      // Sound to play for SOUND_PLAY
        } soundEffect;
    } params;
    
    CollisionEffect* next;   // For linked list
};

// --- Ball structure ---
struct Ball {
    Vector2 position;
    Vector2 velocity;
    float radius;
    Color color;
};

// --- Bouncing Object structure ---
struct BouncingObject {
    Vector2 position;
    Vector2 velocity;
    float radius;          // All bouncing objects are circular for simplicity
    Color color;
    float mass;            // Mass affects collision response
    float restitution;     // Bounciness factor (0.0 to 1.0)
    bool interactWithOtherBouncingObjects;  // If true, will bounce against other bouncing objects
    
    // Linked list of effects to apply when this object collides
    CollisionEffect* onCollisionEffects;
    
    // Pointer to the next bouncing object in the linked list
    BouncingObject* next;
};

// --- Generic Game Object structure (now represents non-bouncing objects) ---
struct GameObject {
    ShapeType type;
    Vector2 position;    // Center of the shape
    Vector2 velocity;
    void* shapeData;     // Pointer to shape-specific data (e.g., ShapeDataRectangle)
    bool isStatic;       // If true, velocity is ignored, object doesn't move
    
    // Linked list of effects to apply to bouncing objects that collide with this object
    CollisionEffect* onCollisionEffects;

    // Function pointers for polymorphism
    void (*render)(GameObject* self);
    // dt_step: the maximum time interval for this collision check (e.g., remaining frame time)
    // timeOfImpact (OUT): calculated time until collision occurs within dt_step
    // collisionNormal (OUT): normal of the surface at the point of impact (pointing away from object surface)
    bool (*checkCollision)(GameObject* self, struct BouncingObject* bouncingObj, float dt_step, float* timeOfImpact, Vector2* collisionNormal);
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

// --- Function Prototypes for BouncingObject Management ---
BouncingObject* createBouncingObject(Vector2 position, Vector2 velocity, float radius, Color color, float mass, float restitution, bool interactWithOtherBouncingObjects);
void addBouncingObjectToList(BouncingObject** head, BouncingObject* newObject);
void freeBouncingObjectList(BouncingObject** head);
void updateBouncingObjectList(BouncingObject* head, float dt);
void renderBouncingObjectList(BouncingObject* head);

// --- Function Prototypes for Collision Effect Management ---
CollisionEffect* createColorChangeEffect(Color newColor, bool continuous);
CollisionEffect* createVelocityBoostEffect(float factor, bool continuous);
CollisionEffect* createVelocityDampenEffect(float factor, bool continuous);
CollisionEffect* createSizeChangeEffect(float factor, bool continuous);
CollisionEffect* createSoundPlayEffect(Sound sound, bool continuous);
void addEffectToList(CollisionEffect** head, CollisionEffect* newEffect);
void freeEffectList(CollisionEffect** head);
void applyEffects(BouncingObject* bouncingObj, GameObject* gameObj, bool isOngoingCollision);

#endif // COMMON_H