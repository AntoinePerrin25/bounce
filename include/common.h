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
    SHAPE_CIRCLE_ARC, // Arc/Open circle
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

/**
 * @brief Structure for arc circle shape data.
 * Represents a circular arc with a specific radius, angle, and thickness.
 * @param radius Radius of the arc circle.
 * @param startAngle Start angle in degrees.
 * @param endAngle End angle in degrees.
 * @param thickness Thickness of the arc.
 * @param color Color of the arc.
 * @param rotation Current rotation of the arc (for dynamic rotation).
 * @param rotationSpeed Speed of rotation (degrees per second).
 * @param removeEscapedBalls Whether balls that pass through the arc should be removed
 */

// Callback function type for collision/escape events with ArcCircle
typedef void (*ArcCircleCallback)(struct GameObject* arc, struct BouncingObject* ball);

// Callback list item for ArcCircle events
typedef struct ArcCircleCallbackNode {
    ArcCircleCallback callback;
    struct ArcCircleCallbackNode* next;
} ArcCircleCallbackNode;

typedef struct {
    float radius;         // Radius of the arc circle
    float startAngle;     // Start angle in degrees
    float endAngle;       // End angle in degrees
    float thickness;      // Thickness of the arc
    Color color;          // Color of the arc
    float rotation;       // Current rotation of the arc (for dynamic rotation)
    float rotationSpeed;  // Speed of rotation (degrees per second)
    bool removeEscapedBalls; // Whether balls that pass through the arc should be removed
    
    // Lists of callback functions
    ArcCircleCallbackNode* onCollisionCallbacks;  // Functions called when a ball collides with the arc
    ArcCircleCallbackNode* onEscapeCallbacks;     // Functions called when a ball escapes through the arc
} ShapeDataArcCircle;

// --- Type of collision effect ---
typedef enum {
    EFFECT_COLOR_CHANGE,      // Change color of the bouncing object
    EFFECT_VELOCITY_BOOST,    // Increase speed of the bouncing object
    EFFECT_VELOCITY_DAMPEN,   // Decrease speed of the bouncing object
    EFFECT_SIZE_CHANGE,       // Change size of the bouncing object
    EFFECT_SOUND_PLAY,        // Play a sound effect
    EFFECT_BALL_DISAPPEAR,    // Make the ball disappear with effects
    EFFECT_BALL_SPAWN         // Spawn a new ball with effects
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
        struct {
            int particleCount; // Number of particles to create when ball disappears
            Color particleColor; // Color of the particles
        } disappearEffect;
        struct {
            Vector2 position; // Position to spawn the new ball (if {0,0}, uses collision point)
            float radius;     // Radius of the new ball (0 for random)
            Color color;      // Color of the new ball (BLACK for random)
        } spawnEffect;
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
    bool markedForDeletion; // If true, this ball will be removed in the next frame
    
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
    bool markedForDeletion; // If true, this object will be removed in the next frame
    
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

// --- Function Prototypes for ArcCircle Callback Management ---
void addCollisionCallbackToArcCircle(GameObject* arcCircle, ArcCircleCallback callback);
void addEscapeCallbackToArcCircle(GameObject* arcCircle, ArcCircleCallback callback);
void freeArcCircleCallbackList(ArcCircleCallbackNode** head);

// --- Function Prototypes for GameObject Management ---
void removeMarkedGameObjects(GameObject** head);

// --- Function Prototypes for BouncingObject Management ---
BouncingObject* createBouncingObject(Vector2 position, Vector2 velocity, float radius, Color color, float mass, float restitution, bool interactWithOtherBouncingObjects);
void addBouncingObjectToList(BouncingObject** head, BouncingObject* newObject);
void freeBouncingObjectList(BouncingObject** head);
void updateBouncingObjectList(BouncingObject* head, float dt);
void renderBouncingObjectList(BouncingObject* head);
void removeMarkedBouncingObjects(BouncingObject** head); // New function to clean up marked objects

// --- Function Prototypes for Collision Effect Management ---
CollisionEffect* createColorChangeEffect(Color newColor, bool continuous);
CollisionEffect* createVelocityBoostEffect(float factor, bool continuous);
CollisionEffect* createVelocityDampenEffect(float factor, bool continuous);
CollisionEffect* createSizeChangeEffect(float factor, bool continuous);
CollisionEffect* createSoundPlayEffect(Sound sound, bool continuous);
CollisionEffect* createBallDisappearEffect(int particleCount, Color particleColor, bool continuous);
CollisionEffect* createBallSpawnEffect(Vector2 position, float radius, Color color, bool continuous);
void addEffectToList(CollisionEffect** head, CollisionEffect* newEffect);
void freeEffectList(CollisionEffect** head);
void applyEffects(BouncingObject* bouncingObj, GameObject* gameObj, bool isOngoingCollision);

#endif // COMMON_H