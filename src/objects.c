#include "../include/common.h"
#include <stdlib.h> // For malloc, free
#include <stdio.h>  // For debug prints (optional)
#include <math.h>   // For sqrtf, fabsf, fmaxf

// --- Physics Helper Implementations ---

// Closest point on segment AB to point P
static Vector2 closestPointOnSegment(Vector2 p, Vector2 a, Vector2 b) __attribute__((unused));
static Vector2 closestPointOnSegment(Vector2 p, Vector2 a, Vector2 b) {
    Vector2 ap = Vector2Subtract(p, a);
    Vector2 ab = Vector2Subtract(b, a);
    float ab2 = Vector2DotProduct(ab, ab);
    if (ab2 < EPSILON2) return a; // a and b are the same point or very close
    float t = Vector2DotProduct(ap, ab) / ab2;
    t = Clamp(t, 0.0f, 1.0f); // Clamp t to be between 0 and 1
    return Vector2Add(a, Vector2Scale(ab, t));
}

// Swept collision: ball moving towards a static point (vertex)
// Solves for t: | (ballPos + ballVel*t) - point |^2 = ballRadius^2
bool sweptBallToStaticPointCollision(Vector2 point,
                                     Vector2 ballPos, Vector2 ballVel, float ballRadius,
                                     float dt_max, float* toi, Vector2* normal) {
    Vector2 relPos = Vector2Subtract(ballPos, point); // Vector from point to ball center

    // Quadratic equation: a*t^2 + b*t + c = 0
    float a = Vector2DotProduct(ballVel, ballVel);
    float b = 2.0f * Vector2DotProduct(relPos, ballVel);
    float c = Vector2DotProduct(relPos, relPos) - ballRadius * ballRadius;

    if (fabsf(a) < EPSILON2) { // Velocity is zero or very small (linear equation c + bt = 0)
        if (c <= 0) { // Already overlapping or touching (or moving away if b > 0)
            if (b < 0 || fabsf(b) < EPSILON2) { // Moving towards or stationary and overlapping
                 *toi = 0.0f;
                Vector2 normDir = Vector2Normalize(relPos);
                if (Vector2LengthSqr(normDir) < EPSILON2) { // Ball center is at the point
                    normDir = Vector2Normalize(Vector2Negate(ballVel)); // Use opposite of velocity
                     if(Vector2LengthSqr(normDir) < EPSILON2) normDir = (Vector2){0, -1}; // Default
                }
                *normal = normDir; // Normal from point to ball
                return true;
            }
        }
        return false;
    }

    float discriminant = b * b - 4.0f * a * c;
    if (discriminant < 0) return false; // No real roots, no collision

    float sqrt_d = sqrtf(discriminant);
    float t1 = (-b - sqrt_d) / (2.0f * a);
    float t2 = (-b + sqrt_d) / (2.0f * a);

    float t_collision = -1.0f;

    // Select the smallest non-negative t within dt_max
    if (t1 >= -EPSILON2 && t1 <= dt_max + EPSILON2) {
        t_collision = t1;
    }
    if (t2 >= -EPSILON2 && t2 <= dt_max + EPSILON2) {
        if (t_collision < -EPSILON2 || t2 < t_collision) {
            t_collision = t2;
        }
    }

    if (t_collision >= -EPSILON2) {
        *toi = fmaxf(0.0f, t_collision); // Ensure toi is not negative
        Vector2 ball_center_at_toi = Vector2Add(ballPos, Vector2Scale(ballVel, *toi));
        *normal = Vector2Normalize(Vector2Subtract(ball_center_at_toi, point)); // Normal from point to ball center at TOI
        if (Vector2LengthSqr(*normal) < EPSILON2) { // Degenerate case
             *normal = Vector2Normalize(Vector2Subtract(ballPos, point)); // Fallback to initial direction
             if(Vector2LengthSqr(*normal) < EPSILON2) *normal = (Vector2){0,-1};
        }
        return true;
    }
    return false;
}

// Swept collision: ball moving towards a static line segment
bool sweptBallToStaticSegmentCollision(Vector2 segP1, Vector2 segP2,
                                       Vector2 ballPos, Vector2 ballVel, float ballRadius,
                                       float dt_max, float* toi, Vector2* normal) {
    float min_valid_toi = dt_max + EPSILON2; // Initialize to be greater than any valid TOI
    bool collided = false;
    Vector2 final_normal = {0,0};

    // 1. Check collision with segment endpoints (as points)
    float current_toi_p1, current_toi_p2;
    Vector2 current_normal_p1, current_normal_p2;

    if (sweptBallToStaticPointCollision(segP1, ballPos, ballVel, ballRadius, dt_max, &current_toi_p1, &current_normal_p1)) {
        if (current_toi_p1 < min_valid_toi) {
            min_valid_toi = current_toi_p1;
            final_normal = current_normal_p1;
            collided = true;
        }
    }
    if (sweptBallToStaticPointCollision(segP2, ballPos, ballVel, ballRadius, dt_max, &current_toi_p2, &current_normal_p2)) {
        if (current_toi_p2 < min_valid_toi) {
            min_valid_toi = current_toi_p2;
            final_normal = current_normal_p2;
            collided = true;
        }
    }

    // 2. Check collision with the segment line itself
    Vector2 segmentVec = Vector2Subtract(segP2, segP1);
    float segmentLenSq = Vector2LengthSqr(segmentVec);
    if (segmentLenSq < EPSILON2) { // Segment is essentially a point, already handled
        if (collided) {
            *toi = min_valid_toi;
            *normal = final_normal;
        }
        return collided;
    }

    // Project ball's position relative to segP1 onto segmentVec and its perpendicular
    Vector2 relPos = Vector2Subtract(ballPos, segP1);
    Vector2 segDirNormalized = Vector2Normalize(segmentVec);
    Vector2 segPerpDir = {-segDirNormalized.y, segDirNormalized.x}; // Normal to the segment's direction

    // Distance from ball center to the infinite line defined by the segment
    // d = (C - P1) . PerpDir
    float distToLine = Vector2DotProduct(relPos, segPerpDir);
    // Velocity component towards the line
    // v_perp = V . PerpDir
    float velCompTowardsLine = Vector2DotProduct(ballVel, segPerpDir);

    if (fabsf(velCompTowardsLine) < EPSILON2) { // Ball moving parallel to segment line
        if (collided) {
            *toi = min_valid_toi;
            *normal = final_normal;
        }
        return collided;
    }

    // Time to reach distance R from line: (R - d) / v_perp or (-R - d) / v_perp
    float t_line1 = (ballRadius - distToLine) / velCompTowardsLine;
    float t_line2 = (-ballRadius - distToLine) / velCompTowardsLine;
    
    float t_line_collision = -1.0f;
    if (t_line1 >= -EPSILON2 && t_line1 <= dt_max + EPSILON2) {
        t_line_collision = t_line1;
    }
    if (t_line2 >= -EPSILON2 && t_line2 <= dt_max + EPSILON2) {
        if (t_line_collision < -EPSILON2 || t_line2 < t_line_collision) {
            t_line_collision = t_line2;
        }
    }

    if (t_line_collision >= -EPSILON2 && t_line_collision < min_valid_toi) {
        // Check if the collision point on the line is within the segment's projection
        Vector2 ballCenterAtToi = Vector2Add(ballPos, Vector2Scale(ballVel, t_line_collision));
        // Point on segment line closest to ballCenterAtToi (this is ballCenterAtToi - (dist_at_toi * segPerpDir))
        Vector2 collisionPointOnLine = Vector2Subtract(ballCenterAtToi, Vector2Scale(segPerpDir, Vector2DotProduct(Vector2Subtract(ballCenterAtToi, segP1), segPerpDir)));
        
        // Project this point onto the segment vector (from segP1)
        float projection = Vector2DotProduct(Vector2Subtract(collisionPointOnLine, segP1), segDirNormalized);

        if (projection >= -EPSILON2 && projection <= sqrtf(segmentLenSq) + EPSILON2) { // Collision point is on the segment
            min_valid_toi = t_line_collision;
            // Normal is from line to ball. If ball hit from "positive" side of segPerpDir, normal is segPerpDir.
            // If hit from "negative" side, normal is -segPerpDir.
            // This is equivalent to: normal = sign(distToLine_at_impact) * segPerpDir
            // Or, more simply, normal = normalize(ball_center_at_toi - collisionPointOnLine)
            final_normal = Vector2Normalize(Vector2Subtract(ballCenterAtToi, collisionPointOnLine));
            if (Vector2LengthSqr(final_normal) < EPSILON2) { // Should not happen if velCompTowardsLine != 0
                final_normal = (distToLine > 0) ? segPerpDir : Vector2Negate(segPerpDir);
            }
            collided = true;
        }
    }
    
    if (collided) {
        *toi = fmaxf(0.0f, min_valid_toi); // Ensure non-negative TOI
        *normal = final_normal;
         if (Vector2LengthSqr(*normal) < EPSILON2) { // Safety for zero normal
            *normal = Vector2Normalize(Vector2Negate(ballVel));
            if(Vector2LengthSqr(*normal) < EPSILON2) *normal = (Vector2){0,-1};
        }
    }
    return collided;
}


// --- Object List Management ---
void addObjectToList(GameObject** head, GameObject* newObject) {
    if (!newObject) return;
    newObject->next = *head;
    *head = newObject;
}

void freeObjectList(GameObject** head) {
    GameObject* current = *head;
    GameObject* next;
    while (current != NULL) {
        next = current->next;
        if (current->destroy) {
            current->destroy(current);
        }
        free(current);
        current = next;
    }
    *head = NULL;
}

void updateObjectList(GameObject* head, float dt) {
    for (GameObject* current = head; current != NULL; current = current->next) {
        if (current->update) {
            current->update(current, dt);
        }
    }
}

void renderObjectList(GameObject* head) {
    for (GameObject* current = head; current != NULL; current = current->next) {
        if (current->render) {
            current->render(current);
        }
    }
}

// Remove all game objects marked for deletion
void removeMarkedGameObjects(GameObject** head) {
    GameObject* current = *head;
    GameObject* prev = NULL;
    GameObject* next = NULL;
    
    while (current != NULL) {
        next = current->next;
        
        if (current->markedForDeletion) {
            // If it's the head of the list
            if (prev == NULL) {
                *head = next;
            } else {
                prev->next = next;
            }
            
            // Free resources associated with this object
            freeEffectList(&current->onCollisionEffects);
            if (current->destroy) {
                current->destroy(current);
            }
            free(current);
        } else {
            prev = current;
        }
        
        current = next;
    }
}

// --- Generic Object Functions ---
static void updateGenericMovingObject(GameObject* self, float dt) {
    if (!self || self->isStatic) return;
    self->position = Vector2Add(self->position, Vector2Scale(self->velocity, dt));

    // Basic screen wrap for objects (optional, can be removed or improved)
    if (self->position.x < -50) self->position.x = SCREEN_WIDTH + 40;
    if (self->position.x > SCREEN_WIDTH + 50) self->position.x = -40;
    if (self->position.y < -50) self->position.y = SCREEN_HEIGHT + 40;
    if (self->position.y > SCREEN_HEIGHT + 50) self->position.y = -40;
}

static void destroyGenericShapeData(GameObject* self) {
    if (self && self->shapeData) {
        // Free any callback lists for ArcCircle objects
        if (self->type == SHAPE_CIRCLE_ARC) {
            ShapeDataArcCircle* arcData = (ShapeDataArcCircle*)self->shapeData;
            freeArcCircleCallbackList(&arcData->onCollisionCallbacks);
            freeArcCircleCallbackList(&arcData->onEscapeCallbacks);
        }
        
        free(self->shapeData);
        self->shapeData = NULL;
    }
}

// --- Rectangle Object ---
static void renderRectangleObj(GameObject* self) {
    ShapeDataRectangle* data = (ShapeDataRectangle*)self->shapeData;
    // DrawRectanglePro takes center, dimensions, origin (for rotation), rotation, color
    DrawRectanglePro(
        (Rectangle){self->position.x, self->position.y, data->width, data->height},
        (Vector2){data->width / 2.0f, data->height / 2.0f}, // Origin at center
        0.0f, // No rotation for now
        data->color
    );
}

static bool checkCollisionRectangleObj(GameObject* self, BouncingObject* bouncingObj, float dt_step, float* timeOfImpact, Vector2* collisionNormal) {    ShapeDataRectangle* data = (ShapeDataRectangle*)self->shapeData;
    // Relative velocity of bouncing object with respect to the (potentially moving) object
    Vector2 relBallVel = Vector2Subtract(bouncingObj->velocity, self->velocity);

    // Rectangle vertices (object's position is its center)
    float hw = data->width / 2.0f;
    float hh = data->height / 2.0f;
    Vector2 objPos = self->position; // Current position of the object for segment calculation

    Vector2 p1 = {objPos.x - hw, objPos.y - hh}; // Top-left
    Vector2 p2 = {objPos.x + hw, objPos.y - hh}; // Top-right
    Vector2 p3 = {objPos.x + hw, objPos.y + hh}; // Bottom-right
    Vector2 p4 = {objPos.x - hw, objPos.y + hh}; // Bottom-left

    Vector2 segments[4][2] = {
        {p1, p2}, {p2, p3}, {p3, p4}, {p4, p1}
    };

    float min_toi = dt_step + EPSILON2; // Initialize with a value larger than dt_step
    bool collided_overall = false;    for (int i = 0; i < 4; ++i) {
        float current_segment_toi;
        Vector2 current_segment_normal;
        // Pass bouncing object's current position, its RELATIVE velocity, and segment (considered static in this call)
        if (sweptBallToStaticSegmentCollision(segments[i][0], segments[i][1],
                                              bouncingObj->position, relBallVel, bouncingObj->radius,
                                              dt_step, &current_segment_toi, &current_segment_normal)) {
            if (current_segment_toi >= -EPSILON2 && current_segment_toi < min_toi) {
                min_toi = current_segment_toi;
                *collisionNormal = current_segment_normal; // Normal from segment towards ball
                collided_overall = true;
            }
        }
    }

    if (collided_overall) {
        *timeOfImpact = min_toi;
    }
    return collided_overall;
}

GameObject* createRectangleObject(Vector2 position, Vector2 velocity, float width, float height, Color color, bool isStatic) {
    GameObject* obj = (GameObject*)malloc(sizeof(GameObject));
    if (!obj) return NULL;
    ShapeDataRectangle* data = (ShapeDataRectangle*)malloc(sizeof(ShapeDataRectangle));
    if (!data) { free(obj); return NULL; }

    data->width = width; data->height = height; data->color = color;    obj->type = SHAPE_RECTANGLE;
    obj->position = position;
    obj->velocity = isStatic ? (Vector2){0,0} : velocity;
    obj->shapeData = data;
    obj->isStatic = isStatic;
    obj->markedForDeletion = false;
    obj->render = renderRectangleObj;
    obj->checkCollision = checkCollisionRectangleObj;
    obj->update = updateGenericMovingObject;
    obj->destroy = destroyGenericShapeData;
    obj->next = NULL;
    return obj;
}

// --- Diamond Object ---
static void renderDiamondObj(GameObject* self) {
    ShapeDataDiamond* data = (ShapeDataDiamond*)self->shapeData;
    Vector2 p = self->position;
    float hw = data->halfWidth; float hh = data->halfHeight;
    Vector2 top = {p.x, p.y - hh}; Vector2 right = {p.x + hw, p.y};
    Vector2 bottom = {p.x, p.y + hh}; Vector2 left = {p.x - hw, p.y};
    DrawLineV(top, right, data->color); DrawLineV(right, bottom, data->color);
    DrawLineV(bottom, left, data->color); DrawLineV(left, top, data->color);
    // Or fill with triangles: DrawTriangle(top, left, right, data->color); DrawTriangle(bottom, left, right, data->color);
}

static bool checkCollisionDiamondObj(GameObject* self, BouncingObject* bouncingObj, float dt_step, float* timeOfImpact, Vector2* collisionNormal) {    ShapeDataDiamond* data = (ShapeDataDiamond*)self->shapeData;
    Vector2 relBallVel = Vector2Subtract(bouncingObj->velocity, self->velocity);
    Vector2 p = self->position;
    float hw = data->halfWidth; float hh = data->halfHeight;

    Vector2 topV = {p.x, p.y - hh}; Vector2 rightV = {p.x + hw, p.y};
    Vector2 bottomV = {p.x, p.y + hh}; Vector2 leftV = {p.x - hw, p.y};
    Vector2 segments[4][2] = { {topV, rightV}, {rightV, bottomV}, {bottomV, leftV}, {leftV, topV} };
    
    float min_toi = dt_step + EPSILON2;
    bool collided_overall = false;

    for (int i = 0; i < 4; ++i) {
        float current_segment_toi;
        Vector2 current_segment_normal;        if (sweptBallToStaticSegmentCollision(segments[i][0], segments[i][1],
                                              bouncingObj->position, relBallVel, bouncingObj->radius,
                                              dt_step, &current_segment_toi, &current_segment_normal)) {
            if (current_segment_toi >= -EPSILON2 && current_segment_toi < min_toi) {
                min_toi = current_segment_toi;
                *collisionNormal = current_segment_normal;
                collided_overall = true;
            }
        }
    }
    if (collided_overall) {
        *timeOfImpact = min_toi;
    }
    return collided_overall;
}

GameObject* createDiamondObject(Vector2 position, Vector2 velocity, float diagWidth, float diagHeight, Color color, bool isStatic) {
    GameObject* obj = (GameObject*)malloc(sizeof(GameObject));
    if (!obj) return NULL;
    ShapeDataDiamond* data = (ShapeDataDiamond*)malloc(sizeof(ShapeDataDiamond));
    if (!data) { free(obj); return NULL; }

    data->halfWidth = diagWidth / 2.0f; data->halfHeight = diagHeight / 2.0f; data->color = color;    obj->type = SHAPE_DIAMOND;
    obj->position = position;
    obj->velocity = isStatic ? (Vector2){0,0} : velocity;
    obj->shapeData = data;
    obj->isStatic = isStatic;
    obj->markedForDeletion = false;
    obj->render = renderDiamondObj;
    obj->checkCollision = checkCollisionDiamondObj;
    obj->update = updateGenericMovingObject;
    obj->destroy = destroyGenericShapeData;
    obj->next = NULL;
    return obj;
}

// --- Arc Circle Object ---
static void renderArcCircleObj(GameObject* self) {
    ShapeDataArcCircle* data = (ShapeDataArcCircle*)self->shapeData;
    // Draw the arc
    DrawRing(
        self->position,
        data->radius - data->thickness/2,
        data->radius + data->thickness/2,
        data->startAngle + data->rotation,
        data->endAngle + data->rotation,
        36, // Number of segments (for smoother arcs)
        data->color
    );
}

static void updateArcCircleObj(GameObject* self, float dt) {
    if (!self) return;
    
    // Update rotation regardless of whether the object is static
    ShapeDataArcCircle* data = (ShapeDataArcCircle*)self->shapeData;
    data->rotation += data->rotationSpeed * dt;
    
    // Only update position if the object is not static
    if (!self->isStatic) {
        // Update position based on velocity
        self->position = Vector2Add(self->position, Vector2Scale(self->velocity, dt));
    }
    // Normalize rotation to avoid large values over time
    while (data->rotation > 360.0f) {
        data->rotation -= 360.0f;
    }
    while (data->rotation < 0.0f) {
        data->rotation += 360.0f;
    }
    
    // Basic screen wrap for objects (optional)
    if (self->position.x < -50) self->position.x = SCREEN_WIDTH + 40;
    if (self->position.x > SCREEN_WIDTH + 50) self->position.x = -40;
    if (self->position.y < -50) self->position.y = SCREEN_HEIGHT + 40;
    if (self->position.y > SCREEN_HEIGHT + 50) self->position.y = -40;
}

static bool isBallInsideCircle(Vector2 ballPos, Vector2 circlePos, float circleRadius, float thickness)
{
    // Calculate the distance between the ball center and the circle center
    float distance = Vector2Distance(ballPos, circlePos);
    
    // Check if the ball is within the circle zone (considering the radius and thickness)
    float outerRadius = circleRadius + thickness/2;
    
    return distance <= outerRadius;
}

// Check if a point is within the angular range of an arc
static bool isPointWithinArcAngles(Vector2 point, Vector2 center, float startAngle, float endAngle, float currentRotation)
{
    // Calculate the angle of the point relative to the center (in degrees)
    float dx = point.x - center.x;
    float dy = point.y - center.y;
    float pointAngle = atan2f(dy, dx) * RAD2DEG;
    
    // Normalize to [0, 360] range
    if (pointAngle < 0) pointAngle += 360.0f;
    
    // Apply the rotation offset and normalize
    float effectiveStart = fmodf(startAngle + currentRotation, 360.0f);
    float effectiveEnd = fmodf(endAngle + currentRotation, 360.0f);
    
    // Handle cases where the arc crosses the 0-degree line
    if (effectiveStart <= effectiveEnd) {
        return (pointAngle >= effectiveStart && pointAngle <= effectiveEnd);
    } else {
        // Arc wraps around from 360 back to 0
        return (pointAngle >= effectiveStart || pointAngle <= effectiveEnd);
    }
}

static bool checkCollisionArcCircleObj(GameObject* self, BouncingObject* bouncingObj, float dt_step, float* timeOfImpact, Vector2* collisionNormal)
{
    if (!self || !bouncingObj) return false;
    
    ShapeDataArcCircle* data = (ShapeDataArcCircle*)self->shapeData;
    if (!data) return false;
    
    // Get relative velocity (bouncing object relative to the arc circle)
    Vector2 relBallVel = Vector2Subtract(bouncingObj->velocity, self->velocity);
    
    // Store center position for clarity
    Vector2 arcCenter = self->position;
    
    // Set up variables for collision detection
    float min_toi = dt_step + EPSILON2; // Initialize to be greater than any valid TOI
    bool collided = false;
    Vector2 final_normal = {0,0};
    
    // Calculate inner and outer radii of the arc circle
    float innerRadius = data->radius - data->thickness/2.0f;
    float outerRadius = data->radius + data->thickness/2.0f;
    
    // 1. Check for collision with the outer circle boundary
    {
        // Use swept ball to static point collision but with negative radius
        // This simulates a ball hitting a circle from outside
        
        // Quadratic equation: |ballPos + ballVel*t - circleCenter|^2 = (ballRadius + outerRadius)^2
        // Simplify by treating it as a point vs sphere collision
        Vector2 relPos = Vector2Subtract(bouncingObj->position, arcCenter);
        float combinedRadius = bouncingObj->radius + outerRadius;
        
        float a = Vector2DotProduct(relBallVel, relBallVel);
        float b = 2.0f * Vector2DotProduct(relPos, relBallVel);
        float c = Vector2DotProduct(relPos, relPos) - combinedRadius * combinedRadius;
        
        if (fabsf(a) < EPSILON2) { // Velocity is very small
            if (c <= 0) { // Already overlapping
                float distance = Vector2Length(relPos);
                if (distance <= combinedRadius + EPSILON2) {
                    if (distance < EPSILON2) { // Ball center very close to circle center
                        final_normal = Vector2Normalize(relBallVel);
                        if (Vector2LengthSqr(final_normal) < EPSILON2) {
                            final_normal = (Vector2){1, 0}; // Default direction
                        } else {
                            final_normal = Vector2Negate(final_normal); // Away from velocity
                        }
                    } else {
                        final_normal = Vector2Normalize(relPos); // Normal points from circle to ball
                    }
                    
                    min_toi = 0.0f; // Immediate collision
                    collided = true;
                }
            }
        } else {
            // Solve quadratic equation
            float discriminant = b * b - 4.0f * a * c;
            if (discriminant >= 0) {
                float sqrt_d = sqrtf(discriminant);
                float t1 = (-b - sqrt_d) / (2.0f * a);
                float t2 = (-b + sqrt_d) / (2.0f * a);
                
                // Find earliest valid collision time
                float t_collision = -1.0f;
                if (t1 >= -EPSILON2 && t1 <= dt_step + EPSILON2) {
                    t_collision = t1;
                }
                if (t2 >= -EPSILON2 && t2 <= dt_step + EPSILON2 && t_collision < -EPSILON2) {
                    t_collision = t2;
                }
                
                if (t_collision >= -EPSILON2 && t_collision < min_toi) {
                    // Calculate ball position at time of impact
                    Vector2 ballPosAtToi = Vector2Add(bouncingObj->position, Vector2Scale(relBallVel, t_collision));
                    // Calculate collision normal (from circle center to ball center)
                    Vector2 normal = Vector2Subtract(ballPosAtToi, arcCenter);
                    
                    // Check if the collision point is within the angular range of the arc
                    if (isPointWithinArcAngles(ballPosAtToi, arcCenter, data->startAngle, data->endAngle, data->rotation)) {
                        min_toi = t_collision;
                        final_normal = Vector2Normalize(normal);
                        collided = true;
                    }
                }
            }
        }
    }
    
    // 2. Check for collision with the inner circle boundary (only if thickness > 0)
    if (innerRadius > EPSILON2) {
        Vector2 relPos = Vector2Subtract(bouncingObj->position, arcCenter);
        float combinedRadius = innerRadius - bouncingObj->radius; // Note the subtraction
        
        // Only check if the combined radius is positive
        if (combinedRadius > EPSILON2) {
            float a = Vector2DotProduct(relBallVel, relBallVel);
            float b = 2.0f * Vector2DotProduct(relPos, relBallVel);
            float c = Vector2DotProduct(relPos, relPos) - combinedRadius * combinedRadius;
            
            // Solving quadratic equation for inner collision
            if (fabsf(a) >= EPSILON2) {
                float discriminant = b * b - 4.0f * a * c;
                if (discriminant >= 0) {
                    float sqrt_d = sqrtf(discriminant);
                    float t1 = (-b - sqrt_d) / (2.0f * a);
                    float t2 = (-b + sqrt_d) / (2.0f * a);
                    
                    // Find earliest valid collision time
                    float t_collision = -1.0f;
                    if (t1 >= -EPSILON2 && t1 <= dt_step + EPSILON2) {
                        t_collision = t1;
                    }
                    if (t2 >= -EPSILON2 && t2 <= dt_step + EPSILON2 && t_collision < -EPSILON2) {
                        t_collision = t2;
                    }
                    
                    if (t_collision >= -EPSILON2 && t_collision < min_toi) {
                        // Calculate ball position at time of impact
                        Vector2 ballPosAtToi = Vector2Add(bouncingObj->position, Vector2Scale(relBallVel, t_collision));
                        // Calculate collision normal (from ball center to circle center - opposite from outer collision)
                        Vector2 normal = Vector2Subtract(arcCenter, ballPosAtToi);
                        
                        // Check if the collision point is within the angular range of the arc
                        if (isPointWithinArcAngles(ballPosAtToi, arcCenter, data->startAngle, data->endAngle, data->rotation)) {
                            min_toi = t_collision;
                            final_normal = Vector2Normalize(normal);
                            collided = true;
                        }
                    }
                }
            }
        }
    }
    
    // 3. Check for collision with the end points of the arc (if they exist)
    if (data->endAngle - data->startAngle < 360.0f) {
        // Calculate the end points of the arc
        float startRad = (data->startAngle + data->rotation) * DEG2RAD;
        float endRad = (data->endAngle + data->rotation) * DEG2RAD;
        
        Vector2 startOuter = {
            arcCenter.x + outerRadius * cosf(startRad),
            arcCenter.y + outerRadius * sinf(startRad)
        };
        
        Vector2 endOuter = {
            arcCenter.x + outerRadius * cosf(endRad),
            arcCenter.y + outerRadius * sinf(endRad)
        };
        
        // If thickness > 0, we also need to check inner endpoints
        Vector2 startInner = {
            arcCenter.x + innerRadius * cosf(startRad),
            arcCenter.y + innerRadius * sinf(startRad)
        };
        
        Vector2 endInner = {
            arcCenter.x + innerRadius * cosf(endRad),
            arcCenter.y + innerRadius * sinf(endRad)
        };
        
        // Check collision with start outer endpoint
        float current_toi;
        Vector2 current_normal;
        if (sweptBallToStaticPointCollision(startOuter, bouncingObj->position, relBallVel,
                                           bouncingObj->radius, dt_step, &current_toi, &current_normal)) {
            if (current_toi < min_toi) {
                min_toi = current_toi;
                final_normal = current_normal;
                collided = true;
            }
        }
        
        // Check collision with end outer endpoint
        if (sweptBallToStaticPointCollision(endOuter, bouncingObj->position, relBallVel,
                                           bouncingObj->radius, dt_step, &current_toi, &current_normal)) {
            if (current_toi < min_toi) {
                min_toi = current_toi;
                final_normal = current_normal;
                collided = true;
            }
        }
        
        // Check collision with start inner endpoint (if there's thickness)
        if (innerRadius > EPSILON2) {
            if (sweptBallToStaticPointCollision(startInner, bouncingObj->position, relBallVel,
                                               bouncingObj->radius, dt_step, &current_toi, &current_normal)) {
                if (current_toi < min_toi) {
                    min_toi = current_toi;
                    final_normal = current_normal;
                    collided = true;
                }
            }
            
            // Check collision with end inner endpoint
            if (sweptBallToStaticPointCollision(endInner, bouncingObj->position, relBallVel,
                                               bouncingObj->radius, dt_step, &current_toi, &current_normal)) {
                if (current_toi < min_toi) {
                    min_toi = current_toi;
                    final_normal = current_normal;
                    collided = true;
                }
            }
            
            // If the arc isn't a full circle, check collisions with the straight segments
            // connecting the inner and outer endpoints
            Vector2 startSegment[2] = { startInner, startOuter };
            Vector2 endSegment[2] = { endInner, endOuter };
            
            // Check collision with start segment
            if (sweptBallToStaticSegmentCollision(startSegment[0], startSegment[1],
                                                 bouncingObj->position, relBallVel, bouncingObj->radius,
                                                 dt_step, &current_toi, &current_normal)) {
                if (current_toi < min_toi) {
                    min_toi = current_toi;
                    final_normal = current_normal;
                    collided = true;
                }
            }
            
            // Check collision with end segment
            if (sweptBallToStaticSegmentCollision(endSegment[0], endSegment[1],
                                                 bouncingObj->position, relBallVel, bouncingObj->radius,
                                                 dt_step, &current_toi, &current_normal)) {
                if (current_toi < min_toi) {
                    min_toi = current_toi;
                    final_normal = current_normal;
                    collided = true;
                }
            }
        }
    }    // Check if the ball is escaping through the gap of the arc (NOT through the arc itself)
    if (data->onEscapeCallbacks != NULL) {
        // Determine if ball is inside the circle now or will be after moving
        Vector2 ballPosAfterStep = Vector2Add(bouncingObj->position, Vector2Scale(bouncingObj->velocity, dt_step));
        bool ballIsInsideNow = isBallInsideCircle(bouncingObj->position, arcCenter, data->radius, data->thickness);
        bool ballWillBeInsideAfter = isBallInsideCircle(ballPosAfterStep, arcCenter, data->radius, data->thickness);
        
        // We only care about balls that are either:
        // 1. Going from inside to outside (escaping)
        // 2. Already outside with velocity pointing away from circle
        if (!ballIsInsideNow && !ballWillBeInsideAfter) {
            // Ball is already outside and staying outside, no escape event
            return collided;
        }
        
        // If the ball is leaving the circle's interior
        if (ballIsInsideNow && !ballWillBeInsideAfter) {
            // Calculate angle of ball position to check if it's leaving through the gap
            Vector2 ballRelPos = Vector2Subtract(bouncingObj->position, arcCenter);
            float ballAngle = atan2f(ballRelPos.y, ballRelPos.x) * RAD2DEG;
            if (ballAngle < 0) ballAngle += 360.0f;
            
            // Calculate exit trajectory and intersection with circle boundary
            Vector2 trajectory = Vector2Normalize(bouncingObj->velocity);
            float outerRadius = data->radius + data->thickness/2;
            
            // Project the position to the boundary to determine the escape point
            Vector2 escapePoint = Vector2Add(arcCenter, Vector2Scale(Vector2Normalize(ballRelPos), outerRadius));
            
            // IMPORTANT: Check if the escape point is NOT within the arc angles
            // This means the ball is escaping through the GAP, not through the arc itself
            if (!isPointWithinArcAngles(escapePoint, arcCenter, data->startAngle, data->endAngle, data->rotation)) {
                // Ball is escaping through the GAP (not the arc), call all escape callbacks
                for (ArcCircleCallbackNode* node = data->onEscapeCallbacks; node != NULL; node = node->next) {
                    if (node->callback) {
                        node->callback(self, bouncingObj);
                    }
                }
                
                // If the arc is configured to remove escaped balls, mark the ball for deletion
                if (data->removeEscapedBalls) {
                    bouncingObj->markedForDeletion = true;
                }
            }
        }
    }
    
    if (collided) {
        // If colliding, call all collision callbacks
        if (data->onCollisionCallbacks != NULL) {
            for (ArcCircleCallbackNode* node = data->onCollisionCallbacks; node != NULL; node = node->next) {
                if (node->callback) {
                    node->callback(self, bouncingObj);
                }
            }
        }
        
        *timeOfImpact = min_toi;
        *collisionNormal = final_normal;
    }
    
    return collided;
}

GameObject* createArcCircleObject(Vector2 position, Vector2 velocity, float radius, float startAngle, float endAngle, float thickness, Color color, bool isStatic, float rotationSpeed, bool removeEscapedBalls) {
    GameObject* obj = (GameObject*)malloc(sizeof(GameObject));
    if (!obj) return NULL;
    
    ShapeDataArcCircle* data = (ShapeDataArcCircle*)malloc(sizeof(ShapeDataArcCircle));
    if (!data) { 
        free(obj); 
        return NULL; 
    }
    
    data->radius = radius;
    data->startAngle = startAngle;
    data->endAngle = endAngle;
    data->thickness = thickness;
    data->color = color;    data->rotation = 0.0f;
    data->rotationSpeed = rotationSpeed;
    data->removeEscapedBalls = removeEscapedBalls;
    data->onCollisionCallbacks = NULL;  // Initialize callback lists to empty
    data->onEscapeCallbacks = NULL;
      obj->type = SHAPE_CIRCLE_ARC;
    obj->position = position;
    obj->velocity = isStatic ? (Vector2){0,0} : velocity;
    obj->shapeData = data;
    obj->isStatic = isStatic;
    obj->markedForDeletion = false;
    obj->render = renderArcCircleObj;
    obj->checkCollision = checkCollisionArcCircleObj;
    obj->update = updateArcCircleObj;
    obj->destroy = destroyGenericShapeData;
    obj->next = NULL;
    obj->onCollisionEffects = NULL;
    
    return obj;
}

// --- ArcCircle Callback Management Functions ---

// Free a linked list of ArcCircle callbacks
void freeArcCircleCallbackList(ArcCircleCallbackNode** head) {
    ArcCircleCallbackNode* current = *head;
    ArcCircleCallbackNode* next;
    
    while (current != NULL) {
        next = current->next;
        free(current);
        current = next;
    }
    
    *head = NULL;
}

// Add a collision callback to an ArcCircle object
void addCollisionCallbackToArcCircle(GameObject* arcCircle, ArcCircleCallback callback) {
    if (!arcCircle || arcCircle->type != SHAPE_CIRCLE_ARC || !callback) return;
    
    ShapeDataArcCircle* data = (ShapeDataArcCircle*)arcCircle->shapeData;
    if (!data) return;
    
    ArcCircleCallbackNode* newNode = (ArcCircleCallbackNode*)malloc(sizeof(ArcCircleCallbackNode));
    if (!newNode) return;
    
    newNode->callback = callback;
    newNode->next = data->onCollisionCallbacks;
    data->onCollisionCallbacks = newNode;
}

// Add an escape callback to an ArcCircle object
void addEscapeCallbackToArcCircle(GameObject* arcCircle, ArcCircleCallback callback) {
    if (!arcCircle || arcCircle->type != SHAPE_CIRCLE_ARC || !callback) return;
    
    ShapeDataArcCircle* data = (ShapeDataArcCircle*)arcCircle->shapeData;
    if (!data) return;
    
    ArcCircleCallbackNode* newNode = (ArcCircleCallbackNode*)malloc(sizeof(ArcCircleCallbackNode));
    if (!newNode) return;
    
    newNode->callback = callback;
    newNode->next = data->onEscapeCallbacks;
    data->onEscapeCallbacks = newNode;
}

// --- BouncingObject Management Functions ---

// Create a new bouncing object
BouncingObject* createBouncingObject(Vector2 position, Vector2 velocity, float radius, Color color, float mass, float restitution, bool interactWithOtherBouncingObjects) {
    BouncingObject* obj = (BouncingObject*)malloc(sizeof(BouncingObject));
    if (!obj) return NULL;
    
    obj->position = position;
    obj->velocity = velocity;
    obj->radius = radius;
    obj->color = color;
    obj->mass = (mass > 0.0f) ? mass : 1.0f; // Ensure positive mass
    obj->restitution = Clamp(restitution, 0.0f, 1.0f); // Ensure valid restitution
    obj->interactWithOtherBouncingObjects = interactWithOtherBouncingObjects;
    obj->markedForDeletion = false; // Initially not marked for deletion
    obj->onCollisionEffects = NULL;
    obj->next = NULL;
    
    return obj;
}

// Add a bouncing object to a linked list
void addBouncingObjectToList(BouncingObject** head, BouncingObject* newObject) {
    if (!newObject) return;
    newObject->next = *head;
    *head = newObject;
}

// Free a linked list of bouncing objects
void freeBouncingObjectList(BouncingObject** head) {
    BouncingObject* current = *head;
    BouncingObject* next;
    
    while (current != NULL) {
        next = current->next;
        // Free any effects attached to this object
        freeEffectList(&current->onCollisionEffects);
        free(current);
        current = next;
    }
    
    *head = NULL;
}

// Remove all bouncing objects marked for deletion
void removeMarkedBouncingObjects(BouncingObject** head) {
    BouncingObject* current = *head;
    BouncingObject* prev = NULL;
    BouncingObject* next = NULL;
    
    while (current != NULL) {
        next = current->next;
        
        if (current->markedForDeletion) {
            // If it's the head of the list
            if (prev == NULL) {
                *head = next;
            } else {
                prev->next = next;
            }
            
            // Free resources associated with this object
            freeEffectList(&current->onCollisionEffects);
            free(current);
        } else {
            prev = current;
        }
        
        current = next;
    }
}

// Update all bouncing objects in a list
void updateBouncingObjectList(BouncingObject* head, float dt) {
    for (BouncingObject* current = head; current != NULL; current = current->next) {
        // Update position based on velocity
        current->position = Vector2Add(current->position, Vector2Scale(current->velocity, dt));
        
        // Basic screen wrap for bouncing objects (optional)
        if (current->position.x < -50) current->position.x = SCREEN_WIDTH + 40;
        if (current->position.x > SCREEN_WIDTH + 50) current->position.x = -40;
        if (current->position.y < -50) current->position.y = SCREEN_HEIGHT + 40;
        if (current->position.y > SCREEN_HEIGHT + 50) current->position.y = -40;
    }
}

// Render all bouncing objects in a list
void renderBouncingObjectList(BouncingObject* head) {
    for (BouncingObject* current = head; current != NULL; current = current->next) {
        DrawCircleV(current->position, current->radius, current->color);
    }
}

// --- Collision Effect Functions ---

// Create a color change effect
CollisionEffect* createColorChangeEffect(Color newColor, bool continuous) {
    CollisionEffect* effect = (CollisionEffect*)malloc(sizeof(CollisionEffect));
    if (!effect) return NULL;
    
    effect->type = EFFECT_COLOR_CHANGE;
    effect->continuous = continuous;
    effect->params.colorEffect.color = newColor;
    effect->next = NULL;
    
    return effect;
}

// Create a velocity boost effect
CollisionEffect* createVelocityBoostEffect(float factor, bool continuous) {
    CollisionEffect* effect = (CollisionEffect*)malloc(sizeof(CollisionEffect));
    if (!effect) return NULL;
    
    effect->type = EFFECT_VELOCITY_BOOST;
    effect->continuous = continuous;
    effect->params.velocityEffect.factor = (factor > 1.0f) ? factor : 1.1f; // Default 10% boost
    effect->next = NULL;
    
    return effect;
}

// Create a velocity dampen effect
CollisionEffect* createVelocityDampenEffect(float factor, bool continuous) {
    CollisionEffect* effect = (CollisionEffect*)malloc(sizeof(CollisionEffect));
    if (!effect) return NULL;
    
    effect->type = EFFECT_VELOCITY_DAMPEN;
    effect->continuous = continuous;
    effect->params.velocityEffect.factor = Clamp(factor, 0.01f, 0.99f); // Default dampen
    effect->next = NULL;
    
    return effect;
}

// Create a size change effect
CollisionEffect* createSizeChangeEffect(float factor, bool continuous) {
    CollisionEffect* effect = (CollisionEffect*)malloc(sizeof(CollisionEffect));
    if (!effect) return NULL;
    
    effect->type = EFFECT_SIZE_CHANGE;
    effect->continuous = continuous;
    effect->params.sizeEffect.factor = factor;
    effect->next = NULL;
    
    return effect;
}

// Create a sound play effect
CollisionEffect* createSoundPlayEffect(Sound sound, bool continuous) {
    CollisionEffect* effect = (CollisionEffect*)malloc(sizeof(CollisionEffect));
    if (!effect) return NULL;
    
    effect->type = EFFECT_SOUND_PLAY;
    effect->continuous = continuous;
    effect->params.soundEffect.sound = sound;
    effect->next = NULL;
    
    return effect;
}

// Create a ball disappear effect
CollisionEffect* createBallDisappearEffect(int particleCount, Color particleColor, bool continuous) {
    CollisionEffect* effect = (CollisionEffect*)malloc(sizeof(CollisionEffect));
    if (!effect) return NULL;
    
    effect->type = EFFECT_BALL_DISAPPEAR;
    effect->continuous = continuous;
    effect->params.disappearEffect.particleCount = particleCount;
    effect->params.disappearEffect.particleColor = particleColor;
    effect->next = NULL;
    
    return effect;
}

// Create a ball spawn effect
CollisionEffect* createBallSpawnEffect(Vector2 position, float radius, Color color, bool continuous) {
    CollisionEffect* effect = (CollisionEffect*)malloc(sizeof(CollisionEffect));
    if (!effect) return NULL;
    
    effect->type = EFFECT_BALL_SPAWN;
    effect->continuous = continuous;
    effect->params.spawnEffect.position = position;
    effect->params.spawnEffect.radius = radius;
    effect->params.spawnEffect.color = color;
    effect->next = NULL;
    
    return effect;
}

// Add an effect to a list of effects
void addEffectToList(CollisionEffect** head, CollisionEffect* newEffect) {
    if (!newEffect) return;
    newEffect->next = *head;
    *head = newEffect;
}

// Free a linked list of effects
void freeEffectList(CollisionEffect** head) {
    CollisionEffect* current = *head;
    CollisionEffect* next;
    
    while (current != NULL) {
        next = current->next;
        free(current);
        current = next;
    }
    
    *head = NULL;
}

// Apply all applicable effects from both bouncing object and game object during a collision
void applyEffects(BouncingObject* bouncingObj, GameObject* gameObj, bool isOngoingCollision) {
    // Apply effects attached to the bouncing object
    for (CollisionEffect* effect = bouncingObj->onCollisionEffects; effect != NULL; effect = effect->next) {
        // Skip if we're in an ongoing collision and the effect is not continuous
        if (isOngoingCollision && !effect->continuous) continue;
        
        // Apply the effect based on its type
        switch (effect->type) {
            case EFFECT_COLOR_CHANGE:
                bouncingObj->color = effect->params.colorEffect.color;
                break;
                
            case EFFECT_VELOCITY_BOOST:
                bouncingObj->velocity = Vector2Scale(bouncingObj->velocity, effect->params.velocityEffect.factor);
                break;
                
            case EFFECT_VELOCITY_DAMPEN:
                bouncingObj->velocity = Vector2Scale(bouncingObj->velocity, effect->params.velocityEffect.factor);
                break;
                
            case EFFECT_SIZE_CHANGE:
                bouncingObj->radius *= effect->params.sizeEffect.factor;
                // Ensure the radius stays within reasonable bounds
                bouncingObj->radius = Clamp(bouncingObj->radius, 2.0f, 100.0f);
                break;
                
            case EFFECT_SOUND_PLAY:
                if (!isOngoingCollision || effect->continuous) { // Only play sound once for non-continuous
                    IsSoundPlaying(effect->params.soundEffect.sound) ? StopSound(effect->params.soundEffect.sound) : PlaySound(effect->params.soundEffect.sound);
                }
                break;
                
            case EFFECT_BALL_DISAPPEAR:
                // Mark the ball for deletion - actual deletion happens in removeMarkedBouncingObjects
                bouncingObj->markedForDeletion = true;
                // In a more advanced version, we could spawn particles here
                break;
                
            case EFFECT_BALL_SPAWN:
                // This would typically be handled by the game logic, not here
                // The main game loop would check for this effect and spawn new balls
                break;
        }
    }
    
    // Apply effects attached to the game object
    if (gameObj) {
        for (CollisionEffect* effect = gameObj->onCollisionEffects; effect != NULL; effect = effect->next) {
            // Skip if we're in an ongoing collision and the effect is not continuous
            if (isOngoingCollision && !effect->continuous) continue;
            
            // Apply the effect based on its type (same implementations as above)
            switch (effect->type) {
                case EFFECT_COLOR_CHANGE:
                    bouncingObj->color = effect->params.colorEffect.color;
                    break;
                    
                case EFFECT_VELOCITY_BOOST:
                    bouncingObj->velocity = Vector2Scale(bouncingObj->velocity, effect->params.velocityEffect.factor);
                    break;
                    
                case EFFECT_VELOCITY_DAMPEN:
                    bouncingObj->velocity = Vector2Scale(bouncingObj->velocity, effect->params.velocityEffect.factor);
                    break;
                    
                case EFFECT_SIZE_CHANGE:
                    bouncingObj->radius *= effect->params.sizeEffect.factor;
                    // Ensure the radius stays within reasonable bounds
                    bouncingObj->radius = Clamp(bouncingObj->radius, 2.0f, 100.0f);
                    break;
                    
                case EFFECT_SOUND_PLAY:
                    if (!isOngoingCollision || effect->continuous) { // Only play sound once for non-continuous
                        PlaySound(effect->params.soundEffect.sound);
                    }
                    break;
                case EFFECT_BALL_DISAPPEAR:
                case EFFECT_BALL_SPAWN:
                    // TODO
                    break;
            }
        }
    }
}

// --- Add Collision Effects to Objects ---

// Helper function to add collision effects to a GameObject
void addCollisionEffectsToGameObject(GameObject* obj, CollisionEffect* effectsList) {
    obj->onCollisionEffects = effectsList;
}

// Helper function to add collision effects to a BouncingObject
void addCollisionEffectsToBouncingObject(BouncingObject* obj, CollisionEffect* effectsList) {
    obj->onCollisionEffects = effectsList;
}

// Create a GameObject with predefined collision effects
GameObject* createGameObjectWithEffects(GameObject* baseObject, CollisionEffect* effectsList) {
    if (!baseObject) return NULL;
    
    baseObject->onCollisionEffects = effectsList;
    return baseObject;
}

// Count the number of bouncing objects in a list
int Count_BouncingObjects(BouncingObject* head) {
    int count = 0;
    for (BouncingObject* current = head; current != NULL; current = current->next) {
        count++;
    }
    return count;
}

// Count the number of game objects in a list
int Count_GameObjects(GameObject* head) {
    int count = 0;
    for (GameObject* current = head; current != NULL; current = current->next) {
        count++;
    }
    return count;
}