#include "../include/common.h"
#include <stdlib.h> // For malloc, free
#include <stdio.h>  // For debug prints (optional)
#include <math.h>   // For sqrtf, fabsf, fmaxf

// --- Physics Helper Implementations ---

// Closest point on segment AB to point P
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

static bool checkCollisionRectangleObj(GameObject* self, Ball* ball, float dt_step, float* timeOfImpact, Vector2* collisionNormal) {
    ShapeDataRectangle* data = (ShapeDataRectangle*)self->shapeData;
    // Relative velocity of ball with respect to the (potentially moving) object
    Vector2 relBallVel = Vector2Subtract(ball->velocity, self->velocity);

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
    bool collided_overall = false;

    for (int i = 0; i < 4; ++i) {
        float current_segment_toi;
        Vector2 current_segment_normal;
        // Pass ball's current position, its RELATIVE velocity, and segment (considered static in this call)
        if (sweptBallToStaticSegmentCollision(segments[i][0], segments[i][1],
                                              ball->position, relBallVel, ball->radius,
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

    data->width = width; data->height = height; data->color = color;
    obj->type = SHAPE_RECTANGLE;
    obj->position = position;
    obj->velocity = isStatic ? (Vector2){0,0} : velocity;
    obj->shapeData = data;
    obj->isStatic = isStatic;
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

static bool checkCollisionDiamondObj(GameObject* self, Ball* ball, float dt_step, float* timeOfImpact, Vector2* collisionNormal) {
    ShapeDataDiamond* data = (ShapeDataDiamond*)self->shapeData;
    Vector2 relBallVel = Vector2Subtract(ball->velocity, self->velocity);
    Vector2 p = self->position;
    float hw = data->halfWidth; float hh = data->halfHeight;

    Vector2 topV = {p.x, p.y - hh}; Vector2 rightV = {p.x + hw, p.y};
    Vector2 bottomV = {p.x, p.y + hh}; Vector2 leftV = {p.x - hw, p.y};
    Vector2 segments[4][2] = { {topV, rightV}, {rightV, bottomV}, {bottomV, leftV}, {leftV, topV} };
    
    float min_toi = dt_step + EPSILON2;
    bool collided_overall = false;

    for (int i = 0; i < 4; ++i) {
        float current_segment_toi;
        Vector2 current_segment_normal;
        if (sweptBallToStaticSegmentCollision(segments[i][0], segments[i][1],
                                              ball->position, relBallVel, ball->radius,
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

    data->halfWidth = diagWidth / 2.0f; data->halfHeight = diagHeight / 2.0f; data->color = color;
    obj->type = SHAPE_DIAMOND;
    obj->position = position;
    obj->velocity = isStatic ? (Vector2){0,0} : velocity;
    obj->shapeData = data;
    obj->isStatic = isStatic;
    obj->render = renderDiamondObj;
    obj->checkCollision = checkCollisionDiamondObj;
    obj->update = updateGenericMovingObject;
    obj->destroy = destroyGenericShapeData;
    obj->next = NULL;
    return obj;
}