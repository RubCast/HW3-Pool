#include <raylib.h>
#include <raymath.h>
#include <iostream>

const int WINDOW_WIDTH = 800;
const int WINDOW_HEIGHT = 600;
const float FPS = 60;
const float TIMESTEP = 1 / FPS; // Sets the timestep to 1 / FPS. But timestep can be any very small value.
const float FRICTION = 1.0f;

struct Ball {
    Vector2 position;
    float radius;
    Color color;

    float mass;
    float inverse_mass; // A variable for 1 / mass. Used in the calculation for acceleration = sum of forces / mass
    Vector2 acceleration;
    Vector2 velocity;
};

struct Wall {
    Vector2 position;
    Vector2 size;
    Color color;
    
    Vector2 velocity;
    float mass;
    float inverse_mass;
};


int main() {
    
    Ball Balls[4] = {
        {400, 400, 30.0f, RED, 1.5f, 1 / Balls[0].mass,Vector2Zero(), Vector2Zero()}, 
        {100, 100, 30.0f, GREEN, 0.4f, 1 / Balls[1].mass, Vector2Zero(), Vector2Zero()},
        {150, 150, 30.0f, BLUE, 2.0f, 1 / Balls[2].mass, Vector2Zero(), Vector2Zero()},
        {200, 200, 30.0f, ORANGE, 7.0f, 1 / Balls[3].mass, Vector2Zero(), Vector2Zero()}
    };

    Wall Walls[4] = {
        {0, 0, 30.0f, 600.0f, RED, 0.0f, 0.0f, 100.0f, 0.0f},
        {770, 0, 30.0f, 600.0f, RED, 0.0f, 0.0f, 100.0f, 0.0f},
        {0, 0, 800.0f, 30.0f, RED, 0.0f, 0.0f, 100.0f, 0.0f},
        {0, 570, 800.0f, 30.0f, RED, 0.0f, 0.0f, 100.0f, 0.0f}
    };


    InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Physics Demo");

    SetTargetFPS(FPS);

    float accumulator = 0;
    float elasticity = 1; 
    while (!WindowShouldClose()) {
        float delta_time = GetFrameTime();
        Vector2 forces = Vector2Zero(); // every frame set the forces to a 0 vector

        // Adds forces with the magnitude of 100 in the direction given by WASD inputs
        if(IsKeyDown(KEY_W)) {
            forces = Vector2Add(forces, {0, -100});
        }
        if(IsKeyDown(KEY_A)) {
            forces = Vector2Add(forces, {-100, 0});
        }
        if(IsKeyDown(KEY_S)) {
            forces = Vector2Add(forces, {0, 100});
        }
        if(IsKeyDown(KEY_D)) {
            forces = Vector2Add(forces, {100, 0});
        }
        // Does Vector - Scalar multiplication with the sum of all forces and the inverse mass of the ball
        Balls[0].acceleration = Vector2Scale(forces, Balls[0].inverse_mass);
        
        // ------------
        // PHYSICS STEP
        // ------------
        accumulator += delta_time;
        while(accumulator >= TIMESTEP) {
            // ------ SEMI-IMPLICIT EULER INTEGRATION -------
            for(int i = 0; i < 4; i++){
            // Computes for velocity using v(t + dt) = v(t) + (a(t) * dt)
                Balls[i].velocity = Vector2Add(Balls[i].velocity, Vector2Scale(Balls[i].acceleration, TIMESTEP));
            // Reduces the velocity by applying friction using v(n) = v(n - 1) - b / m * v(n - 1) * t
                Balls[i].velocity = Vector2Subtract(Balls[i].velocity, Vector2Scale(Balls[i].velocity, FRICTION * Balls[i].inverse_mass * TIMESTEP));
            // Computes for change in position using x(t + dt) = x(t) + (v(t + dt) * dt)
                Balls[i].position = Vector2Add(Balls[i].position, Vector2Scale(Balls[i].velocity, TIMESTEP));

                // This is for all other circles
                for(int x = 0; x < 4; x++){
                    if(x == i){
                        x++;
                    }

                    float totalRadius = Balls[x].radius + Balls[i].radius;
                    float distanceBetweenCircles = Vector2Distance(Balls[x].position, Balls[i].position);

                    Vector2 relativeVelocity = Vector2Subtract(Balls[x].velocity, Balls[i].velocity);
                    Vector2 collisionNorm = Vector2Subtract(Balls[x].position, Balls[i].position);
                    float approachCheck = Vector2DotProduct(relativeVelocity, collisionNorm);

                    if ((totalRadius >= distanceBetweenCircles) && (approachCheck < 0)){
                        float impulse = -(((1 + elasticity) * Vector2DotProduct(relativeVelocity, collisionNorm))
                        /((Vector2DotProduct(collisionNorm, collisionNorm)) * (Balls[i].inverse_mass + Balls[x].inverse_mass)));

                        Balls[i].velocity = Vector2Subtract(Balls[i].velocity, Vector2Scale(collisionNorm, impulse * Balls[i].inverse_mass));
                        Balls[x].velocity = Vector2Add(Balls[x].velocity, Vector2Scale(collisionNorm, impulse * Balls[x].inverse_mass));
                    } 
                }
                
                // Wall Collisions
                for(int w = 0; w < 4; w++){
                    Vector2 pointOnRect;
                    pointOnRect.x = Clamp(Balls[w].position.x, Walls[i].position.x, Walls[i].position.x + Walls[i].size.x);
                    pointOnRect.y = Clamp(Balls[w].position.y, Walls[i].position.y, Walls[i].position.y + Walls[i].size.y);

                    Vector2 relativeVelocity = Vector2Subtract(Balls[w].velocity, Walls[i].velocity);
                    Vector2 collisionNorm = Vector2Subtract(Balls[w].position, pointOnRect);
                    float approachCheck = Vector2DotProduct(relativeVelocity, collisionNorm);

                    float distRectCircle = Vector2Distance(pointOnRect, Balls[w].position);
                    
                    if((distRectCircle <= Balls[w].radius) && (approachCheck < 0)){
                        float impulse = -(((1 + elasticity) * Vector2DotProduct(relativeVelocity, collisionNorm))
                        /((Vector2DotProduct(collisionNorm, collisionNorm)) * (Walls[i].inverse_mass + Balls[w].inverse_mass)));
                        Balls[w].velocity = Vector2Add(Balls[w].velocity, Vector2Scale(collisionNorm, impulse * Balls[w].inverse_mass));
                    }
                }

                // Holes Collisions

            }
            // Reduces the accumulater with the TIMESTEP
            accumulator -= TIMESTEP;
        }

        BeginDrawing();
        ClearBackground(WHITE);
        for(int i = 0; i < 4; i++){
            DrawCircleV(Balls[i].position, Balls[i].radius, Balls[i].color);
        }
        for(int i = 0; i < 4; i++){
            DrawRectangleV(Walls[i].position, Walls[i].size, Walls[i].color);
        }
        EndDrawing();
    }
    CloseWindow();
    return 0;
}