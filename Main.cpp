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
        {400, 400, 100.0f, RED, 1.5f, 1 / Balls[0].mass,Vector2Zero(), Vector2Zero()}, 
        {100, 100, 30.0f, GREEN, 0.4f, 1 / Balls[1].mass, Vector2Zero(), Vector2Zero()},
        {150, 150, 60.0f, BLUE, 2.0f, 1 / Balls[2].mass, Vector2Zero(), Vector2Zero()},
        {200, 200, 45.0f, ORANGE, 7.0f, 1 / Balls[3].mass, Vector2Zero(), Vector2Zero()}
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
    float elasticity = 0; 
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

            // Negates the velocity at x and y if the object hits a wall. (Basic Collision Detection)
                
                // This is for all other circles
                for(int x = i; x < 4; x++){ // This still needs some fixes o7
                    // add somethign that checks if it's iterating through itself ig
                    float totalRadius = Balls[x].radius + Balls[x].radius;
                    float distanceBetweenCircles = Vector2Distance(Balls[x].position, Balls[x].position);

                    Vector2 relativeVelocity = Vector2Subtract(Balls[x].velocity, Balls[i].velocity);
                    Vector2 collisionNorm = Vector2Subtract(Balls[x].position, Balls[i].position);
                    float approachCheck = Vector2DotProduct(relativeVelocity, collisionNorm);

                    if ((totalRadius >= distanceBetweenCircles) && (approachCheck < 0)){
                        float impulse = -(((1 + elasticity) * Vector2DotProduct(relativeVelocity, collisionNorm))
                        /((Vector2DotProduct(collisionNorm, collisionNorm)) * (Balls[i].inverse_mass + Balls[x].inverse_mass)));

                        Balls[i].velocity = Vector2Subtract(Balls[i].velocity, Vector2Scale(collisionNorm, impulse * Balls[i].inverse_mass));
                        Balls[x].velocity = Vector2Add(Balls[x].velocity, Vector2Scale(collisionNorm, impulse * Balls[x].inverse_mass));
                    }          

                    
                    Vector2 pointOnRect;
                    pointOnRect.x = Clamp(Balls[i].position.x, Walls[x].position.x, Walls[x].position.x + Walls[x].size.x);
                    pointOnRect.y = Clamp(Balls[i].position.y, Walls[x].position.y, Walls[x].position.y + Walls[x].size.y);

                    relativeVelocity = Vector2Subtract(Balls[i].velocity, Walls[x].velocity);
                    collisionNorm = Vector2Subtract(Balls[i].position, pointOnRect);
                    approachCheck = Vector2DotProduct(relativeVelocity, collisionNorm);

                    float distRectCircle = Vector2Distance(pointOnRect, Balls[i].position);
                    
                    if((distRectCircle <= Balls[i].radius) && (approachCheck < 0)){
                        float impulse = -(((1 + elasticity) * Vector2DotProduct(relativeVelocity, collisionNorm))
                        /((Vector2DotProduct(collisionNorm, collisionNorm)) * (Walls[x].inverse_mass + Balls[i].inverse_mass)));
                        Balls[i].velocity = Vector2Add(Balls[i].velocity, Vector2Scale(collisionNorm, impulse * Balls[i].inverse_mass));
                    }
                }

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