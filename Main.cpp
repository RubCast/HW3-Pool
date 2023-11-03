#include <raylib.h>
#include <raymath.h>
#include <iostream>

const int WINDOW_WIDTH = 800;
const int WINDOW_HEIGHT = 600;
const float FPS = 60;
const float TIMESTEP = 1 / FPS; // Sets the timestep to 1 / FPS. But timestep can be any very small value.
const float FRICTION = 50.0f;

struct Circle {
    Vector2 position;
    float radius;
    bool is_cue;
    Color color;
    float mass;
    float inverse_mass; // A variable for 1 / mass. Used in the calculation for acceleration = sum of forces / mass
    Vector2 acceleration;
    Vector2 velocity;
};

Circle Balls[5] = {
    {400, 400, 30.0f, true, WHITE, 30.0f, 1 / Balls[0].mass,Vector2Zero(), Vector2Zero()}, 
    {100, 100, 30.0f, false, RED, 30.0f, 1 / Balls[1].mass, Vector2Zero(), Vector2Zero()},
    {150, 150, 30.0f, false, ORANGE, 30.0f, 1 / Balls[2].mass, Vector2Zero(), Vector2Zero()},
    {200, 200, 30.0f, false, YELLOW, 30.0f, 1 / Balls[3].mass, Vector2Zero(), Vector2Zero()},
    {200, 400, 30.0f, false, BLUE, 30.0f, 1 / Balls[4].mass, Vector2Zero(), Vector2Zero()}
};

Circle Holes[4] = {
    {15, 15, 50.0f, false, BLACK, 100.0f, 0.0f, Vector2Zero(), Vector2Zero()},
    {785, 15, 50.0f, false, BLACK, 100.0f, 0.0f, Vector2Zero(), Vector2Zero()},
    {15, 585, 50.0f, false, BLACK, 100.0f, 0.0f, Vector2Zero(), Vector2Zero()},
    {785, 585, 50.0f, false, BLACK, 100.0f, 0.0f, Vector2Zero(), Vector2Zero()}
};

struct Wall {
    Vector2 position;
    Vector2 size;
    Color color;
    
    Vector2 velocity;
    float mass;
    float inverse_mass;
};

Wall Walls[4] = {
    {0, 0, 30.0f, 600.0f, RED, 0.0f, 0.0f, 100.0f, 0.0f},
    {770, 0, 30.0f, 600.0f, RED, 0.0f, 0.0f, 100.0f, 0.0f},
    {0, 0, 800.0f, 30.0f, RED, 0.0f, 0.0f, 100.0f, 0.0f},
    {0, 570, 800.0f, 30.0f, RED, 0.0f, 0.0f, 100.0f, 0.0f}
};

struct Pole {
    Vector2 position;
    Color color;
    Vector2 click_origin;
    Vector2 difference;

}Pole;

int main() {
    bool is_moving;
    Pole.color = YELLOW;
    InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Pool Table");

    SetTargetFPS(FPS);

    float accumulator = 0;
    float elasticity = 1; 
    while (!WindowShouldClose()) {
        float delta_time = GetFrameTime();
        if(IsKeyPressed(KEY_SPACE)){
            Balls[0] = {400, 400, 30.0f, true, WHITE, 30.0f, 1 / Balls[0].mass,Vector2Zero(), Vector2Zero()};
            Balls[1] = {100, 100, 30.0f, false, RED, 30.0f, 1 / Balls[1].mass, Vector2Zero(), Vector2Zero()};
            Balls[2] = {150, 150, 30.0f, false, ORANGE, 30.0f, 1 / Balls[2].mass, Vector2Zero(), Vector2Zero()};
            Balls[3] = {200, 200, 30.0f, false, YELLOW, 30.0f, 1 / Balls[3].mass, Vector2Zero(), Vector2Zero()};
            Balls[4] = {200, 400, 30.0f, false, BLUE, 30.0f, 1 / Balls[4].mass, Vector2Zero(), Vector2Zero()};
        }
        if(is_moving == false){
            if(IsMouseButtonDown(MOUSE_BUTTON_LEFT)){
                Pole.click_origin = Balls[0].position;
                Pole.position = GetMousePosition();
            }
        
            if(IsMouseButtonReleased(MOUSE_BUTTON_LEFT)){
                is_moving = true;
                Pole.difference = Vector2Negate(Vector2Subtract(Pole.position, Pole.click_origin));
                Balls[0].velocity = Vector2Add(Balls[0].velocity, Vector2Scale(Pole.difference, .10f));
                Pole.click_origin = Vector2Zero();
                Pole.position = Vector2Zero();
            }
        }
        
    	// ------------
        // PHYSICS STEP
        // ------------
        else if(is_moving == true){       
            // Does Vector - Scalar multiplication with the sum of all forces and the inverse mass of the ball
            Balls[0].position = Vector2Add(Balls[0].position, Balls[0].velocity);
            Balls[0].acceleration = Vector2Scale(Balls[0].velocity, 0.95f);
            
            accumulator += delta_time;
            while(accumulator >= TIMESTEP) {
                // ------ SEMI-IMPLICIT EULER INTEGRATION -------
                for(int i = 0; i < 5; i++){
                // Computes for velocity using v(t + dt) = v(t) + (a(t) * dt)
                    Balls[i].velocity = Vector2Add(Balls[i].velocity, Vector2Scale(Balls[i].acceleration, TIMESTEP));
                // Reduces the velocity by applying friction using v(n) = v(n - 1) - b / m * v(n - 1) * t
                    Balls[i].velocity = Vector2Subtract(Balls[i].velocity, Vector2Scale(Balls[i].velocity, FRICTION * Balls[i].inverse_mass * TIMESTEP));
                    if(Vector2Length(Balls[i].velocity) < 0.1f){
                        Balls[i].velocity = {0, 0};
                    }

                // Computes for change in position using x(t + dt) = x(t) + (v(t + dt) * dt)
                    Balls[i].position = Vector2Add(Balls[i].position, Vector2Scale(Balls[i].velocity, TIMESTEP));
                    
                    // Holes Collisions
                    for(int x = 0; x < 5; x++){
                        float totalRadius = Balls[x].radius + Holes[i].radius;
                        float distanceBetweenCircles = Vector2Distance(Balls[x].position, Holes[i].position);

                        Vector2 relativeVelocity = Vector2Subtract(Balls[x].velocity, Holes[i].velocity);
                        Vector2 collisionNorm = Vector2Subtract(Balls[x].position, Holes[i].position);
                        float approachCheck = Vector2DotProduct(relativeVelocity, collisionNorm);

                        if ((totalRadius >= distanceBetweenCircles) && (approachCheck < 0)){
                            float impulse = -(((1 + elasticity) * Vector2DotProduct(relativeVelocity, collisionNorm))
                            /((Vector2DotProduct(collisionNorm, collisionNorm)) * (Holes[i].inverse_mass + Balls[x].inverse_mass)));
                            if(Balls[x].is_cue == true){
                                Balls[x].position.x = 400;
                                Balls[x].position.y = 400;
                                Balls[x].velocity = Vector2Zero();
                            }
                            else{
                                Balls[x].radius = 0.0f;
                                Balls[x].position = Vector2Zero();
                            }
                        } 
                    }
                    
                    // This is for all other circles
                    for(int x = 0; x < 5; x++){
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
                    for(int w = 0; w < 5; w++){
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
                }
                bool moving = false;
                for(int i = 0; i < 5; i++){
                    if((Balls[i].velocity.x != 0) && (Balls[i].velocity.y != 0)){
                        moving = true;
                        i = 5;
                    }
                }
                // Reduces the accumulater with the TIMESTEP
                accumulator -= TIMESTEP;
                is_moving = moving;
            }
        }
        BeginDrawing();
        ClearBackground(GREEN);
        for(int i = 0; i < 5; i++){
            DrawCircleV(Balls[i].position, Balls[i].radius, Balls[i].color);
        }
        for(int i = 0; i < 4; i++){
            DrawRectangleV(Walls[i].position, Walls[i].size, Walls[i].color);
        }
        for(int i = 0; i < 4; i++){
            DrawCircleV(Holes[i].position, Holes[i].radius, Holes[i].color);
        }
        DrawLineEx(Pole.click_origin, Pole.position, 5.0f, Pole.color);
        EndDrawing();
    }
    CloseWindow();
    return 0;
}