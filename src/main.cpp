#include "raylib.h"

#include "debug_draw.h"
#include "scene_camera.h"
#include "world.h"

using namespace PhysicsEngine;
using namespace PhysicsEngineRendering;

int main() {
    const int screen_width = 1280;
    const int screen_height = 720;

    InitWindow(screen_width, screen_height, "physics engine");
    SetTargetFPS(60);

    DebugDraw debug_draw;
    debug_draw.init();

    SceneCamera camera;
    camera.init({10.0f, 10.0f, 10.0f}, {0.0f, 0.0f, 0.0f});

    World world(1.0f / 60.0f, make_vec3(0.0f, -9.81f, 0.0f));

    // static ground plane (just a big flat box for now)
    world.add_body(RigidBody::create_static(Shape::make_box(10.0f, 0.1f, 10.0f),
                                            make_vec3(0.0f, -0.1f, 0.0f)));

    // dynamic box sitting above ground
    auto t = RigidBody::create_dynamic(Shape::make_box(0.5f, 0.5f, 0.5f), 1.0f,
                                       make_vec3(0.0f, 3.0f, 0.0f));
    t.has_gravity = false;
    world.add_body(t);
    // world.add_body(RigidBody::create_dynamic(
    //     Shape::make_box(0.5f, 0.5f, 0.5f), 1.0f, make_vec3(0.0f, 3.0f,
    //     0.0f)));

    world.add_body(RigidBody::create_dynamic(
        Shape::make_box(0.5f, 0.5f, 0.5f), 1.0f, make_vec3(0.0f, 2.0f, 0.0f)));
    world.add_body(RigidBody::create_dynamic(
        Shape::make_box(0.5f, 0.5f, 0.5f), 1.0f, make_vec3(0.0f, 1.0f, 0.0f)));

    world.add_body(RigidBody::create_dynamic(Shape::make_sphere(0.5f), 1.0f,
                                             make_vec3(1.0f, 3.0f, 0.0f)));
    world.add_body(RigidBody::create_dynamic(Shape::make_sphere(0.5f), 1.0f,
                                             make_vec3(-1.0f, 3.0f, 0.0f)));
    world.add_body(RigidBody::create_dynamic(Shape::make_sphere(0.5f), 1.0f,
                                             make_vec3(-1.0f, 4.2f, 0.0f)));
    while (!WindowShouldClose()) {

        // --- simulation step ---
        world.step();

        // --- render ---
        camera.update();

        BeginDrawing();
        ClearBackground(RAYWHITE);

        camera.begin();
        debug_draw.draw_grid(20, 1.0f);
        debug_draw.draw_bodies(world.bodies);
        camera.end();

        DrawFPS(10, 10);
        EndDrawing();
    }

    debug_draw.shutdown();
    CloseWindow();

    return 0;
}
