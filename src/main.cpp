#include "raylib.h"

#include "debug_draw.h"
#include "interaction.h"
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

    // static ground plane
    world.add_body(RigidBody::create_static(Shape::make_box(10.0f, 0.1f, 10.0f),
                                            make_vec3(0.0f, -0.1f, 0.0f)));

    // a few starting bodies
    world.add_body(RigidBody::create_dynamic(
        Shape::make_box(0.5f, 0.5f, 0.5f), 1.0f, make_vec3(0.0f, 3.0f, 0.0f)));
    world.add_body(RigidBody::create_dynamic(
        Shape::make_box(0.5f, 0.5f, 0.5f), 1.0f, make_vec3(0.0f, 2.0f, 0.0f)));
    world.add_body(RigidBody::create_dynamic(
        Shape::make_box(0.5f, 0.5f, 0.5f), 1.0f, make_vec3(0.0f, 1.0f, 0.0f)));

    Interaction interaction;
    bool paused = false;

    while (!WindowShouldClose()) {

        // pause toggle
        if (IsKeyPressed(KEY_SPACE))
            paused = !paused;

        // physics step (skipped when paused)
        if (!paused)
            world.step();

        interaction.update(world, camera);

        // camera
        camera.update();

        // render
        BeginDrawing();
        ClearBackground(RAYWHITE);

        camera.begin();
        debug_draw.draw_grid(20, 1.0f);
        debug_draw.draw_bodies(world.bodies);
        interaction.draw_3d(world, camera, debug_draw);
        camera.end();

        // 2D HUD
        interaction.draw_hud(paused);
        DrawFPS(10, 10);

        EndDrawing();
    }

    debug_draw.shutdown();
    CloseWindow();

    return 0;
}
