#include "raylib.h"

#include "debug_draw.h"
#include "rigid_body.h"
#include "scene_camera.h"
#include "shape.h"
#include "vec3.h"

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

    // test bodies
    std::vector<RigidBody> bodies;

    // static ground plane (just a big flat box for now)
    bodies.push_back(RigidBody::create_static(
        Shape::make_box(10.0f, 0.1f, 10.0f), make_vec3(0.0f, -0.1f, 0.0f)));

    // dynamic box sitting above ground
    bodies.push_back(RigidBody::create_dynamic(
        Shape::make_box(0.5f, 0.5f, 0.5f), 1.0f, make_vec3(0.0f, 3.0f, 0.0f)));

    const float dt = 1.0f / 60.0f;
    const Vec3 gravity = make_vec3(0.0f, -9.81f, 0.0f);

    while (!WindowShouldClose()) {
        for (auto &body : bodies) {
            if (body.is_static())
                continue;

            body.clear_accumulators();
            body.apply_force(gravity * body.mass);

            // velocity update: v += (F/m) * dt
            body.linear_velocity =
                body.linear_velocity +
                make_vec3(body.force.data_[0] * body.inv_mass,
                          body.force.data_[1] * body.inv_mass,
                          body.force.data_[2] * body.inv_mass) *
                    dt;

            body.integrate_position(dt);
        }

        // --- render ---
        camera.update();

        BeginDrawing();
        ClearBackground(RAYWHITE);

        camera.begin();
        debug_draw.draw_grid(20, 1.0f);
        debug_draw.draw_bodies(bodies);
        camera.end();

        DrawFPS(10, 10);
        EndDrawing();
    }

    debug_draw.shutdown();
    CloseWindow();

    return 0;
}
