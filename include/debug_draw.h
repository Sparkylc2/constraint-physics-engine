#pragma once

#include "raylib.h"
#include "rigid_body.h"
#include <vector>

namespace PhysicsEngineRendering {

struct DebugDraw {
    // pre-built unit meshes, make em once
    Model unit_cube;
    Model unit_sphere;

    // called after InitWindow
    void init();

    // called before CloseWindow
    void shutdown();

    void draw_body(const PhysicsEngine::RigidBody &body, Color colour) const;
    void draw_bodies(const std::vector<PhysicsEngine::RigidBody> &bodies) const;

    // debug visualisation helpers
    void draw_point(const PhysicsEngine::Vec3 &p, Color colour) const;
    void draw_line(const PhysicsEngine::Vec3 &a, const PhysicsEngine::Vec3 &b,
                   Color colour) const;
    void draw_grid(int slices, float spacing) const;
};
} // namespace PhysicsEngineRendering
