#pragma once
#include "debug_draw.h"
#include "distance_constraint.h"
#include "raycast.h"
#include "scene_camera.h"
#include "world.h"

namespace PhysicsEngine {

enum class InteractionMode { None, Body, Force, Drag };
enum class BodyTool { Box = 1, Sphere = 2 };
enum class ForceTool { Spring = 1, Damper = 2, Rod = 3 };

struct Interaction {
    InteractionMode mode = InteractionMode::None;
    BodyTool body_tool = BodyTool::Box;
    ForceTool force_tool = ForceTool::Spring;
    bool place_static = false;

    // two-click constraint placement
    bool awaiting_second = false;
    std::size_t first_body_idx = 0;
    Vec3 first_local_anchor = Vec3::zeros();
    Vec3 first_world_point = Vec3::zeros();

    // drag mode state
    bool dragging = false;
    std::size_t drag_body_idx = 0;
    Vec3 drag_local_anchor = Vec3::zeros();
    float drag_grab_dist = 5.0f;
    Vec3 drag_target = Vec3::zeros();
    float drag_stiffness = 30.0f;
    float drag_damping = 8.0f;

    // live hover
    RayHit hover;

    void update(World &world,
                const PhysicsEngineRendering::SceneCamera &camera);

    void draw_3d(const World &world,
                 const PhysicsEngineRendering::SceneCamera &camera,
                 const PhysicsEngineRendering::DebugDraw &dd) const;

    void draw_hud(bool paused) const;

  private:
    void handle_keys(World &world);
    void handle_body_click(World &world);
    void handle_force_click(World &world);
    void handle_drag(World &world,
                     const PhysicsEngineRendering::SceneCamera &camera);

    void draw_constraints(const World &world,
                          const PhysicsEngineRendering::DebugDraw &dd) const;
    void draw_preview(const PhysicsEngineRendering::DebugDraw &dd) const;
    void draw_crosshair() const;

    void draw_spring_line(const Vec3 &a, const Vec3 &b, Color col,
                          const PhysicsEngineRendering::DebugDraw &dd) const;
};

} // namespace PhysicsEngine
