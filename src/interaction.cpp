#include "interaction.h"
#include "math_utils.h"
#include "raylib.h"
#include <cmath>
#include <cstdio>

namespace PhysicsEngine {

static Vec3 from_rl(Vector3 v) { return make_vec3(v.x, v.y, v.z); }
static Vector3 to_rl(const Vec3 &v) {
    return {v.data_[0], v.data_[1], v.data_[2]};
}

static Constraints::DistanceType force_tool_to_type(ForceTool t) {
    switch (t) {
    case ForceTool::Spring:
        return Constraints::DistanceType::Spring;
    case ForceTool::Damper:
        return Constraints::DistanceType::Damper;
    case ForceTool::Rod:
        return Constraints::DistanceType::Rod;
    }
    return Constraints::DistanceType::Rod;
}

// ============================================================
// key handling
// ============================================================

void Interaction::handle_keys(World &world) {
    if (IsKeyPressed(KEY_B)) {
        mode = (mode == InteractionMode::Body) ? InteractionMode::None
                                               : InteractionMode::Body;
        awaiting_second = false;
        dragging = false;
    }
    if (IsKeyPressed(KEY_F)) {
        mode = (mode == InteractionMode::Force) ? InteractionMode::None
                                                : InteractionMode::Force;
        awaiting_second = false;
        dragging = false;
    }
    if (IsKeyPressed(KEY_G)) {
        mode = (mode == InteractionMode::Drag) ? InteractionMode::None
                                               : InteractionMode::Drag;
        awaiting_second = false;
        dragging = false;
    }

    if (IsKeyPressed(KEY_ONE)) {
        if (mode == InteractionMode::Body)
            body_tool = BodyTool::Box;
        if (mode == InteractionMode::Force)
            force_tool = ForceTool::Spring;
    }
    if (IsKeyPressed(KEY_TWO)) {
        if (mode == InteractionMode::Body)
            body_tool = BodyTool::Sphere;
        if (mode == InteractionMode::Force)
            force_tool = ForceTool::Damper;
    }
    if (IsKeyPressed(KEY_THREE)) {
        if (mode == InteractionMode::Force)
            force_tool = ForceTool::Rod;
    }

    if (IsKeyPressed(KEY_S) && mode == InteractionMode::Body) {
        place_static = !place_static;
    }

    if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT) && awaiting_second) {
        awaiting_second = false;
    }

    if (IsKeyPressed(KEY_BACKSPACE)) {
        std::cout << hover.body_idx << std::endl;
        if (hover.body_idx != -1 && hover.body_idx != 0) {
            std::cout << "delete" << std::endl;
            world.bodies.erase(world.bodies.begin() + hover.body_idx);
        }
    }
}

// ============================================================
// body placement
// ============================================================

void Interaction::handle_body_click(World &world) {
    float half = 0.5f;
    float mass = 1.0f;

    if (!hover.hit)
        return;

    Vec3 spawn_pos = hover.world_point + hover.normal * half;

    RigidBody body;
    if (body_tool == BodyTool::Box) {
        Shape s = Shape::make_box(half, half, half);
        body = place_static ? RigidBody::create_static(s, spawn_pos)
                            : RigidBody::create_dynamic(s, mass, spawn_pos);
    } else {
        Shape s = Shape::make_sphere(half);
        body = place_static ? RigidBody::create_static(s, spawn_pos)
                            : RigidBody::create_dynamic(s, mass, spawn_pos);
    }

    world.add_body(body);
}

// ============================================================
// constraint placement (two-click)
// ============================================================

void Interaction::handle_force_click(World &world) {
    if (!hover.hit) {
        if (awaiting_second)
            awaiting_second = false;
        return;
    }

    if (!awaiting_second) {
        first_body_idx = hover.body_idx;
        first_local_anchor = hover.local_point;
        first_world_point = hover.world_point;
        awaiting_second = true;
    } else {
        if (hover.body_idx == first_body_idx)
            return;

        if (world.bodies[first_body_idx].is_static() &&
            world.bodies[hover.body_idx].is_static())
            return;

        Vec3 p1 = first_world_point;
        Vec3 p2 = hover.world_point;
        float rest = length(p2 - p1);

        auto constraint = std::make_unique<Constraints::DistanceConstraint>(
            first_body_idx, hover.body_idx, first_local_anchor,
            hover.local_point, rest, force_tool_to_type(force_tool));

        world.constraints.push_back(std::move(constraint));
        awaiting_second = false;
    }
}

// ============================================================
// drag mode
// ============================================================

void Interaction::handle_drag(
    World &world, const PhysicsEngineRendering::SceneCamera &camera) {

    float cx = GetScreenWidth() * 0.5f;
    float cy = GetScreenHeight() * 0.5f;
    Ray ray = GetMouseRay({cx, cy}, camera.camera);
    Vec3 origin = from_rl(ray.position);
    Vec3 dir = from_rl(ray.direction);

    // start drag on press
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        if (hover.hit && !world.bodies[hover.body_idx].is_static()) {
            dragging = true;
            drag_body_idx = hover.body_idx;
            drag_local_anchor = hover.local_point;
            drag_grab_dist = hover.t;
        }
    }

    // stop drag on release
    if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
        dragging = false;
    }

    if (!dragging)
        return;

    // bounds check in case bodies were removed
    if (drag_body_idx >= world.bodies.size()) {
        dragging = false;
        return;
    }

    // compute target: point along the ray at the original grab distance
    float dir_len = length(dir);
    Vec3 norm_dir = (dir_len > 1e-6f) ? dir * (1.0f / dir_len) : dir;
    drag_target = origin + norm_dir * drag_grab_dist;

    // compute anchor world position
    RigidBody &body = world.bodies[drag_body_idx];
    Vec3 anchor_world =
        body.position + body.rotation_matrix() * drag_local_anchor;

    // spring force: F = -k * (anchor - target) - c * v
    Vec3 diff = anchor_world - drag_target;
    Vec3 spring_force =
        diff * (-drag_stiffness) + body.linear_velocity * (-drag_damping);

    // apply as velocity change (runs after step, persists to next frame)
    float dt = world.settings.dt;
    body.linear_velocity += spring_force * body.inv_mass * dt;

    // torque from off-centre force application
    Vec3 r = anchor_world - body.position;
    Vec3 torque = cross(r, spring_force);
    body.angular_velocity += body.inv_inertia_world * torque * dt;
}

// ============================================================
// main update
// ============================================================

void Interaction::update(World &world,
                         const PhysicsEngineRendering::SceneCamera &camera) {

    handle_keys(world);
    if (mode == InteractionMode::None)
        return;
    if (!camera.cursor_grabbed)
        return;

    float cx = GetScreenWidth() * 0.5f;
    float cy = GetScreenHeight() * 0.5f;
    Ray ray = GetMouseRay({cx, cy}, camera.camera);
    Vec3 origin = from_rl(ray.position);
    Vec3 dir = from_rl(ray.direction);

    hover = raycast(origin, dir, world.bodies);

    if (mode == InteractionMode::Drag) {
        handle_drag(world, camera);
        return;
    }

    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        if (mode == InteractionMode::Body)
            handle_body_click(world);
        else if (mode == InteractionMode::Force)
            handle_force_click(world);
    }
}

// ============================================================
// 3D drawing
// ============================================================

void Interaction::draw_spring_line(
    const Vec3 &a, const Vec3 &b, Color col,
    const PhysicsEngineRendering::DebugDraw &dd) const {
    Vec3 delta = b - a;
    float len = length(delta);
    if (len < 1e-4f) {
        dd.draw_point(a, col);
        return;
    }
    Vec3 dir = delta * (1.0f / len);

    Vec3 ref = (std::abs(dir.data_[1]) < 0.9f) ? make_vec3(0, 1, 0)
                                               : make_vec3(1, 0, 0);
    Vec3 perp = normalise(cross(dir, ref));

    int segments = 14;
    float amp = 0.08f;
    Vec3 prev = a;

    for (int i = 1; i <= segments; i++) {
        float t = (float)i / segments;
        Vec3 on_line = a + delta * t;

        float offset = 0.0f;
        if (i > 0 && i < segments)
            offset = (i % 2 == 0) ? amp : -amp;

        Vec3 pt = on_line + perp * offset;
        dd.draw_line(prev, pt, col);
        prev = pt;
    }
}

void Interaction::draw_constraints(
    const World &world, const PhysicsEngineRendering::DebugDraw &dd) const {
    for (const auto &c : world.constraints) {
        auto *dc =
            dynamic_cast<const Constraints::DistanceConstraint *>(c.get());
        if (!dc)
            continue;

        Vec3 p1, p2;
        dc->endpoints(world.bodies, p1, p2);
        Color col = dc->colour();

        dd.draw_point(p1, col);
        dd.draw_point(p2, col);

        switch (dc->type) {
        case Constraints::DistanceType::Spring:
            draw_spring_line(p1, p2, col, dd);
            break;
        case Constraints::DistanceType::Damper: {
            Vec3 d = p2 - p1;
            int dashes = 8;
            for (int i = 0; i < dashes; i++) {
                float t0 = (float)i / dashes;
                float t1 = (float)(i + 1) / dashes;
                float mid = (t0 + t1) * 0.5f;
                dd.draw_line(p1 + d * t0, p1 + d * mid, col);
            }
            break;
        }
        case Constraints::DistanceType::Rod:
            dd.draw_line(p1, p2, col);
            break;
        }
    }
}

void Interaction::draw_preview(
    const PhysicsEngineRendering::DebugDraw &dd) const {
    if (hover.hit && mode != InteractionMode::None) {
        dd.draw_point(hover.world_point, GREEN);
    }

    // body placement ghost
    if (mode == InteractionMode::Body && hover.hit) {
        float half = 0.5f;
        Vec3 preview_pos = hover.world_point + hover.normal * half;
        if (body_tool == BodyTool::Box) {
            DrawCubeWires(to_rl(preview_pos), 1.0f, 1.0f, 1.0f,
                          {0, 255, 0, 150});
        } else {
            DrawSphereWires(to_rl(preview_pos), half, 8, 8, {0, 255, 0, 150});
        }
    }

    // force constraint preview
    if (mode == InteractionMode::Force && awaiting_second) {
        dd.draw_point(first_world_point, YELLOW);
        Vec3 end_point = hover.hit ? hover.world_point : first_world_point;
        Color preview_col = {255, 255, 0, 180};
        switch (force_tool) {
        case ForceTool::Spring:
            draw_spring_line(first_world_point, end_point, preview_col, dd);
            break;
        case ForceTool::Damper:
        case ForceTool::Rod:
            dd.draw_line(first_world_point, end_point, preview_col);
            break;
        }
        if (hover.hit)
            dd.draw_point(end_point, GREEN);
    }

    // drag spring visualisation
    if (mode == InteractionMode::Drag && dragging) {
        dd.draw_point(drag_target, {255, 100, 100, 255});
        // can't easily get anchor_world here without bodies, so we draw
        // from the drag_target; the 3d method handles the full line
    }
}

void Interaction::draw_3d(
    const World &world, const PhysicsEngineRendering::SceneCamera & /*camera*/,
    const PhysicsEngineRendering::DebugDraw &dd) const {
    draw_constraints(world, dd);
    draw_preview(dd);

    // drag spring line (needs body data)
    if (mode == InteractionMode::Drag && dragging &&
        drag_body_idx < world.bodies.size()) {
        const RigidBody &body = world.bodies[drag_body_idx];
        Vec3 anchor_world =
            body.position + body.rotation_matrix() * drag_local_anchor;

        draw_spring_line(anchor_world, drag_target, {255, 100, 100, 255}, dd);
        dd.draw_point(anchor_world, RED);
        dd.draw_point(drag_target, {255, 100, 100, 200});
    }
}

// ============================================================
// 2D HUD
// ============================================================

void Interaction::draw_crosshair() const {
    int cx = GetScreenWidth() / 2;
    int cy = GetScreenHeight() / 2;
    int size = 8;
    Color col = {200, 200, 200, 180};
    DrawLine(cx - size, cy, cx + size, cy, col);
    DrawLine(cx, cy - size, cx, cy + size, col);
}

void Interaction::draw_hud(bool paused) const {
    draw_crosshair();

    int y = 30;
    const int line_h = 20;

    if (paused) {
        DrawText("PAUSED", GetScreenWidth() / 2 - 40, 10, 20, RED);
    }

    if (mode == InteractionMode::None) {
        DrawText("[B] Body  [F] Force  [G] Grab  [Space] Pause", 10, y, 16,
                 GRAY);
        return;
    }

    if (mode == InteractionMode::Body) {
        const char *tool = (body_tool == BodyTool::Box) ? "Box" : "Sphere";
        const char *stat = place_static ? "ON" : "OFF";
        char buf[128];
        snprintf(buf, sizeof(buf),
                 "BODY: [1] Box  [2] Sphere  |  [S] Static: %s", stat);
        DrawText(buf, 10, y, 16, ORANGE);
        y += line_h;
        snprintf(buf, sizeof(buf), "Tool: %s  |  Click surface to place", tool);
        DrawText(buf, 10, y, 16, ORANGE);
    }

    if (mode == InteractionMode::Force) {
        const char *tool = "?";
        switch (force_tool) {
        case ForceTool::Spring:
            tool = "Spring";
            break;
        case ForceTool::Damper:
            tool = "Damper";
            break;
        case ForceTool::Rod:
            tool = "Rod";
            break;
        }
        char buf[128];
        snprintf(buf, sizeof(buf), "FORCE: [1] Spring  [2] Damper  [3] Rod");
        DrawText(buf, 10, y, 16, SKYBLUE);
        y += line_h;
        if (awaiting_second) {
            snprintf(buf, sizeof(buf),
                     "Tool: %s  |  Click second body (right-click cancel)",
                     tool);
            DrawText(buf, 10, y, 16, YELLOW);
        } else {
            snprintf(buf, sizeof(buf), "Tool: %s  |  Click first body", tool);
            DrawText(buf, 10, y, 16, SKYBLUE);
        }
    }

    if (mode == InteractionMode::Drag) {
        DrawText("GRAB: Hold left-click on a body to drag", 10, y, 16,
                 {255, 100, 100, 255});
        if (dragging) {
            DrawText("Dragging...", 10, y + line_h, 16, RED);
        }
    }
}

} // namespace PhysicsEngine
