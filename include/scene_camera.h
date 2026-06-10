#pragma once
#include "raylib.h"

namespace PhysicsEngineRendering {

enum class CameraMode { orbital, fps, locked };

struct SceneCamera {
    Camera3D camera;
    CameraMode mode;
    bool input_enabled; // for later w the gui

    // fps camera state
    float yaw;           // radians, 0 = looking down -Z
    float pitch;         // radians, clamped to avoid gimbal flip
    float move_speed;    // units per second
    float sensitivity;   // radians per pixel of mouse delta
    bool cursor_grabbed; // whether we've captured the cursor

    void init(Vector3 position, Vector3 target);
    void update();

    void set_mode(CameraMode new_mode);
    void enable_input();
    void disable_input();

    // focuses the camera on a world point
    void look_at(Vector3 target);

    // forwarding for BeginMode3D / EndMode3D
    void begin() const;
    void end() const;

  private:
    void update_fps();
    void update_orbital();
};

} // namespace PhysicsEngineRendering
