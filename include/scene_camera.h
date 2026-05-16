#pragma once
#include "raylib.h"

namespace PhysicsEngineRendering {

enum class CameraMode { orbital, free, locked };

struct SceneCamera {
    Camera3D camera;
    CameraMode mode;
    bool input_enabled; // for later w the gui

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
};

} // namespace PhysicsEngineRendering
