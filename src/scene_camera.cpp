#include "scene_camera.h"
#include "raylib.h"

namespace PhysicsEngineRendering {

void SceneCamera::init(Vector3 position, Vector3 target) {
    this->camera = {};
    this->camera.position = position;
    this->camera.target = target;
    this->camera.up = {0.0f, 1.0f, 0.0f};
    this->camera.fovy = 45.0f;
    this->camera.projection = CAMERA_PERSPECTIVE;

    this->mode = CameraMode::orbital;
    this->input_enabled = true;
}

void SceneCamera::update() {
    if (!this->input_enabled)
        return;

    switch (this->mode) {
    case CameraMode::orbital:
        UpdateCamera(&this->camera, CAMERA_ORBITAL);
        break;
    case CameraMode::free:
        UpdateCamera(&this->camera, CAMERA_FREE);
        break;
    case CameraMode::locked:
        break;
    }
}

void SceneCamera::set_mode(CameraMode new_mode) { this->mode = new_mode; }
void SceneCamera::enable_input() { this->input_enabled = true; }
void SceneCamera::disable_input() { this->input_enabled = false; }
void SceneCamera::look_at(Vector3 target) { this->camera.target = target; }
void SceneCamera::begin() const { BeginMode3D(this->camera); }
void SceneCamera::end() const { EndMode3D(); }
} // namespace PhysicsEngineRendering
