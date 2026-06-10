#include "scene_camera.h"
#include "raylib.h"
#include <cmath>

namespace PhysicsEngineRendering {

void SceneCamera::init(Vector3 position, Vector3 target) {
    this->camera = {};
    this->camera.position = position;
    this->camera.target = target;
    this->camera.up = {0.0f, 1.0f, 0.0f};
    this->camera.fovy = 45.0f;
    this->camera.projection = CAMERA_PERSPECTIVE;

    this->mode = CameraMode::fps;
    this->input_enabled = true;
    this->move_speed = 8.0f;
    this->sensitivity = 0.003f;
    this->cursor_grabbed = false;

    float dx = target.x - position.x;
    float dy = target.y - position.y;
    float dz = target.z - position.z;
    float horiz = std::sqrt(dx * dx + dz * dz);

    this->yaw = std::atan2(dx, dz);
    this->pitch = std::atan2(dy, horiz);
}

void SceneCamera::update() {
    if (!this->input_enabled)
        return;

    switch (this->mode) {
    case CameraMode::fps:
        update_fps();
        break;
    case CameraMode::orbital:
        update_orbital();
        break;
    case CameraMode::locked:
        break;
    }
}

void SceneCamera::update_fps() {
    if (!this->cursor_grabbed) {
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            DisableCursor();
            this->cursor_grabbed = true;
        }
        return;
    }

    if (IsKeyPressed(KEY_ESCAPE)) {
        EnableCursor();
        this->cursor_grabbed = false;
        return;
    }

    float dt = GetFrameTime();

    // mouse look
    Vector2 delta = GetMouseDelta();
    this->yaw -= delta.x * this->sensitivity;
    this->pitch -= delta.y * this->sensitivity;

    // clamps pitch
    const float max_pitch = 1.5f; // ~86 degrees
    if (this->pitch > max_pitch)
        this->pitch = max_pitch;
    if (this->pitch < -max_pitch)
        this->pitch = -max_pitch;

    // todo: move to a matrix based system
    // with local coords and rotation/translation matrices
    // forward/right vectors
    float cos_p = std::cos(this->pitch);
    float sin_p = std::sin(this->pitch);
    float cos_y = std::cos(this->yaw);
    float sin_y = std::sin(this->yaw);

    // forward: the direction the camera is looking
    Vector3 forward = {sin_y * cos_p, sin_p, cos_y * cos_p};

    // right: perpendicular to forward on the XZ plane
    Vector3 right = {-cos_y, 0.0f, sin_y};

    // up: world up for vertical movement
    Vector3 up = {0.0f, 1.0f, 0.0f};

    float speed = this->move_speed * dt;
    if (IsKeyDown(KEY_LEFT_SHIFT))
        speed *= 2.5f;

    Vector3 &pos = this->camera.position;

    if (IsKeyDown(KEY_W)) {
        pos.x += forward.x * speed;
        pos.y += forward.y * speed;
        pos.z += forward.z * speed;
    }
    if (IsKeyDown(KEY_S)) {
        pos.x -= forward.x * speed;
        pos.y -= forward.y * speed;
        pos.z -= forward.z * speed;
    }
    if (IsKeyDown(KEY_A)) {
        pos.x -= right.x * speed;
        pos.z -= right.z * speed;
    }
    if (IsKeyDown(KEY_D)) {
        pos.x += right.x * speed;
        pos.z += right.z * speed;
    }
    if (IsKeyDown(KEY_SPACE)) {
        pos.y += speed;
    }
    if (IsKeyDown(KEY_LEFT_CONTROL)) {
        pos.y -= speed;
    }

    this->camera.target = {pos.x + forward.x, pos.y + forward.y,
                           pos.z + forward.z};
}

void SceneCamera::update_orbital() {
    UpdateCamera(&this->camera, CAMERA_ORBITAL);
}

void SceneCamera::set_mode(CameraMode new_mode) {
    if (this->mode == CameraMode::fps && new_mode != CameraMode::fps) {
        if (this->cursor_grabbed) {
            EnableCursor();
            this->cursor_grabbed = false;
        }
    }
    this->mode = new_mode;
}

void SceneCamera::enable_input() { this->input_enabled = true; }
void SceneCamera::disable_input() { this->input_enabled = false; }

void SceneCamera::look_at(Vector3 target) {
    this->camera.target = target;

    float dx = target.x - this->camera.position.x;
    float dy = target.y - this->camera.position.y;
    float dz = target.z - this->camera.position.z;
    float horiz = std::sqrt(dx * dx + dz * dz);
    this->yaw = std::atan2(dx, dz);
    this->pitch = std::atan2(dy, horiz);
}

void SceneCamera::begin() const { BeginMode3D(this->camera); }
void SceneCamera::end() const { EndMode3D(); }

} // namespace PhysicsEngineRendering
