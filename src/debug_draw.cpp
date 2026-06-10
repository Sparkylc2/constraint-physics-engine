#include "debug_draw.h"
#include "mat.h"
#include "math_utils.h"
#include "rlgl.h"

namespace PhysicsEngineRendering {

// builds a raylib Matrix from quaternion + position + scale
// avoids depending on RigidBody::rotation_matrix() so rendering works
// independently of physics implementation
static Matrix build_transform(const PhysicsEngine::Quaternion &q,
                              const PhysicsEngine::Vec3 &pos, float sx,
                              float sy, float sz) {
    PhysicsEngine::Quaternion rot_q =
        PhysicsEngine::MathUtils::approx_equal(q.norm(), 1.0f) ? q
                                                               : q.normalised();

    // standard formula
    const float w = rot_q.w, x = rot_q.x, y = rot_q.y, z = rot_q.z;
    const float x2 = x + x, y2 = y + y, z2 = z + z;
    const float xx = x * x2, xy = x * y2, xz = x * z2;
    const float yy = y * y2, yz = y * z2, zz = z * z2;
    const float wx = w * x2, wy = w * y2, wz = w * z2;

    // raylib is column-major:
    // m0 m4 m8  m12     col0  col1  col2  col3
    // m1 m5 m9  m13
    // m2 m6 m10 m14
    // m3 m7 m11 m15

    // scale placed into rotation
    Matrix m = {};
    m.m0 = (1.0f - (yy + zz)) * sx;
    m.m1 = (xy + wz) * sx;
    m.m2 = (xz - wy) * sx;
    m.m3 = 0.0f;

    m.m4 = (xy - wz) * sy;
    m.m5 = (1.0f - (xx + zz)) * sy;
    m.m6 = (yz + wx) * sy;
    m.m7 = 0.0f;

    m.m8 = (xz + wy) * sz;
    m.m9 = (yz - wx) * sz;
    m.m10 = (1.0f - (xx + yy)) * sz;
    m.m11 = 0.0f;

    m.m12 = pos.data_[0];
    m.m13 = pos.data_[1];
    m.m14 = pos.data_[2];
    m.m15 = 1.0f;

    return m;
}

static Vector3 to_rl(const PhysicsEngine::Vec3 &v) {
    return {v.data_[0], v.data_[1], v.data_[2]};
}

void DebugDraw::init() {
    // unit cube: side length 1, centered at origin
    const Mesh cube_mesh = GenMeshCube(1.0f, 1.0f, 1.0f);
    this->unit_cube = LoadModelFromMesh(cube_mesh);

    const Mesh sphere_mesh = GenMeshSphere(1.0f, 16, 16);
    this->unit_sphere = LoadModelFromMesh(sphere_mesh);
}

void DebugDraw::shutdown() {
    UnloadModel(this->unit_cube);
    UnloadModel(this->unit_sphere);
}

void DebugDraw::draw_body(const PhysicsEngine::RigidBody &body,
                          Color colour) const {
    using PhysicsEngine::ShapeType;

    switch (body.shape.type) {
    case ShapeType::box: {
        const auto &he = body.shape.box.half_extents;

        // scale from unit cube to full box dimensions (2 * half_extents)
        const float sx = 2.0f * he.data_[0];
        const float sy = 2.0f * he.data_[1];
        const float sz = 2.0f * he.data_[2];

        Matrix transform =
            build_transform(body.orientation, body.position, sx, sy, sz);

        Model model = this->unit_cube;
        model.transform = transform;

        DrawModel(model, {0, 0, 0}, 1.0f, colour);

        // wireframe for depth
        DrawModelWires(model, {0, 0, 0}, 1.0f, BLACK);
        break;
    }
    case ShapeType::sphere: {
        const float r = body.shape.sphere.radius;
        Matrix transform =
            build_transform(body.orientation, body.position, r, r, r);

        Model model = this->unit_sphere;
        model.transform = transform;
        DrawModel(model, {0, 0, 0}, 1.0f, colour);
        DrawModelWires(model, {0, 0, 0}, 1.0f, BLACK);
        break;
    }
    }
}

void DebugDraw::draw_bodies(
    const std::vector<PhysicsEngine::RigidBody> &bodies) const {
    // alternate colours so bodies are distinguishable
    static const Color palette[] = {
        BLUE, RED, GREEN, ORANGE, PURPLE, SKYBLUE, MAROON, LIME,
    };
    static const int palette_size = sizeof(palette) / sizeof(palette[0]);

    for (std::size_t i = 0; i < bodies.size(); i++) {
        const Color colour =
            bodies[i].is_static() ? DARKGRAY : palette[i % palette_size];
        draw_body(bodies[i], colour);
    }
}

void DebugDraw::draw_point(const PhysicsEngine::Vec3 &p, Color colour) const {
    DrawSphere(to_rl(p), 0.05f, colour);
}

void DebugDraw::draw_line(const PhysicsEngine::Vec3 &a,
                          const PhysicsEngine::Vec3 &b, Color colour) const {
    DrawLine3D(to_rl(a), to_rl(b), colour);
}

void DebugDraw::draw_grid(int slices, float spacing) const {
    DrawGrid(slices, spacing);
}
} // namespace PhysicsEngineRendering
