#include "manifold_clip.h"
#include "math_utils.h"
#include <cassert>
#include <cmath>

namespace PhysicsEngine::Collisions {

// maximum polygon vertices during clipping (4 face verts + clipping can add)
static constexpr int MAX_CLIP_VERTS = 16;

// support-face extraction per shape type

//  returns the 4 world-space vertices of the face whose
// outward normal is most aligned with dir. also returns the face
// outward normal in world space.
static int box_support_face(const RigidBody &body, const Vec3 &dir,
                            Vec3 *out_verts, Vec3 &out_normal) {
    const Mat<3, 3> R = body.rotation_matrix();
    const Vec3 &he = body.shape.box.half_extents;

    // find which face axis is most aligned with dir
    int best_axis = 0;
    float best_dot = -1e30f;
    float best_sign = 1.0f;

    for (int i = 0; i < 3; i++) {
        Vec3 face_n = Vec3::zeros();
        face_n.data_[i] = 1.0f;
        Vec3 world_n = R * face_n;

        float d = dot(world_n, dir);
        if (d > best_dot) {
            best_dot = d;
            best_axis = i;
            best_sign = 1.0f;
        }
        if (-d > best_dot) {
            best_dot = -d;
            best_axis = i;
            best_sign = -1.0f;
        }
    }

    Vec3 local_n = Vec3::zeros();
    local_n.data_[best_axis] = best_sign;
    out_normal = R * local_n;

    int i1 = (best_axis + 1) % 3;
    int i2 = (best_axis + 2) % 3;

    // build 4 local vertices
    Vec3 local_verts[4];
    for (int v = 0; v < 4; v++) {
        float s1 = (v & 1) ? 1.0f : -1.0f;
        float s2 = (v & 2) ? 1.0f : -1.0f;
        local_verts[v] = Vec3::zeros();
        local_verts[v].data_[best_axis] = best_sign * he.data_[best_axis];
        local_verts[v].data_[i1] = s1 * he.data_[i1];
        local_verts[v].data_[i2] = s2 * he.data_[i2];
    }

    Vec3 tmp = local_verts[2];
    local_verts[2] = local_verts[3];
    local_verts[3] = tmp;

    for (int v = 0; v < 4; v++)
        out_verts[v] = body.position + R * local_verts[v];

    return 4;
}

// for a sphere: no face to clip, return 0
static int sphere_support_face(const RigidBody &body, const Vec3 &dir,
                               Vec3 *out_verts, Vec3 &out_normal) {
    return 0;
}

// for later
static int polygon_support_face(const RigidBody &body, const Vec3 &dir,
                                Vec3 *out_verts, Vec3 &out_normal) {
    return 0;
}

// dispatcher
static int get_support_face(const RigidBody &body, const Vec3 &dir,
                            Vec3 *out_verts, Vec3 &out_normal) {
    switch (body.shape.type) {
    case ShapeType::box:
        return box_support_face(body, dir, out_verts, out_normal);
    case ShapeType::sphere:
        return sphere_support_face(body, dir, out_verts, out_normal);
    case ShapeType::polygon:
        return polygon_support_face(body, dir, out_verts, out_normal);
    }
    return 0;
}

// basically sutherland clipping
// clips polygon `in` (n_in verts) against the half-space dot(n, v) <= d
// writes result to `out`, returns number of output vertices
static int clip_poly(const Vec3 *in, int n_in, Vec3 *out, const Vec3 &plane_n,
                     float plane_d) {
    if (n_in == 0)
        return 0;
    int n_out = 0;
    for (int i = 0; i < n_in; i++) {
        const Vec3 &curr = in[i];
        const Vec3 &next = in[(i + 1) % n_in];
        float d_curr = dot(plane_n, curr) - plane_d;
        float d_next = dot(plane_n, next) - plane_d;
        if (d_curr <= 0.0f)
            out[n_out++] = curr;
        if ((d_curr > 0.0f) != (d_next > 0.0f)) {
            float t = d_curr / (d_curr - d_next);
            out[n_out++] = curr + (next - curr) * t;
        }
    }
    return n_out;
}

std::size_t clip_manifold(const RigidBody &a, std::size_t a_idx,
                          const RigidBody &b, std::size_t b_idx,
                          const Vec3 &epa_normal, float /*epa_depth*/,
                          ContactManifold &manifold) {
    // get support face on A in direction of epa_normal (A's face facing B)
    // and support face on B in direction of -epa_normal (B's face facing A)
    // epa_normal points in A's separation direction (away from B in Minkowski
    // space), so A's colliding face has outward normal aligned with epa_normal
    Vec3 face_a_verts[MAX_CLIP_VERTS], face_b_verts[MAX_CLIP_VERTS];
    Vec3 normal_a, normal_b;

    int n_a = get_support_face(a, epa_normal, face_a_verts, normal_a);
    int n_b = get_support_face(b, -epa_normal, face_b_verts, normal_b);

    if (n_a == 0 || n_b == 0)
        return 0;

    bool a_is_ref = std::abs(dot(normal_a, epa_normal)) >=
                    std::abs(dot(normal_b, -epa_normal));

    const Vec3 *ref_verts = a_is_ref ? face_a_verts : face_b_verts;
    int n_ref = a_is_ref ? n_a : n_b;
    Vec3 ref_normal = a_is_ref ? normal_a : normal_b;

    const Vec3 *inc_verts_src = a_is_ref ? face_b_verts : face_a_verts;
    int n_inc = a_is_ref ? n_b : n_a;

    Vec3 poly[MAX_CLIP_VERTS], clipped[MAX_CLIP_VERTS];
    for (int i = 0; i < n_inc && i < MAX_CLIP_VERTS; i++)
        poly[i] = inc_verts_src[i];
    int n_poly = n_inc;

    for (int i = 0; i < n_ref; i++) {
        const Vec3 &edge_start = ref_verts[i];
        const Vec3 &edge_end = ref_verts[(i + 1) % n_ref];
        Vec3 edge = edge_end - edge_start;

        Vec3 side_n = cross(edge, ref_normal);
        float side_len = length(side_n);
        if (side_len < 1e-8f)
            continue;
        side_n = side_n * (1.0f / side_len);

        float side_d = dot(side_n, edge_start);

        n_poly = clip_poly(poly, n_poly, clipped, side_n, side_d);

        // swap buffers
        for (int j = 0; j < n_poly && j < MAX_CLIP_VERTS; j++)
            poly[j] = clipped[j];

        if (n_poly == 0)
            return 0;
    }

    float ref_d = dot(ref_normal, ref_verts[0]);

    manifold.body1 = a_idx;
    manifold.body2 = b_idx;
    manifold.num_points = 0;

    for (int i = 0; i < n_poly; i++) {
        float separation = dot(ref_normal, poly[i]) - ref_d;
        if (separation > 0.0f)
            continue; // above reference plane, discard

        // project point onto reference face for stable contact
        Vec3 contact_on_ref = poly[i] - ref_normal * separation;
        Vec3 contact_on_inc = poly[i];

        // r vectors relative to body COMs
        Vec3 r1, r2;
        if (a_is_ref) {
            r1 = contact_on_ref - a.position;
            r2 = contact_on_inc - b.position;
        } else {
            r1 = contact_on_inc - a.position;
            r2 = contact_on_ref - b.position;
        }

        // penetration is negative when overlapping
        float pen = separation; // already <= 0

        // contact id, hash the vertex index for caching
        uint32_t id = static_cast<uint32_t>(i);

        manifold.points[manifold.num_points] = {r1, r2, epa_normal, pen, id};
        manifold.num_points++;

        if (manifold.num_points >= 8)
            break;
    }

    return manifold.num_points;
}

} // namespace PhysicsEngine::Collisions
