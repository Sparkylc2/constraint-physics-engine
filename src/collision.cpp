#include "collision.h"
#include "gjk.h"
#include "mat.h"
#include "math_utils.h"
#include "vec3.h"

namespace PhysicsEngine::Collisions {

// sutherland-Hodgman: clip polygon against plane where dot(n, v) <= d
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

// closest points between two line segments
// seg1: p1 + s * d1, s in [-h1, h1], d1 is unit
// seg2: p2 + t * d2, t in [-h2, h2], d2 is unit
static void closest_seg_seg(const Vec3 &p1, const Vec3 &d1, float h1,
                            const Vec3 &p2, const Vec3 &d2, float h2, Vec3 &c1,
                            Vec3 &c2) {
    Vec3 r = p1 - p2;
    float b = dot(d1, d2);
    float f = dot(d1, r);
    float e = dot(d2, r);
    float denom = 1.0f - b * b;

    float s = (denom > 1e-6f) ? std::clamp((b * e - f) / denom, -h1, h1) : 0.0f;
    float t = std::clamp(b * s + e, -h2, h2);
    s = std::clamp(b * t - f, -h1, h1);

    c1 = p1 + d1 * s;
    c2 = p2 + d2 * t;
}

ContactManifold collide(const std::vector<RigidBody> bodies,
                        std::size_t b_1_idx, std::size_t b_2_idx) {
    const RigidBody b_1 = bodies[b_1_idx];
    const RigidBody b_2 = bodies[b_2_idx];

    ShapeType t_1 = b_1.shape.type;
    ShapeType t_2 = b_2.shape.type;

    // sphere-sphere
    if (t_1 == ShapeType::sphere && t_2 == ShapeType::sphere)
        return sphere_sphere(b_1, b_1_idx, b_2, b_2_idx);
    // box-sphere (ensure box is first arg)
    if (t_1 == ShapeType::box && t_2 == ShapeType::sphere)
        return box_sphere(b_1, b_1_idx, b_2, b_2_idx);
    if (t_1 == ShapeType::sphere && t_2 == ShapeType::box)
        return box_sphere(b_2, b_2_idx, b_1, b_1_idx);
    if (t_1 == ShapeType::box && t_2 == ShapeType::box)
        return box_box(b_1, b_1_idx, b_2, b_2_idx);

    return GJK::convex_convex(b_1, b_1_idx, b_2, b_2_idx);
}

ContactManifold box_plane(const RigidBody &plane, std::size_t plane_idx,
                          const RigidBody &box, std::size_t box_idx) {
    // body 1 is ground, body 2 is the box
    Vec3 b1_normal = {0.0f, 1.0f,
                      0.0f}; // simplified so normal is just straight up

    Vec3 b1_position = plane.position;

    // body 2 info
    Mat<3, 3> b2_rotation_matrix = box.rotation_matrix();
    Vec3 b2_position = box.position;

    Vec3 b2_local_vertices[8]; // todo: maybe store this in the shape object
                               // itself so we dont recompute

    box.shape.box.get_vertices(b2_local_vertices);

    Vec3 b2_world_vertices[8];
    for (std::size_t i = 0; i < 8; i++) {
        b2_world_vertices[i] =
            b2_position + b2_rotation_matrix * b2_local_vertices[i];
    }

    const float plane_y = vy(b1_position) + vy(plane.shape.box.half_extents);

    ContactManifold contact_manifold = {plane_idx, box_idx};
    // find the contact point
    for (std::size_t i = 0; i < 8; i++) {
        float pen = vy(b2_world_vertices[i]) - plane_y;
        if (pen > 0.0f)
            continue; // no penetration

        const Vec3 r2_world = b2_world_vertices[i] - b2_position;
        const Vec3 r1_world = make_vec3(vx(b2_world_vertices[i]), plane_y,
                                        vz(b2_world_vertices[i])) -
                              b1_position;
        const Vec3 normal = b1_normal;
        const float penetration = pen;
        const uint32_t id = i;

        ContactPoint point = {r1_world, r2_world, normal, penetration, id};
        contact_manifold.points[contact_manifold.num_points] = point;
        contact_manifold.num_points++;
    }
    return contact_manifold;
}

ContactManifold sphere_plane(const RigidBody &plane, std::size_t plane_idx,
                             const RigidBody &sphere, std::size_t box_idx) {
    // body 1 is ground, body 2 is the box
    Vec3 b1_normal = {0.0f, 1.0f,
                      0.0f}; // simplified so normal is just straight up

    Vec3 b1_position = plane.position;

    Vec3 b2_position = sphere.position;
    const float b2_radius = sphere.shape.sphere.radius;

    const float plane_y = vy(b1_position) + vy(plane.shape.box.half_extents);
    const float pen = vy(b2_position) - b2_radius - plane_y;

    ContactManifold manifold = {plane_idx, box_idx};
    if (pen <= 0.0f) {
        const Vec3 contact_pt = b2_position - b1_normal * b2_radius;
        const Vec3 r1_world = contact_pt - b1_position;
        const Vec3 r2_world = contact_pt - b2_position;

        ContactPoint point = {r1_world, r2_world, b1_normal, pen, 0};
        manifold.points[manifold.num_points] = point;
        manifold.num_points++;
    }
    return manifold;
}

ContactManifold sphere_sphere(const RigidBody &s1, std::size_t s1_idx,
                              const RigidBody &s2, std::size_t s2_idx) {

    // sphere sep and dist
    const Vec3 d = s2.position - s1.position;
    const float d_norm = MathUtils::NORM(d);

    // normal of collision, making sure if they are nearly ontop of eachother
    // its fine
    const Vec3 normal =
        !MathUtils::approx_zero(d_norm) ? d / d_norm : make_vec3(0, 1.0, 0.0);
    const float pen =
        d_norm - (s1.shape.sphere.radius + s2.shape.sphere.radius);

    ContactManifold manifold = {s1_idx, s2_idx};
    if (pen <= 0.0f) {
        // midpoint of overlap region
        const Vec3 contact_pt =
            s1.position + normal * (s1.shape.sphere.radius + pen / 2.0f);

        const Vec3 r1_world = contact_pt - s1.position;
        const Vec3 r2_world = contact_pt - s2.position;

        ContactPoint point = {r1_world, r2_world, normal, pen, 0};
        manifold.points[manifold.num_points] = point;
        manifold.num_points++;
    }

    return manifold;
};

ContactManifold box_sphere(const RigidBody &b, std::size_t b_idx,
                           const RigidBody &s, std::size_t s_idx) {

    const Mat<3, 3> b_rot_mat = b.rotation_matrix();
    const Mat<3, 3> b_rot_mat_t = b_rot_mat.transpose();

    const Vec3 h_f = b.shape.box.half_extents;

    const Vec3 local_center = b_rot_mat_t * (s.position - b.position);
    const Vec3 closest = {
        std::max(-vx(h_f), std::min(vx(local_center), vx(h_f))),
        std::max(-vy(h_f), std::min(vy(local_center), vy(h_f))),
        std::max(-vz(h_f), std::min(vz(local_center), vz(h_f))),
    };

    const Vec3 delta = local_center - closest;
    const float dist = MathUtils::NORM(delta);

    ContactManifold manifold = {b_idx, s_idx};

    if (dist > s.shape.sphere.radius) {
        return manifold;
    }

    if (!MathUtils::approx_zero(dist)) {
        // sphere centre outside of box
        const Vec3 normal = b_rot_mat * (delta / dist);
        const float pen = dist - s.shape.sphere.radius;
        const Vec3 contact_pt = b_rot_mat * closest + b.position;

        const Vec3 r1_world = contact_pt - b.position;
        const Vec3 r2_world = contact_pt - s.position;

        manifold.points[manifold.num_points] = {r1_world, r2_world, normal, pen,
                                                0};
        manifold.num_points++;
    } else {
        // sphere centre inside box
        float min_dist = std::numeric_limits<float>::max();
        int min_axis = 0;
        float min_sign = 1.0f;

        for (int i = 0; i < 3; i++) {
            float pos_dist = h_f.data_[i] - local_center.data_[i];
            float neg_dist = h_f.data_[i] + local_center.data_[i];
            if (pos_dist < min_dist) {
                min_dist = pos_dist;
                min_axis = i;
                min_sign = 1.0f;
            }
            if (neg_dist < min_dist) {
                min_dist = neg_dist;
                min_axis = i;
                min_sign = -1.0f;
            }
        }

        Vec3 local_normal = Vec3::zeros();
        local_normal.data_[min_axis] = min_sign;
        const Vec3 normal = b_rot_mat * local_normal;
        const float pen = -(min_dist + s.shape.sphere.radius);
        const Vec3 contact_pt =
            b.position + b_rot_mat * (local_center + local_normal * min_dist);

        const Vec3 r1_world = contact_pt - b.position;
        const Vec3 r2_world = contact_pt - s.position;
        manifold.points[manifold.num_points] = {r1_world, r2_world, normal, pen,
                                                0};
        manifold.num_points++;
    }
    return manifold;
};

ContactManifold box_box(const RigidBody &a, std::size_t a_idx,
                        const RigidBody &b, std::size_t b_idx) {
    const Mat<3, 3> Ra = a.rotation_matrix();
    const Mat<3, 3> Rb = b.rotation_matrix();
    const Mat<3, 3> R = Ra.transpose() * Rb; // relative rotation
    const Vec3 t_vec =
        Ra.transpose() * (b.position - a.position); // B center in A's frame

    const Vec3 &ha = a.shape.box.half_extents;
    const Vec3 &hb = b.shape.box.half_extents;

    // abs(R) + epsilon for parallel edge stability (thanks claude)
    Mat<3, 3> absR;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            absR(i, j) = std::abs(R(i, j)) + 1e-6f;

    float min_pen = std::numeric_limits<float>::max();
    int min_type = -1; // 0=face_a, 1=face_b, 2=edge
    int min_idx = -1;  // which face axis
    int min_edge_a = -1, min_edge_b = -1;
    Vec3 min_axis_local; // separating axis in A's local frame, points A->B

    // face normals of A in the three axes
    for (int i = 0; i < 3; i++) {
        float sep = std::abs(t_vec.data_[i]);
        float proj = ha.data_[i] + hb.data_[0] * absR(i, 0) +
                     hb.data_[1] * absR(i, 1) + hb.data_[2] * absR(i, 2);
        float pen = proj - sep;
        if (pen < 0.0f)
            return {a_idx, b_idx};
        if (pen < min_pen) {
            min_pen = pen;
            min_type = 0;
            min_idx = i;
            Vec3 ax = Vec3::zeros();
            ax.data_[i] = (t_vec.data_[i] >= 0.0f) ? 1.0f : -1.0f;
            min_axis_local = ax;
        }
    }

    // face normals of B in the three axes
    for (int j = 0; j < 3; j++) {
        float sep_val =
            vx(t_vec) * R(0, j) + vy(t_vec) * R(1, j) + vz(t_vec) * R(2, j);
        float sep = std::abs(sep_val);
        float proj = hb.data_[j] + ha.data_[0] * absR(0, j) +
                     ha.data_[1] * absR(1, j) + ha.data_[2] * absR(2, j);
        float pen = proj - sep;
        if (pen < 0.0f)
            return {a_idx, b_idx};
        if (pen < min_pen) {
            min_pen = pen;
            min_type = 1;
            min_idx = j;
            Vec3 ax = mat_col(R, j);
            if (sep_val < 0.0f)
                ax = -ax;
            min_axis_local = ax;
        }
    }

    // 9 edge-edge axes
    for (int i = 0; i < 3; i++) {
        int i1 = (i + 1) % 3, i2 = (i + 2) % 3;
        for (int j = 0; j < 3; j++) {
            int j1 = (j + 1) % 3, j2 = (j + 2) % 3;

            // axis = unit(i) x R.col(j) in A's frame
            Vec3 L = Vec3::zeros();
            L.data_[i1] = -R(i2, j);
            L.data_[i2] = R(i1, j);
            float axis_len = length(L);
            if (axis_len < 1e-6f)
                continue; // parallel edges, skip

            // use unnormalized formulas for the overlap test
            // (Ericson/Gottschalk)
            float sep_val =
                t_vec.data_[i2] * R(i1, j) - t_vec.data_[i1] * R(i2, j);
            float proj_a =
                ha.data_[i1] * absR(i2, j) + ha.data_[i2] * absR(i1, j);
            float proj_b =
                hb.data_[j1] * absR(i, j2) + hb.data_[j2] * absR(i, j1);

            float pen_raw = proj_a + proj_b - std::abs(sep_val);
            if (pen_raw < 0.0f)
                return {a_idx, b_idx};

            float pen = pen_raw / axis_len;

            // bias toward face contacts to avoid jitter (again thanks claude)
            if (pen * 1.05f < min_pen) {
                min_pen = pen;
                min_type = 2;
                min_edge_a = i;
                min_edge_b = j;
                Vec3 axis_dir = L * (1.0f / axis_len);
                if (sep_val < 0.0f)
                    axis_dir = -axis_dir;
                min_axis_local = axis_dir;
            }
        }
    }

    // separating axis in world space (always points A->B)
    Vec3 normal_world = Ra * min_axis_local;
    ContactManifold manifold = {a_idx, b_idx};

    // face contact (Sutherland-Hodgman clipping) ========
    if (min_type <= 1) {
        const RigidBody &ref = (min_type == 0) ? a : b;
        const RigidBody &inc = (min_type == 0) ? b : a;
        const Mat<3, 3> &R_ref = (min_type == 0) ? Ra : Rb;
        const Mat<3, 3> &R_inc = (min_type == 0) ? Rb : Ra;
        const Vec3 &h_ref = (min_type == 0) ? ha : hb;
        const Vec3 &h_inc = (min_type == 0) ? hb : ha;

        // reference face normal, outward from ref body
        Vec3 ref_normal = (min_type == 0) ? normal_world : -normal_world;
        int ref_ax = min_idx;
        int ref_s1 = (ref_ax + 1) % 3, ref_s2 = (ref_ax + 2) % 3;

        // reference face center in world
        Vec3 ref_center = ref.position + ref_normal * h_ref.data_[ref_ax];

        // find incident face, most anti-parallel face to ref normal
        Vec3 ref_n_inc_local = R_inc.transpose() * ref_normal;
        int inc_ax = 0;
        float max_abs = 0.0f;
        for (int k = 0; k < 3; k++) {
            float val = std::abs(ref_n_inc_local.data_[k]);
            if (val > max_abs) {
                max_abs = val;
                inc_ax = k;
            }
        }
        float inc_sign = (ref_n_inc_local.data_[inc_ax] >= 0.0f) ? -1.0f : 1.0f;

        // incident face vertices in world space
        int inc_s1 = (inc_ax + 1) % 3, inc_s2 = (inc_ax + 2) % 3;
        float winding[4][2] = {{1, 1}, {-1, 1}, {-1, -1}, {1, -1}};
        Vec3 inc_verts[4];
        for (int k = 0; k < 4; k++) {
            Vec3 v = Vec3::zeros();
            v.data_[inc_ax] = inc_sign * h_inc.data_[inc_ax];
            v.data_[inc_s1] = winding[k][0] * h_inc.data_[inc_s1];
            v.data_[inc_s2] = winding[k][1] * h_inc.data_[inc_s2];
            inc_verts[k] = inc.position + R_inc * v;
        }

        // 4 side planes of the reference face (in world space)
        // each keeps points inside the face bounds
        Vec3 side_n[4] = {
            mat_col(R_ref, ref_s1),
            -mat_col(R_ref, ref_s1),
            mat_col(R_ref, ref_s2),
            -mat_col(R_ref, ref_s2),
        };
        float side_d[4] = {
            h_ref.data_[ref_s1] + dot(side_n[0], ref.position),
            h_ref.data_[ref_s1] + dot(side_n[1], ref.position),
            h_ref.data_[ref_s2] + dot(side_n[2], ref.position),
            h_ref.data_[ref_s2] + dot(side_n[3], ref.position),
        };

        // clip incident face through all 4 side planes
        Vec3 buf1[16], buf2[16];
        std::copy(inc_verts, inc_verts + 4, buf1);
        int n = 4;
        n = clip_poly(buf1, n, buf2, side_n[0], side_d[0]);
        n = clip_poly(buf2, n, buf1, side_n[1], side_d[1]);
        n = clip_poly(buf1, n, buf2, side_n[2], side_d[2]);
        n = clip_poly(buf2, n, buf1, side_n[3], side_d[3]);

        // keep clipped points below the reference face
        for (int k = 0; k < n && manifold.num_points < 8; k++) {
            float pen = dot(ref_normal, buf1[k] - ref_center);
            if (pen > 0.0f)
                continue;

            Vec3 contact_on_ref = buf1[k] - ref_normal * pen;
            Vec3 r1, r2;
            if (min_type == 0) {
                r1 = contact_on_ref - a.position;
                r2 = buf1[k] - b.position;
            } else {
                r1 = buf1[k] - a.position;
                r2 = contact_on_ref - b.position;
            }

            manifold.points[manifold.num_points] = {r1, r2, normal_world, pen,
                                                    static_cast<uint32_t>(k)};
            manifold.num_points++;
        }

        // edge-edge contact
    } else {
        int i = min_edge_a, j = min_edge_b;
        int i1 = (i + 1) % 3, i2 = (i + 2) % 3;
        int j1 = (j + 1) % 3, j2 = (j + 2) % 3;

        // support edge on A: direction along axis i, positioned by the other
        // two
        Vec3 dir_a = mat_col(Ra, i);
        float sa1 = (dot(mat_col(Ra, i1), normal_world) > 0.0f) ? 1.0f : -1.0f;
        float sa2 = (dot(mat_col(Ra, i2), normal_world) > 0.0f) ? 1.0f : -1.0f;
        Vec3 mid_a = a.position + mat_col(Ra, i1) * (sa1 * ha.data_[i1]) +
                     mat_col(Ra, i2) * (sa2 * ha.data_[i2]);

        // support edge on B: support in -normal direction
        Vec3 dir_b = mat_col(Rb, j);
        float sb1 = (dot(mat_col(Rb, j1), normal_world) > 0.0f) ? -1.0f : 1.0f;
        float sb2 = (dot(mat_col(Rb, j2), normal_world) > 0.0f) ? -1.0f : 1.0f;
        Vec3 mid_b = b.position + mat_col(Rb, j1) * (sb1 * hb.data_[j1]) +
                     mat_col(Rb, j2) * (sb2 * hb.data_[j2]);

        Vec3 c1, c2;
        closest_seg_seg(mid_a, dir_a, ha.data_[i], mid_b, dir_b, hb.data_[j],
                        c1, c2);

        manifold.points[0] = {c1 - a.position, c2 - b.position, normal_world,
                              -min_pen, 0};
        manifold.num_points = 1;
    }

    return manifold;
}

ContactManifold polygon_sphere(const RigidBody &polygon,
                               std::size_t polygon_idx, const RigidBody &sphere,
                               std::size_t sphere_idx) {
    return GJK::convex_convex(polygon, polygon_idx, sphere, sphere_idx);
}

ContactManifold polygon_box(const RigidBody &polygon, std::size_t polygon_idx,
                            const RigidBody &box, std::size_t box_idx) {
    return GJK::convex_convex(polygon, polygon_idx, box, box_idx);
}

ContactManifold polygon_polygon(const RigidBody &body_1, std::size_t body_1_idx,
                                const RigidBody &body_2,
                                std::size_t body_2_idx) {
    return GJK::convex_convex(body_1, body_1_idx, body_2, body_2_idx);
}
} // namespace PhysicsEngine::Collisions
