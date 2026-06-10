#include "gjk.h"
#include "manifold_clip.h"
#include "math_utils.h"

namespace PhysicsEngine::Collisions::GJK {

// support functions
Vec3 support(const RigidBody &body, const Vec3 &dir) {
    switch (body.shape.type) {
    case ShapeType::box: {
        // transform direction to local space, pick the vertex that
        // maximises dot product with it, transform back
        const Mat<3, 3> R = body.rotation_matrix();
        const Vec3 local_dir = R.transpose() * dir;
        const Vec3 &he = body.shape.box.half_extents;

        Vec3 local_sup;
        for (int i = 0; i < 3; i++) {
            local_sup.data_[i] =
                (local_dir.data_[i] >= 0.0f) ? he.data_[i] : -he.data_[i];
        }

        return body.position + R * local_sup;
    }
    case ShapeType::sphere: {
        const float len = length(dir);
        if (MathUtils::approx_zero(len))
            return body.position;
        return body.position + dir * (body.shape.sphere.radius / len);
    }
    case ShapeType::polygon: {
        const Mat<3, 3> R = body.rotation_matrix();
        const Vec3 local_dir = R.transpose() * dir;
        const auto &poly = body.shape.polygon;
        assert(poly.num_vertices > 0 && "polygon with no vertices");

        std::size_t best = 0;
        float best_dot = dot(poly.vertices[0], local_dir);
        for (std::size_t i = 1; i < poly.num_vertices; i++) {
            float d = dot(poly.vertices[i], local_dir);
            if (d > best_dot) {
                best_dot = d;
                best = i;
            }
        }
        return body.position + R * poly.vertices[best];
    }
    }
    return body.position;
}

// internal stuff
// it tracks a point in Minkowski difference space together with the
// witness points on each shape that produced it
struct SupportPoint {
    Vec3 point; // = a - b, lives in Minkowski difference space
    Vec3 a;     // witness on shape A (world space)
    Vec3 b;     // witness on shape B (world space)
};

static SupportPoint support_md(const RigidBody &a, const RigidBody &b,
                               const Vec3 &dir) {
    Vec3 sa = support(a, dir);
    Vec3 sb = support(b, -dir);
    return {sa - sb, sa, sb};
}

// (a x b) x c — used in GJK to get a direction perpendicular to a line
// segment and pointing toward the origin
static Vec3 triple_cross(const Vec3 &a, const Vec3 &b, const Vec3 &c) {
    return cross(cross(a, b), c);
}

// gjk simplex definition
struct Simplex {
    SupportPoint pts[4];
    int size = 0;

    // newest point always goes to index 0
    void push_front(const SupportPoint &p) {
        assert(size < 4);
        for (int i = size; i > 0; i--)
            pts[i] = pts[i - 1];
        pts[0] = p;
        size++;
    }
};

// case handlers:
// convention: pts[0] = A (newest), pts[1] = B, pts[2] = C, pts[3] = D
// each handler returns true if the simplex contains the origin,
// otherwise it prunes the simplex and sets a new search direction

static bool do_line(Simplex &s, Vec3 &dir) {
    const Vec3 &a = s.pts[0].point;
    const Vec3 &b = s.pts[1].point;

    const Vec3 ab = b - a;
    const Vec3 ao = -a; // vector from A toward origin

    if (dot(ab, ao) > 0.0f) {
        // origin projects onto segment AB
        // direction: perpendicular to AB, toward origin
        dir = triple_cross(ab, ao, ab);

        // degenerate: origin lies exactly on the line AB
        if (MathUtils::approx_zero(length(dir))) {
            // pick any perpendicular
            dir = (std::abs(ab.data_[0]) < 0.9f)
                      ? cross(ab, make_vec3(1.0f, 0.0f, 0.0f))
                      : cross(ab, make_vec3(0.0f, 1.0f, 0.0f));
        }
        // keep {A, B}
    } else {
        // origin is behind A, drop B
        s.size = 1;
        dir = ao;
    }
    return false;
}

static bool do_line_from_tri(Simplex &s, Vec3 &dir);

static bool do_triangle(Simplex &s, Vec3 &dir) {
    const Vec3 &a = s.pts[0].point;
    const Vec3 &b = s.pts[1].point;
    const Vec3 &c = s.pts[2].point;

    const Vec3 ab = b - a;
    const Vec3 ac = c - a;
    const Vec3 ao = -a;
    const Vec3 abc = cross(ab, ac); // triangle normal

    // check if origin is outside edge AC (on the side away from B)
    if (dot(cross(abc, ac), ao) > 0.0f) {
        if (dot(ac, ao) > 0.0f) {
            // region AC: keep A and C
            s.pts[1] = s.pts[2]; // move C to slot 1
            s.size = 2;
            dir = triple_cross(ac, ao, ac);
        } else {
            // fall through to AB edge check
            s.size = 2; // keep A, B
            return do_line(s, dir);
        }
    } else if (dot(cross(ab, abc), ao) > 0.0f) {
        // origin is outside edge AB
        s.size = 2; // keep A, B
        return do_line(s, dir);
    } else {
        // origin projects inside the triangle, check which side
        if (dot(abc, ao) > 0.0f) {
            // above triangle (same side as normal)
            dir = abc;
            // keep {A, B, C} ordering as-is
        } else {
            // below triangle, flip winding so normal faces origin
            SupportPoint tmp = s.pts[1];
            s.pts[1] = s.pts[2];
            s.pts[2] = tmp;
            dir = -abc;
        }
    }
    return false;
}

static bool do_tetrahedron(Simplex &s, Vec3 &dir) {
    const Vec3 &a = s.pts[0].point;
    const Vec3 &b = s.pts[1].point;
    const Vec3 &c = s.pts[2].point;
    const Vec3 &d = s.pts[3].point;

    const Vec3 ab = b - a;
    const Vec3 ac = c - a;
    const Vec3 ad = d - a;
    const Vec3 ao = -a;

    // face normals, oriented outward (away from the opposite vertex)
    Vec3 abc = cross(ab, ac);
    if (dot(abc, ad) > 0.0f)
        abc = -abc; // make it point away from D

    Vec3 acd = cross(ac, ad);
    if (dot(acd, ab) > 0.0f)
        acd = -acd; // point away from B

    Vec3 adb = cross(ad, ab);
    if (dot(adb, ac) > 0.0f)
        adb = -adb; // point away from C

    // origin outside face ABC -> reduce to triangle ABC
    if (dot(abc, ao) > 0.0f) {
        // keep A(0), B(1), C(2), drop D
        s.size = 3;
        return do_triangle(s, dir);
    }

    // origin outside face ACD -> reduce to triangle ACD
    if (dot(acd, ao) > 0.0f) {
        s.pts[1] = s.pts[2]; // C into slot 1
        s.pts[2] = s.pts[3]; // D into slot 2
        s.size = 3;
        return do_triangle(s, dir);
    }

    // origin outside face ADB -> reduce to triangle ADB
    if (dot(adb, ao) > 0.0f) {
        SupportPoint tmp = s.pts[1]; // save B
        s.pts[1] = s.pts[3];         // D into slot 1
        s.pts[2] = tmp;              // B into slot 2
        s.size = 3;
        return do_triangle(s, dir);
    }

    // origin is inside the tetrahedron
    return true;
}

static bool do_simplex(Simplex &s, Vec3 &dir) {
    switch (s.size) {
    case 2:
        return do_line(s, dir);
    case 3:
        return do_triangle(s, dir);
    case 4:
        return do_tetrahedron(s, dir);
    }
    return false;
}

// main gjk loop
static constexpr int GJK_MAX_ITER = 64;

struct GJKResult {
    bool intersecting;
    Simplex simplex;
};

static GJKResult gjk(const RigidBody &a, const RigidBody &b) {
    // initial search direction: center of A toward center of B
    Vec3 dir = b.position - a.position;
    if (MathUtils::approx_zero(length(dir)))
        dir = make_vec3(1.0f, 0.0f, 0.0f);

    Simplex simplex;
    simplex.pts[0] = support_md(a, b, dir);
    simplex.size = 1;

    // search toward origin from first support point
    dir = -simplex.pts[0].point;
    if (MathUtils::approx_zero(length(dir))) {
        // if the first support point is the origin -> shapes are touching at a
        // single point. treat as intersection.
        return {true, simplex};
    }

    for (int iter = 0; iter < GJK_MAX_ITER; iter++) {
        SupportPoint new_pt = support_md(a, b, dir);

        // if the new point didn't pass the origin along the search
        // direction, the mink. diff. doesn't contain the origin
        if (dot(new_pt.point, dir) < 0.0f)
            return {false, {}};

        simplex.push_front(new_pt);

        if (do_simplex(simplex, dir))
            return {true, simplex};

        // if direction collapsed to zero, exit
        if (MathUtils::approx_zero(length(dir)))
            return {false, {}};
    }

    // didn't converge, treat as no intersection
    return {false, {}};
}

// epa
//
// if gjk confirms the origin is inside the mink. diff.
// the epa expands the gjk tetrahedron into a convex polytope and finds
// the face closest to the origin.
// that face then has a normal and a distance that gives us the penetration info

static constexpr int EPA_MAX_ITER = 64;
static constexpr int EPA_MAX_FACES = 128;
static constexpr int EPA_MAX_VERTS = 128;
static constexpr float EPA_TOLERANCE = 1e-4f;

struct EPAFace {
    int a, b, c;
    Vec3 normal;
    float distance; // from origin to face along normal (>= 0)
};

// builds a face from 3 vertex indices, orienting the normal to point
// away from the origin (outward)
static EPAFace make_face(const SupportPoint *verts, int a, int b, int c) {
    Vec3 ab = verts[b].point - verts[a].point;
    Vec3 ac = verts[c].point - verts[a].point;
    Vec3 n = cross(ab, ac);
    float len = length(n);

    EPAFace face;
    face.a = a;
    face.b = b;
    face.c = c;

    if (len < 1e-10f) {
        // degenerate face
        face.normal = make_vec3(1.0f, 0.0f, 0.0f);
        face.distance = 0.0f;
        return face;
    }

    n = n * (1.0f / len);

    // distance from origin to face plane = dot(normal, any_vertex)
    float d = dot(n, verts[a].point);

    // if negative, normal is pointing toward origin, needs to be flipped
    if (d < 0.0f) {
        n = -n;
        d = -d;
        // winding swapped to keep normal consistent with vertex order
        face.b = c;
        face.c = b;
    }

    face.normal = n;
    face.distance = d;
    return face;
}

// recovers barycentric coords of the projection of the origin onto a face,
// then interpolate the witness points to get contact locations
static void bary_contact(const SupportPoint *verts, const EPAFace &face,
                         Vec3 &contact_a, Vec3 &contact_b) {
    const Vec3 &p0 = verts[face.a].point;
    const Vec3 &p1 = verts[face.b].point;
    const Vec3 &p2 = verts[face.c].point;

    // closest point on face to origin is the projection along face normal
    Vec3 proj = face.normal * face.distance;

    // barycentric coordinates of proj in triangle (p0, p1, p2)
    Vec3 v0 = p1 - p0;
    Vec3 v1 = p2 - p0;
    Vec3 v2 = proj - p0;

    float d00 = dot(v0, v0);
    float d01 = dot(v0, v1);
    float d11 = dot(v1, v1);
    float d20 = dot(v2, v0);
    float d21 = dot(v2, v1);
    float denom = d00 * d11 - d01 * d01;

    float bv = 0.0f, bw = 0.0f, bu = 1.0f;
    if (std::abs(denom) > 1e-10f) {
        bv = (d11 * d20 - d01 * d21) / denom;
        bw = (d00 * d21 - d01 * d20) / denom;
        bu = 1.0f - bv - bw;
    }

    // clamps to triangle
    bu = std::max(0.0f, bu);
    bv = std::max(0.0f, bv);
    bw = std::max(0.0f, bw);
    float sum = bu + bv + bw;
    if (sum > 1e-10f) {
        bu /= sum;
        bv /= sum;
        bw /= sum;
    }

    // interpolate witness points
    contact_a =
        verts[face.a].a * bu + verts[face.b].a * bv + verts[face.c].a * bw;
    contact_b =
        verts[face.a].b * bu + verts[face.b].b * bv + verts[face.c].b * bw;
}

struct EPAResult {
    Vec3 normal;    // penetration normal (A -> B direction)
    float depth;    // positive penetration depth
    Vec3 contact_a; // contact point on shape A
    Vec3 contact_b; // contact point on shape B
    bool valid;
};

static EPAResult epa(const RigidBody &a, const RigidBody &b,
                     const Simplex &simplex) {

    EPAResult fail = {Vec3::zeros(), 0.0f, Vec3::zeros(), Vec3::zeros(), false};

    // need a full tetrahedron
    if (simplex.size < 4)
        return fail;

    // check for degenerate
    {
        Vec3 ab = simplex.pts[1].point - simplex.pts[0].point;
        Vec3 ac = simplex.pts[2].point - simplex.pts[0].point;
        Vec3 ad = simplex.pts[3].point - simplex.pts[0].point;
        if (std::abs(dot(cross(ab, ac), ad)) < 1e-8f)
            return fail;
    }

    SupportPoint verts[EPA_MAX_VERTS];
    int num_verts = 4;
    for (int i = 0; i < 4; i++)
        verts[i] = simplex.pts[i];

    EPAFace faces[EPA_MAX_FACES];
    int num_faces = 0;

    // initial 4 faces of the tetrahedron
    // make_face orients normals outward from origin
    faces[num_faces++] = make_face(verts, 0, 1, 2);
    faces[num_faces++] = make_face(verts, 0, 3, 1);
    faces[num_faces++] = make_face(verts, 0, 2, 3);
    faces[num_faces++] = make_face(verts, 1, 3, 2);

    for (int iter = 0; iter < EPA_MAX_ITER; iter++) {
        // find closest face to the origin
        int closest = 0;
        float min_dist = faces[0].distance;
        for (int i = 1; i < num_faces; i++) {
            if (faces[i].distance < min_dist) {
                min_dist = faces[i].distance;
                closest = i;
            }
        }

        const Vec3 search_dir = faces[closest].normal;

        // expand by sampling a new support point
        SupportPoint new_pt = support_md(a, b, search_dir);
        float new_dist = dot(new_pt.point, search_dir);

        // convergence, as new point barely extends past the closest face
        if (new_dist - min_dist < EPA_TOLERANCE) {
            EPAResult result;
            result.normal = faces[closest].normal;
            result.depth = faces[closest].distance;
            bary_contact(verts, faces[closest], result.contact_a,
                         result.contact_b);
            result.valid = true;
            return result;
        }

        // add the new vertex
        if (num_verts >= EPA_MAX_VERTS)
            break;
        int new_idx = num_verts++;
        verts[new_idx] = new_pt;

        // remove every face visible from the new point and collect
        // the horizon edges (boundary between removed and kept faces).
        //
        // an edge that appears once in the removed set is on the
        // horizon, one that appears twice is internal and should get removed
        struct Edge {
            int a, b;
        };
        Edge horizon[256];
        int num_horizon = 0;

        for (int i = num_faces - 1; i >= 0; i--) {
            // face is visible if the new point is on the outward side
            Vec3 to_pt = new_pt.point - verts[faces[i].a].point;
            if (dot(faces[i].normal, to_pt) <= 0.0f)
                continue;

            // face is visible, get edges
            int fa = faces[i].a, fb = faces[i].b, fc = faces[i].c;
            Edge edges[3] = {{fa, fb}, {fb, fc}, {fc, fa}};

            // look for the reverse edge already in the horizon list
            for (int e = 0; e < 3; e++) {
                bool found = false;
                for (int h = 0; h < num_horizon; h++) {
                    if (horizon[h].a == edges[e].b &&
                        horizon[h].b == edges[e].a) {
                        // internal edge, remove through a swap with last
                        horizon[h] = horizon[num_horizon - 1];
                        num_horizon--;
                        found = true;
                        break;
                    }
                }
                if (!found && num_horizon < 256) {
                    horizon[num_horizon++] = edges[e];
                }
            }

            // remove (again swap with last)
            faces[i] = faces[num_faces - 1];
            num_faces--;
        }

        // create a new face from each horizon edge
        // to the new vertex
        for (int h = 0; h < num_horizon && num_faces < EPA_MAX_FACES; h++) {
            faces[num_faces++] =
                make_face(verts, new_idx, horizon[h].a, horizon[h].b);
        }

        if (num_faces == 0)
            break; // somethings broken
    }

    // if it didn't converge, return best guess
    if (num_faces == 0)
        return fail;

    int closest = 0;
    float min_dist = faces[0].distance;
    for (int i = 1; i < num_faces; i++) {
        if (faces[i].distance < min_dist) {
            min_dist = faces[i].distance;
            closest = i;
        }
    }

    EPAResult result;
    result.normal = faces[closest].normal;
    result.depth = faces[closest].distance;
    bary_contact(verts, faces[closest], result.contact_a, result.contact_b);
    result.valid = true;
    return result;
}

// convex-convex collision
Collisions::ContactManifold convex_convex(const RigidBody &a, std::size_t a_idx,
                                          const RigidBody &b,
                                          std::size_t b_idx) {

    Collisions::ContactManifold manifold = {a_idx, b_idx};

    // phase 1: gjk boolean intersection test
    GJKResult gjk_result = gjk(a, b);
    if (!gjk_result.intersecting)
        return manifold;

    // phase 2: epa penetration depth + normal
    EPAResult epa_result = epa(a, b, gjk_result.simplex);
    if (!epa_result.valid)
        return manifold;

    const Vec3 &normal = epa_result.normal;
    const float depth = epa_result.depth;

    // small contacts should be removed
    if (depth < 1e-6f)
        return manifold;

    // phase 3: build a multi-point manifold via face clipping
    // fixes single-contact-point rocking problem for shapes
    // that have faces (boxes, eventually polygons with face data).
    std::size_t clipped =
        Collisions::clip_manifold(a, a_idx, b, b_idx, normal, depth, manifold);

    if (clipped > 0)
        return manifold;

    // single EPA contact point (spheres, polygons without
    // face data, etc)
    Vec3 r1 = epa_result.contact_a - a.position;
    Vec3 r2 = epa_result.contact_b - b.position;

    // penetration is negative when overlapping (engine convention)
    manifold.points[0] = {r1, r2, normal, -depth, 0};
    manifold.num_points = 1;

    return manifold;
}

} // namespace PhysicsEngine::Collisions::GJK
