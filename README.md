# constraint-physics-engine

A 3D rigid body engine in C++17, written from the Box2D GDC slides — constraints as Jacobian rows, solved with projected Gauss-Seidel, with warm starting from a contact cache.

I wanted to try my hand at a proper 3D solver. This solver isn't polished by any means (interaction is about as barebones as it gets), but this provided a fantastic opportunity to gain a deeper understanding about 3D representations of the physics I have been working with.

## What this is

A from-scratch implementation rather than a library build: the maths types (`Vec3`, `Mat<R,C>`, `DynMat`, `Quaternion`), the collision detection, the solver and the interaction layer are all written here. raylib is the only dependency, and only for the window, input and debug rendering.

The point was to follow the derivations in Erin Catto's GDC decks closely enough that the code reads against them — the constraint row layout, the `eta`/`zeta` split, the PGS loop and the warm-starting pass are commented with the equation and algorithm numbers they come from.

**Current capabilities:**
- **Rigid bodies**: 3D, 6-DOF. Position + quaternion orientation, symplectic Euler integration, world-space inertia tensor rebuilt each frame from the orientation, static bodies via zero inverse mass
- **Constraint solver**: scalar constraint rows carrying Jacobian blocks, the precomputed `B = M⁻¹Jᵀ` blocks, effective inverse mass and impulse bounds; projected Gauss-Seidel with per-row clamping, and warm starting from the previous frame's accumulated impulses
- **Contacts**: one normal row (lower-bounded at zero, Baumgarte bias on penetration) and two friction rows (tangent basis built off the normal, bounded by a Coulomb estimate) per contact point
- **Contact caching**: manifold points carry stable ids, so accumulated impulses survive frame to frame and are fed back in as the warm start
- **Collision detection**:
  - Broadphase: AABBs from the current transform, brute-force pair test, static-static pairs skipped
  - Narrowphase: analytic sphere-sphere, box-sphere, box-box and box/sphere-plane paths, with GJK + EPA as the general convex-convex fallback (including polygon shapes)
  - Manifold generation: support-face extraction and Sutherland-Hodgman clipping after EPA, so a resting box gets up to 8 contact points instead of one
- **Joints**: a distance constraint in three flavours — spring (soft bias proportional to displacement), damper (pure velocity row) and rod (Baumgarte-corrected, rigid)
- **Raycasting**: closest-hit across the scene, with per-shape ray-box and ray-sphere tests, returning the world and local hit point plus surface normal
- **Interaction**: place boxes and spheres (static or dynamic) on the surface under the cursor, wire springs, dampers and rods between two picked bodies, grab and drag a body with a mouse spring, delete bodies under the crosshair
- **Rendering**: raylib debug draw with pre-built unit meshes, an orbital/FPS/locked scene camera, and a HUD that reflects the current tool


## Building

Requires a C++17 compiler and raylib (via Homebrew on macOS — the Makefile picks up `brew --prefix` and links the OpenGL/Cocoa/IOKit/CoreVideo frameworks).

```bash
make
```

```bash
make run
```

## Running

The scene starts as a static ground plane with three boxes stacked above it.

| Key | Action |
| --- | --- |
| `Space` | pause / unpause the simulation |
| `B` | body tool — `1` box, `2` sphere, `S` toggles static placement, click a surface to place |
| `F` | force tool — `1` spring, `2` damper, `3` rod; click two bodies, right-click cancels |
| `G` | grab tool — hold left-click on a body to drag it with a mouse spring |
| `Backspace` | delete the body under the crosshair |
| `W A S D Q E` | fly the camera (hold `Shift` to speed up) |
| `Esc` | release the mouse cursor |

Interaction runs while paused, so a scene can be built up a body at a time before letting it go.

## Project structure

```
constraint-physics-engine/
├── include/
│   ├── vec3.h, mat.h, dynmat.h,     # hand-rolled maths types
│   │   quaternion.h, math_utils.h
│   ├── rigid_body.h, shape.h        # body state, box/sphere/polygon shapes
│   ├── world.h, solver.h,           # the step loop and PGS
│   │   solver_settings.h
│   ├── constraint.h,                # constraint rows, contact row assembly,
│   │   distance_constraint.h        # spring/damper/rod joint
│   ├── contact_cache.h              # per-pair impulse cache for warm starting
│   ├── aabb.h, broadphase.h         # broadphase
│   ├── collision.h, gjk.h,          # narrowphase + manifold generation
│   │   manifold_clip.h
│   ├── raycast.h, interaction.h     # picking and the editing tools
│   └── scene_camera.h, debug_draw.h # raylib rendering
├── src/                             # mirrors include/, plus main.cpp
├── Makefile
└── build/
```

## References

- [Catto, *Iterative Dynamics with Temporal Coherence* (GDC 2005)](https://box2d.org/files/ErinCatto_IterativeDynamics_GDC2005.pdf) — the constraint row formulation, the PGS solver (Algorithm 4) and warm starting (Algorithm 5). Equation numbers in the source refer to this one.
- [Catto, *Fast and Simple Physics using Sequential Impulses* (GDC 2006)](https://box2d.org/files/ErinCatto_SequentialImpulses_GDC2006.pdf)
- [Catto, *Computing Distance and Contact Manifolds* (GDC 2007)](https://box2d.org/files/ErinCatto_ContactManifolds_GDC2007.pdf) — GJK, and the support-face clipping used to build multi-point manifolds.
