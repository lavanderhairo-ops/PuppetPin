// Puppet Pin - as-rigid-as-possible mesh deformation driven by pins.
//
// Static solve (each frame on its own, frame-order independent):
//   1. an as-similar-as-possible linear solve (Igarashi et al. 2005, step 1) gives a
//      rotation-aware starting shape,
//   2. local/global ARAP iterations with per-triangle rotations (Liu et al. 2008)
//      remove the unwanted scaling.
// Dynamic step (Smooth Follow / Elastic): each frame the exact pinned pose is solved, and every
// vertex follows it through a damped spring (implicit, so always stable). The spring's effect
// fades in with the distance (along the mesh) from the nearest pin or anchor, so pinned areas
// stay exact while free parts lag, overshoot and settle. The lag is also limited by that distance
// (like a flexible rod held at the pins), so the shape never folds over where the lag fades in.
//
// Pins are soft constraints: a pin without radius drags the exact point it was placed on
// (barycentric binding), a pin with a radius locks every vertex inside the radius to the
// pin's rigid transform (position, rotation, scale). A pin with an influence only pulls the
// area within that distance (along the mesh); beyond every such reach the image keeps the
// pose it has without those pins. A weight map can additionally anchor vertices to their
// rest position or stiffen triangles.
#pragma once

#include "pp_mesh.h"
#include "pp_sparse.h"

#include <memory>
#include <mutex>

namespace pp {

struct PinInput {
    Vec2 src;               // rest position (canonical)
    Vec2 dst;               // current position (canonical)
    double radius = 0.0;    // lock radius (canonical); 0 = point pin
    double rotation = 0.0;  // radians, applied to the locked region
    double scale = 1.0;     // applied to the locked region
    double strength = 1.0;  // pull strength in [0,1]
    int layer = 0;          // where parts overlap, the part around this pin goes over lower layers
    double influence = 0.0; // how far (canonical, along the mesh) the pull reaches; 0 = no limit
    double falloff = 0.5;   // share of the influence over which the pull fades out
};

struct SolverParams {
    double rigidity = 1.0;  // 1: as-rigid-as-possible, 0: allows local scaling
    int iterations = 12;
    // Optional material maps (sizes must match the mesh).
    const std::vector<float>* anchor = nullptr;     // per vertex 0..1: hold towards the rest pose (1 = locked)
    const std::vector<float>* stiffness = nullptr;  // per triangle multiplier on the shape stiffness (>= 0)
    // Stiffness multiplier for the triangles right around a point pin: spreads a hard pull over the
    // surrounding material instead of stretching the pinned triangle into a spike. 1 = off.
    double pinStiffen = 16.0;
};

struct DynamicsParams {
    bool elastic = true;    // false: Smooth Follow (lags behind, no bounce)
    double firmness = 2.0;  // how fast the material responds: wobbles per second
    double damping = 0.15;  // 0 = wobbles for long, 1 = no overshoot (Elastic)
    double fps = 24.0;
    double reach = 0.25;    // distance (share of the mesh size) over which the jiggle fades in from pins
    double maxSlope = 0.5;  // a point may trail at most this many pixels per pixel of distance from the pins
    int substeps = 4;
};

struct DynamicState {
    std::vector<Vec2> pos, vel;  // per mesh vertex: the spring-driven positions and velocities
    std::vector<Vec2> target;    // the pinned pose of the previous frame
    bool empty() const { return pos.empty(); }
};

struct Deformation {
    std::vector<Vec2> pos;      // deformed vertex positions
    std::vector<int> triLayer;  // draw order per triangle where parts overlap (higher on top)
    std::vector<int> pinTri;    // triangle each pin is bound to (-1: pin is off the mesh)
    std::vector<Vec2> pinRest;  // rest point each pin is bound to
};

class PuppetSolver {
public:
    explicit PuppetSolver(std::shared_ptr<const Mesh> mesh);
    const Mesh& mesh() const { return *mesh_; }
    const std::shared_ptr<const Mesh>& meshPtr() const { return mesh_; }

    // Static solve. Thread-safe.
    void solve(const std::vector<PinInput>& pins, const SolverParams& sp, Deformation& out) const;

    // Advances `state` by one frame to `pins`. An empty state starts at the static solution,
    // at rest. Thread-safe.
    void step(const std::vector<PinInput>& pins, const SolverParams& sp, const DynamicsParams& dp,
              DynamicState& state, Deformation& out) const;

private:
    struct Binding;
    std::shared_ptr<const Binding> binding(const std::vector<PinInput>& pins, const SolverParams& sp) const;
    std::shared_ptr<const Binding> buildBinding(const std::vector<PinInput>& pins, const SolverParams& sp) const;
    std::vector<double> jiggleWeights(const Binding& B, const SolverParams& sp, double reach,
                                      std::vector<double>* distOut = nullptr) const;
    void assembleLaplacian(const Binding& B, double reg, SparseSymBuilder& lap) const;
    void fillInfo(const Binding& B, const std::vector<PinInput>& pins, Deformation& out) const;
    void solveCore(const std::vector<PinInput>& pins, const SolverParams& sp, const std::vector<Vec2>* anchorTarget,
                   const std::vector<float>* restBlend, Deformation& out) const;
    std::shared_ptr<const std::vector<float>> reachMap(const std::vector<PinInput>& pins) const;
    bool heldAnchor(const std::vector<PinInput>& pins, const SolverParams& sp,
                    std::shared_ptr<const std::vector<float>>& reach, std::vector<float>& held,
                    std::vector<float>* stiff = nullptr) const;
    void constraintRhs(const Binding& B, const std::vector<PinInput>& pins, const std::vector<Vec2>* anchorTarget,
                       std::vector<double>& cx, std::vector<double>& cy) const;
    void rotationRhs(const Binding& B, const std::vector<double>& X, const std::vector<double>& Y, double rigidity,
                     std::vector<double>& rx, std::vector<double>& ry) const;

    std::shared_ptr<const Mesh> mesh_;
    std::vector<std::array<double, 3>> cot_;  // per triangle: weight of the edge opposite vertex k
    double area_ = 0.0;                       // total mesh area
    std::vector<std::vector<std::pair<int, double>>> adj_;  // vertex neighbours with edge lengths
    mutable std::mutex mtx_;
    mutable std::vector<std::pair<std::vector<double>, std::shared_ptr<const Binding>>> cache_;
    mutable std::vector<double> reachKey_;
    mutable std::shared_ptr<const std::vector<float>> reach_;
};

}  // namespace pp
