// Puppet Pin - rendering of the deformed mesh and image helpers.
#pragma once

#include "pp_mesh.h"

#include <cstddef>
#include <functional>

namespace pp {

// A float RGBA (premultiplied) image in pixel coordinates, OFX style: rows go up in y,
// rowBytes may be negative.
struct ImageView {
    float* data = nullptr;
    int x1 = 0, y1 = 0, x2 = 0, y2 = 0;  // bounds, x2/y2 exclusive
    ptrdiff_t rowBytes = 0;

    bool valid() const { return data != nullptr && x2 > x1 && y2 > y1; }
    bool contains(int x, int y) const { return x >= x1 && x < x2 && y >= y1 && y < y2; }
    float* pixel(int x, int y) const
    {
        return reinterpret_cast<float*>(reinterpret_cast<char*>(data) + (ptrdiff_t)(y - y1) * rowBytes) +
               (ptrdiff_t)(x - x1) * 4;
    }
};

// Maps canonical coordinates to pixel coordinates: pixel = canonical * s + o.
struct Xform {
    double sx = 1.0, sy = 1.0, ox = 0.0, oy = 0.0;
    Vec2 apply(const Vec2& c) const { return {c.x * sx + ox, c.y * sy + oy}; }
    Vec2 invert(const Vec2& p) const { return {(p.x - ox) / sx, (p.y - oy) / sy}; }
};

struct RectI {
    int x1 = 0, y1 = 0, x2 = 0, y2 = 0;
};

// Samples the alpha of src over the canonical rectangle [fx, fx+fw] x [fy, fy+fh] into
// a grid of cells of canonical size `cell`. fullFrame ignores alpha (everything covered).
void buildMaskGrid(const ImageView& src, const Xform& toSrc, double fx, double fy, double fw, double fh,
                   double cell, bool fullFrame, MaskGrid& out);

struct RenderJob {
    const ImageView* src = nullptr;
    const ImageView* dst = nullptr;
    RectI window;                          // pixels of dst to write
    const Mesh* mesh = nullptr;
    const std::vector<Vec2>* deformed = nullptr;  // canonical, one per mesh vertex
    const std::vector<int>* triLayer = nullptr;   // optional
    Xform toSrc, toDst;                    // canonical -> pixel
    int aa = 1;                            // supersampling per axis (1..4)
    bool keepUnmeshed = true;              // pass through pixels the rest mesh does not cover
    int threads = 0;                       // 0 = hardware concurrency
    // Image layers: triangle t shows sources[mesh->triSource[t]] (src when missing). Layer 0 is src.
    const std::vector<const ImageView*>* sources = nullptr;
    int onlySource = -1;                   // >= 0: render only that layer's triangles
    // Instead of the image: writes a per-vertex 2D value (interpolated) into red/green, alpha = coverage
    // (motion vectors).
    const std::vector<Vec2>* attribute = nullptr;
};

// Renders the deformed mesh, texture-mapping the source by the rest positions.
void renderPuppet(const RenderJob& job);

// Copies src to dst inside window (transparent where src has no pixels).
void copyImage(const ImageView& src, const ImageView& dst, const RectI& window);

// Debug burn-in: mesh wireframe and pin markers drawn into dst (pixel space via toDst).
struct OverlayPin {
    Vec2 pos;
    bool selected = false;
};
void drawBurnIn(const ImageView& dst, const RectI& window, const Mesh* mesh, const std::vector<Vec2>* positions,
                const std::vector<OverlayPin>& pins, const Xform& toDst, double lineAlpha);

// Runs f(i) for i in [0, n) on worker threads.
void parallelFor(int n, int threads, const std::function<void(int)>& f);

}  // namespace pp
