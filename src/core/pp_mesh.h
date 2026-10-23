// Puppet Pin - triangle mesh generation from an image's coverage (alpha).
//
// The mesh hugs the non-transparent pixels: the alpha is thresholded, grown by an
// "expansion" distance, its outline is traced and simplified, the interior is filled
// with a regular lattice and everything is triangulated with an exact-arithmetic
// conforming Delaunay triangulation. Separate islands become separate mesh components,
// so a pin on one part never drags an unconnected part along.
#pragma once

#include "pp_geom.h"

#include <cstdint>
#include <string>

namespace pp {

// Coverage samples on a regular grid in canonical coordinates.
struct MaskGrid {
    int w = 0, h = 0;
    double x0 = 0.0, y0 = 0.0;  // canonical coordinate of the lower-left corner of cell (0,0)
    double cell = 1.0;          // canonical size of a cell
    std::vector<float> v;       // coverage in [0,1], row-major, row 0 at the bottom (y up)
    float at(int x, int y) const { return v[(size_t)y * w + x]; }
};

struct MeshParams {
    int targetTriangles = 600;  // approximate triangle count over the covered area
    double expansion = 3.0;     // how far the mesh extends past the alpha edge (canonical units)
    double threshold = 0.05;    // alpha above this counts as covered
};

struct Mesh {
    std::vector<Vec2> verts;         // rest positions (canonical)
    std::vector<Tri> tris;           // counter-clockwise (y up)
    std::vector<uint8_t> boundary;   // 1 if the vertex lies on the outline
    std::vector<int> vertComp;       // connected component per vertex
    std::vector<int> triComp;        // connected component per triangle
    int numComps = 0;
    double edgeLength = 0.0;         // nominal edge length (canonical)
    std::vector<int> triSource;      // image layer each triangle shows (empty: all layer 0)
    TriLocator locator;              // rest-space point location

    bool empty() const { return tris.empty(); }
    // Computes components and the locator from verts/tris.
    void finalize();
};

struct MeshStats {
    int contourPoints = 0;
    int interiorPoints = 0;
    int splitPoints = 0;
    int missingConstraints = 0;
    int loops = 0;
    double ms = 0.0;
};

// Adds `part` (the mesh of image layer `source`) to `dst`; call dst.finalize() afterwards. Layers
// added later are drawn on top (their triangles come later).
void appendMesh(Mesh& dst, const Mesh& part, int source);

// Rest-space triangle at p, the top layer's where layers overlap (-1: none). Falls back to the
// nearest triangle within maxDist.
int locateTop(const Mesh& m, const Vec2& p, double bary[3], double maxDist = 0.0);

// Builds the mesh. Returns false if nothing is covered.
bool buildMesh(const MaskGrid& mask, const MeshParams& params, Mesh& out, MeshStats* stats = nullptr);

// Content hash of a mask grid (used to cache meshes across frames).
uint64_t hashMask(const MaskGrid& mask);

}  // namespace pp
