// Puppet Pin - small 2D geometry helpers shared by the mesh, solver and renderer.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

namespace pp {

struct Vec2 {
    double x = 0.0, y = 0.0;
    Vec2() = default;
    Vec2(double x_, double y_) : x(x_), y(y_) {}
    Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
    Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
    Vec2 operator-() const { return {-x, -y}; }
    Vec2 operator*(double s) const { return {x * s, y * s}; }
    Vec2 operator/(double s) const { return {x / s, y / s}; }
    Vec2& operator+=(const Vec2& o) { x += o.x; y += o.y; return *this; }
    Vec2& operator-=(const Vec2& o) { x -= o.x; y -= o.y; return *this; }
    Vec2& operator*=(double s) { x *= s; y *= s; return *this; }
};

inline Vec2 operator*(double s, const Vec2& v) { return v * s; }
inline double dot(const Vec2& a, const Vec2& b) { return a.x * b.x + a.y * b.y; }
inline double cross(const Vec2& a, const Vec2& b) { return a.x * b.y - a.y * b.x; }
inline double length2(const Vec2& a) { return dot(a, a); }
inline double length(const Vec2& a) { return std::sqrt(dot(a, a)); }
inline Vec2 perp(const Vec2& a) { return {-a.y, a.x}; }
inline Vec2 rotate(const Vec2& a, double c, double s) { return {c * a.x - s * a.y, s * a.x + c * a.y}; }

using Tri = std::array<int, 3>;

// Barycentric coordinates of p in triangle abc. Returns false for degenerate triangles.
inline bool barycentric(const Vec2& a, const Vec2& b, const Vec2& c, const Vec2& p, double out[3])
{
    const double d = cross(b - a, c - a);
    if (std::fabs(d) < 1e-300) return false;
    out[1] = cross(p - a, c - a) / d;
    out[2] = cross(b - a, p - a) / d;
    out[0] = 1.0 - out[1] - out[2];
    return true;
}

// Squared distance from p to segment ab, and the closest point.
inline double segmentDist2(const Vec2& p, const Vec2& a, const Vec2& b, Vec2* closest = nullptr)
{
    const Vec2 ab = b - a;
    const double l2 = length2(ab);
    double t = l2 > 0.0 ? dot(p - a, ab) / l2 : 0.0;
    t = std::min(1.0, std::max(0.0, t));
    const Vec2 c = a + ab * t;
    if (closest) *closest = c;
    return length2(p - c);
}

// Uniform-grid spatial index over a triangle set, for point location queries.
class TriLocator {
public:
    void build(const std::vector<Vec2>& pos, const std::vector<Tri>& tris);
    bool empty() const { return tris_.empty(); }

    // First triangle containing p (barycentric tolerance eps). -1 if none.
    int locate(const Vec2& p, double bary[3], double eps = 1e-9) const;

    // All triangles containing p, as (triangle, barycentric) pairs.
    void locateAll(const Vec2& p, std::vector<std::pair<int, std::array<double, 3>>>& out, double eps = 1e-9) const;

    // Closest triangle within maxDist of p. bary describes the closest point on that triangle.
    int nearest(const Vec2& p, double maxDist, double bary[3], double* dist = nullptr) const;

    const std::vector<Vec2>& positions() const { return pos_; }
    const std::vector<Tri>& triangles() const { return tris_; }

private:
    template <class F> void forCells(double x0, double y0, double x1, double y1, F&& f) const;

    std::vector<Vec2> pos_;
    std::vector<Tri> tris_;
    double gx0_ = 0.0, gy0_ = 0.0, inv_ = 1.0;
    int nx_ = 0, ny_ = 0;
    std::vector<int> start_, items_;
};

}  // namespace pp
