#include "pp_mesh.h"

#include <chrono>
#include <cstring>
#include <limits>
#include <memory>
#include <unordered_set>

namespace pp {
namespace {

constexpr double kInf = 1e30;

// ---------------------------------------------------------------------------------------
// Euclidean distance transform (Felzenszwalb & Huttenlocher), squared distances.

void dt1d(const double* f, int n, double* d, int* v, double* z)
{
    int k = -1;
    for (int q = 0; q < n; ++q) {
        if (f[q] >= kInf) continue;
        if (k < 0) { k = 0; v[0] = q; z[0] = -kInf; z[1] = kInf; continue; }
        double s = 0.0;
        while (true) {
            const int p = v[k];
            s = ((f[q] + (double)q * q) - (f[p] + (double)p * p)) / (2.0 * (q - p));
            if (s <= z[k]) {
                if (--k < 0) break;
            } else {
                break;
            }
        }
        if (k < 0) { k = 0; v[0] = q; z[0] = -kInf; z[1] = kInf; continue; }
        ++k;
        v[k] = q;
        z[k] = s;
        z[k + 1] = kInf;
    }
    if (k < 0) {
        for (int q = 0; q < n; ++q) d[q] = kInf;
        return;
    }
    k = 0;
    for (int q = 0; q < n; ++q) {
        while (z[k + 1] < q) ++k;
        const double dq = q - v[k];
        d[q] = dq * dq + f[v[k]];
    }
}

// Squared distance from every sample to the nearest sample with feature != 0.
void edt2d(const std::vector<uint8_t>& feature, int W, int H, std::vector<double>& out)
{
    out.assign((size_t)W * H, kInf);
    for (size_t i = 0; i < out.size(); ++i)
        if (feature[i]) out[i] = 0.0;
    const int n = std::max(W, H);
    std::vector<double> f(n), d(n), z(n + 1);
    std::vector<int> v(n);
    for (int x = 0; x < W; ++x) {
        for (int y = 0; y < H; ++y) f[y] = out[(size_t)y * W + x];
        dt1d(f.data(), H, d.data(), v.data(), z.data());
        for (int y = 0; y < H; ++y) out[(size_t)y * W + x] = d[y];
    }
    for (int y = 0; y < H; ++y) {
        double* row = &out[(size_t)y * W];
        std::copy(row, row + W, f.begin());
        dt1d(f.data(), W, row, v.data(), z.data());
    }
}

// Clears 4-connected islands with fewer than minSize samples.
void removeSpecks(std::vector<uint8_t>& in, int W, int H, int minSize)
{
    std::vector<int> label(in.size(), -1);
    std::vector<int> stack, members;
    for (int start = 0; start < (int)in.size(); ++start) {
        if (!in[start] || label[start] >= 0) continue;
        stack.assign(1, start);
        members.clear();
        label[start] = start;
        while (!stack.empty()) {
            const int i = stack.back();
            stack.pop_back();
            members.push_back(i);
            const int x = i % W, y = i / W;
            const int nb[4] = {x > 0 ? i - 1 : -1, x + 1 < W ? i + 1 : -1, y > 0 ? i - W : -1, y + 1 < H ? i + W : -1};
            for (int j : nb)
                if (j >= 0 && in[j] && label[j] < 0) { label[j] = start; stack.push_back(j); }
        }
        if ((int)members.size() < minSize)
            for (int i : members) in[i] = 0;
    }
}

// ---------------------------------------------------------------------------------------
// Marching squares: closed iso-contours of s at level L, inside (s < L) on the left.

std::vector<std::vector<Vec2>> traceContours(const std::vector<double>& s, int W, int H, double L)
{
    auto S = [&](int x, int y) { return s[(size_t)y * W + x]; };
    auto In = [&](int x, int y) { return S(x, y) < L; };
    std::vector<int> hId((size_t)W * H, -1), vId((size_t)W * H, -1);
    std::vector<Vec2> pts;
    std::vector<int> next;
    auto crossT = [&](double a, double b) {
        const double t = (L - a) / (b - a);
        return std::min(0.999, std::max(0.001, t));
    };
    auto hPt = [&](int x, int y) {
        int& id = hId[(size_t)y * W + x];
        if (id < 0) {
            id = (int)pts.size();
            pts.push_back({x + crossT(S(x, y), S(x + 1, y)), (double)y});
            next.push_back(-1);
        }
        return id;
    };
    auto vPt = [&](int x, int y) {
        int& id = vId[(size_t)y * W + x];
        if (id < 0) {
            id = (int)pts.size();
            pts.push_back({(double)x, y + crossT(S(x, y), S(x, y + 1))});
            next.push_back(-1);
        }
        return id;
    };

    for (int y = 0; y + 1 < H; ++y) {
        for (int x = 0; x + 1 < W; ++x) {
            const bool c0 = In(x, y), c1 = In(x + 1, y), c2 = In(x + 1, y + 1), c3 = In(x, y + 1);
            const int idx = (int)c0 | ((int)c1 << 1) | ((int)c2 << 2) | ((int)c3 << 3);
            if (idx == 0 || idx == 15) continue;
            const Vec2 corner[4] = {{(double)x, (double)y}, {(double)x + 1, (double)y},
                                    {(double)x + 1, (double)y + 1}, {(double)x, (double)y + 1}};
            const bool cin[4] = {c0, c1, c2, c3};
            // Edges: 0 bottom (c0-c1), 1 right (c1-c2), 2 top (c3-c2), 3 left (c0-c3).
            auto edgePt = [&](int e) {
                switch (e) {
                    case 0: return hPt(x, y);
                    case 1: return vPt(x + 1, y);
                    case 2: return hPt(x, y + 1);
                    default: return vPt(x, y);
                }
            };
            auto emit = [&](int ea, int eb, int testCorner) {
                int a = edgePt(ea), b = edgePt(eb);
                const Vec2 P = pts[a], Q = pts[b];
                if (testCorner < 0) {
                    double best = -1.0;
                    for (int c = 0; c < 4; ++c) {
                        const double sd = std::fabs(cross(Q - P, corner[c] - P));
                        if (sd > best) { best = sd; testCorner = c; }
                    }
                }
                const double side = cross(Q - P, corner[testCorner] - P);
                const bool in = cin[testCorner];
                if ((in && side < 0.0) || (!in && side > 0.0)) std::swap(a, b);
                next[a] = b;
            };
            if (idx == 5 || idx == 10) {
                const bool centerIn = 0.25 * (S(x, y) + S(x + 1, y) + S(x + 1, y + 1) + S(x, y + 1)) < L;
                if ((idx == 5) == centerIn) {
                    emit(0, 1, 1);
                    emit(2, 3, 3);
                } else {
                    emit(0, 3, 0);
                    emit(1, 2, 2);
                }
            } else {
                int e[4], ne = 0;
                if (c0 != c1) e[ne++] = 0;
                if (c1 != c2) e[ne++] = 1;
                if (c3 != c2) e[ne++] = 2;
                if (c0 != c3) e[ne++] = 3;
                if (ne == 2) emit(e[0], e[1], -1);
            }
        }
    }

    std::vector<std::vector<Vec2>> loops;
    std::vector<char> used(pts.size(), 0);
    for (int i = 0; i < (int)pts.size(); ++i) {
        if (used[i] || next[i] < 0) continue;
        std::vector<Vec2> loop;
        int j = i;
        bool ok = true;
        while (!used[j]) {
            used[j] = 1;
            loop.push_back(pts[j]);
            j = next[j];
            if (j < 0) { ok = false; break; }
        }
        if (ok && j == i && loop.size() >= 3) loops.push_back(std::move(loop));
    }
    return loops;
}

// Douglas-Peucker for a closed polyline.
std::vector<Vec2> simplifyClosed(const std::vector<Vec2>& in, double tol)
{
    const int n = (int)in.size();
    if (n <= 4) return in;
    int far = 0;
    double best = -1.0;
    for (int i = 1; i < n; ++i) {
        const double d = length2(in[i] - in[0]);
        if (d > best) { best = d; far = i; }
    }
    std::vector<char> keep(n, 0);
    keep[0] = keep[far] = 1;
    std::vector<std::pair<int, int>> stack = {{0, far}, {far, n}};
    const double tol2 = tol * tol;
    while (!stack.empty()) {
        const auto [a, b] = stack.back();
        stack.pop_back();
        if (b - a < 2) continue;
        const Vec2& A = in[a];
        const Vec2& B = in[b % n];
        int m = -1;
        double md = tol2;
        for (int i = a + 1; i < b; ++i) {
            const double d = segmentDist2(in[i], A, B);
            if (d > md) { md = d; m = i; }
        }
        if (m >= 0) {
            keep[m] = 1;
            stack.push_back({a, m});
            stack.push_back({m, b});
        }
    }
    std::vector<Vec2> out;
    for (int i = 0; i < n; ++i)
        if (keep[i]) out.push_back(in[i]);
    return out;
}

std::vector<Vec2> subdivideClosed(const std::vector<Vec2>& in, double maxLen)
{
    std::vector<Vec2> out;
    const int n = (int)in.size();
    for (int i = 0; i < n; ++i) {
        const Vec2 a = in[i], b = in[(i + 1) % n];
        out.push_back(a);
        const int k = (int)std::ceil(length(b - a) / maxLen);
        for (int j = 1; j < k; ++j) out.push_back(a + (b - a) * ((double)j / k));
    }
    return out;
}

// Drops outline vertices that make very short edges, when doing so only cuts a sliver off
// the region (never by more than ~half a cell, so covered pixels stay covered).
std::vector<Vec2> mergeShort(std::vector<Vec2> p, double minLen)
{
    bool changed = true;
    while (changed && p.size() > 3) {
        changed = false;
        for (size_t i = 0; i < p.size() && p.size() > 3;) {
            const size_t n = p.size();
            const Vec2 a = p[(i + n - 1) % n], b = p[i], c = p[(i + 1) % n];
            if (std::min(length(b - a), length(c - b)) >= minLen) { ++i; continue; }
            const double dev = std::sqrt(segmentDist2(b, a, c));
            const double side = cross(c - a, b - a);  // < 0: b bulges outwards
            if ((side <= 0.0 && dev <= 0.6) || dev <= 0.12) {
                p.erase(p.begin() + i);
                changed = true;
            } else {
                ++i;
            }
        }
    }
    return p;
}

double signedArea(const std::vector<Vec2>& p)
{
    double a = 0.0;
    for (size_t i = 0, n = p.size(); i < n; ++i) a += cross(p[i], p[(i + 1) % n]);
    return 0.5 * a;
}

// ---------------------------------------------------------------------------------------
// Incremental Delaunay triangulation (Bowyer-Watson) with exact integer predicates.

struct IP {
    int64_t x, y;
    bool operator==(const IP& o) const { return x == o.x && y == o.y; }
};

inline int64_t orient2(const IP& a, const IP& b, const IP& c)
{
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

inline int inCircle(const IP& a, const IP& b, const IP& c, const IP& d)
{
    const __int128 adx = a.x - d.x, ady = a.y - d.y;
    const __int128 bdx = b.x - d.x, bdy = b.y - d.y;
    const __int128 cdx = c.x - d.x, cdy = c.y - d.y;
    const __int128 ad = adx * adx + ady * ady;
    const __int128 bd = bdx * bdx + bdy * bdy;
    const __int128 cd = cdx * cdx + cdy * cdy;
    const __int128 det = ad * (bdx * cdy - cdx * bdy) + bd * (cdx * ady - adx * cdy) + cd * (adx * bdy - bdx * ady);
    return det > 0 ? 1 : (det < 0 ? -1 : 0);
}

inline uint64_t edgeKey(int a, int b)
{
    if (a > b) std::swap(a, b);
    return ((uint64_t)(uint32_t)a << 32) | (uint32_t)b;
}

class Delaunay {
public:
    struct T {
        int v[3];
        int n[3];  // n[i] is the neighbour across the edge opposite v[i]
    };
    std::vector<IP> P;
    std::vector<T> tri;
    std::vector<char> alive;

    void init(int64_t minx, int64_t miny, int64_t maxx, int64_t maxy)
    {
        const int64_t cx = (minx + maxx) / 2, cy = (miny + maxy) / 2;
        const int64_t r = std::max(maxx - minx, maxy - miny) / 2 + 64;
        const int64_t M = r * 32;
        P = {{cx - M, cy - M}, {cx + M, cy - M}, {cx, cy + M}};
        tri.assign(1, T{{0, 1, 2}, {-1, -1, -1}});
        alive.assign(1, 1);
        mark_.assign(1, 0);
        free_.clear();
        last_ = 0;
    }

    static bool isSuper(int v) { return v < 3; }

    // Inserts p and returns its vertex index (or that of an identical existing vertex).
    int insert(const IP& p)
    {
        const int t0 = locate(p);
        if (t0 < 0) return -1;
        for (int k = 0; k < 3; ++k)
            if (P[tri[t0].v[k]] == p) return tri[t0].v[k];

        const int vi = (int)P.size();
        P.push_back(p);
        ++stamp_;
        cavity_.clear();
        cavity_.push_back(t0);
        mark_[t0] = stamp_;
        for (size_t q = 0; q < cavity_.size(); ++q) {
            const T tr = tri[cavity_[q]];
            for (int i = 0; i < 3; ++i) {
                const int nb = tr.n[i];
                if (nb < 0 || mark_[nb] == stamp_) continue;
                const T& nt = tri[nb];
                if (inCircle(P[nt.v[0]], P[nt.v[1]], P[nt.v[2]], p) > 0) {
                    mark_[nb] = stamp_;
                    cavity_.push_back(nb);
                }
            }
        }

        // Cavity boundary; grow the cavity if some boundary edge is not strictly visible.
        for (int guard = 0;; ++guard) {
            bnd_.clear();
            int grow = -1;
            for (int ct : cavity_) {
                const T& tr = tri[ct];
                for (int i = 0; i < 3; ++i) {
                    const int nb = tr.n[i];
                    if (nb >= 0 && mark_[nb] == stamp_) continue;
                    const int a = tr.v[(i + 1) % 3], b = tr.v[(i + 2) % 3];
                    if (orient2(P[a], P[b], p) <= 0 && nb >= 0) { grow = nb; break; }
                    int slot = -1;
                    if (nb >= 0)
                        for (int j = 0; j < 3; ++j)
                            if (tri[nb].n[j] == ct) slot = j;
                    bnd_.push_back({a, b, nb, slot});
                }
                if (grow >= 0) break;
            }
            if (grow < 0 || guard > 256) break;
            mark_[grow] = stamp_;
            cavity_.push_back(grow);
        }

        for (int ct : cavity_) {
            alive[ct] = 0;
            free_.push_back(ct);
        }
        newT_.clear();
        for (const BEdge& e : bnd_) {
            const int nt = alloc();
            tri[nt] = T{{vi, e.a, e.b}, {e.nb, -1, -1}};
            if (e.nb >= 0 && e.slot >= 0) tri[e.nb].n[e.slot] = nt;
            newT_.push_back(nt);
        }
        for (size_t i = 0; i < newT_.size(); ++i) {
            T& ti = tri[newT_[i]];
            for (size_t j = 0; j < newT_.size(); ++j) {
                if (i == j) continue;
                const T& tj = tri[newT_[j]];
                if (tj.v[1] == ti.v[2]) ti.n[1] = newT_[j];
                if (tj.v[2] == ti.v[1]) ti.n[2] = newT_[j];
            }
        }
        last_ = newT_.empty() ? -1 : newT_[0];
        return vi;
    }

    void edgeSet(std::unordered_set<uint64_t>& s) const
    {
        s.clear();
        s.reserve(tri.size() * 2);
        for (size_t t = 0; t < tri.size(); ++t) {
            if (!alive[t]) continue;
            for (int k = 0; k < 3; ++k) s.insert(edgeKey(tri[t].v[k], tri[t].v[(k + 1) % 3]));
        }
    }

private:
    struct BEdge {
        int a, b, nb, slot;
    };

    int alloc()
    {
        if (!free_.empty()) {
            const int t = free_.back();
            free_.pop_back();
            alive[t] = 1;
            return t;
        }
        tri.push_back(T{});
        alive.push_back(1);
        mark_.push_back(0);
        return (int)tri.size() - 1;
    }

    int locate(const IP& p)
    {
        int t = last_;
        if (t < 0 || t >= (int)tri.size() || !alive[t]) {
            t = -1;
            for (int i = (int)tri.size() - 1; i >= 0; --i)
                if (alive[i]) { t = i; break; }
        }
        const int maxSteps = (int)tri.size() + 64;
        for (int step = 0; t >= 0 && step < maxSteps; ++step) {
            const T& tr = tri[t];
            int nextT = -2;
            const int s0 = step % 3;
            for (int k = 0; k < 3; ++k) {
                const int i = (s0 + k) % 3;
                if (orient2(P[tr.v[(i + 1) % 3]], P[tr.v[(i + 2) % 3]], p) < 0) { nextT = tr.n[i]; break; }
            }
            if (nextT == -2) return t;
            t = nextT;
        }
        for (int i = 0; i < (int)tri.size(); ++i) {
            if (!alive[i]) continue;
            const T& tr = tri[i];
            if (orient2(P[tr.v[0]], P[tr.v[1]], p) >= 0 && orient2(P[tr.v[1]], P[tr.v[2]], p) >= 0 &&
                orient2(P[tr.v[2]], P[tr.v[0]], p) >= 0)
                return i;
        }
        return -1;
    }

    std::vector<int> free_, mark_, cavity_, newT_;
    std::vector<BEdge> bnd_;
    int stamp_ = 0, last_ = 0;
};

// Even-odd point-in-region test against a set of segments, bucketed by y.
class SegmentIndex {
public:
    SegmentIndex(const std::vector<std::pair<Vec2, Vec2>>& segs, double binH) : segs_(segs)
    {
        if (segs_.empty()) return;
        y0_ = segs_[0].first.y;
        double y1 = y0_;
        for (const auto& s : segs_) {
            y0_ = std::min({y0_, s.first.y, s.second.y});
            y1 = std::max({y1, s.first.y, s.second.y});
        }
        binH_ = std::max(binH, 1e-6);
        nb_ = std::max(1, (int)std::ceil((y1 - y0_) / binH_) + 1);
        bins_.resize(nb_);
        for (int i = 0; i < (int)segs_.size(); ++i) {
            const double a = std::min(segs_[i].first.y, segs_[i].second.y);
            const double b = std::max(segs_[i].first.y, segs_[i].second.y);
            const int b0 = bin(a), b1 = bin(b);
            for (int k = b0; k <= b1; ++k) bins_[k].push_back(i);
        }
    }

    bool inside(const Vec2& p) const
    {
        if (segs_.empty()) return false;
        if (p.y < y0_ || bin(p.y) >= nb_) return false;
        bool in = false;
        for (int i : bins_[bin(p.y)]) {
            const Vec2& a = segs_[i].first;
            const Vec2& b = segs_[i].second;
            if ((a.y > p.y) != (b.y > p.y)) {
                const double x = a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y);
                if (x > p.x) in = !in;
            }
        }
        return in;
    }

private:
    int bin(double y) const { return std::max(0, std::min(nb_ - 1, (int)std::floor((y - y0_) / binH_))); }
    std::vector<std::pair<Vec2, Vec2>> segs_;
    std::vector<std::vector<int>> bins_;
    double y0_ = 0.0, binH_ = 1.0;
    int nb_ = 0;
};

}  // namespace

// ---------------------------------------------------------------------------------------

void Mesh::finalize()
{
    const int nv = (int)verts.size();
    std::vector<int> parent(nv);
    for (int i = 0; i < nv; ++i) parent[i] = i;
    auto find = [&](int x) {
        while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; }
        return x;
    };
    for (const Tri& t : tris) {
        const int a = find(t[0]), b = find(t[1]), c = find(t[2]);
        parent[b] = a;
        parent[find(c)] = a;
    }
    std::vector<int> id(nv, -1);
    numComps = 0;
    vertComp.assign(nv, -1);
    for (int i = 0; i < nv; ++i) {
        const int r = find(i);
        if (id[r] < 0) id[r] = numComps++;
        vertComp[i] = id[r];
    }
    triComp.resize(tris.size());
    for (size_t t = 0; t < tris.size(); ++t) triComp[t] = vertComp[tris[t][0]];
    if (boundary.size() != verts.size()) boundary.assign(verts.size(), 0);
    locator.build(verts, tris);
}

void appendMesh(Mesh& dst, const Mesh& part, int source)
{
    const int base = (int)dst.verts.size();
    if (dst.triSource.size() != dst.tris.size()) dst.triSource.assign(dst.tris.size(), 0);
    const double n0 = (double)dst.tris.size(), n1 = (double)part.tris.size();
    dst.edgeLength = n0 + n1 > 0.0 ? (dst.edgeLength * n0 + part.edgeLength * n1) / (n0 + n1) : part.edgeLength;
    dst.verts.insert(dst.verts.end(), part.verts.begin(), part.verts.end());
    if (dst.boundary.size() != (size_t)base) dst.boundary.resize(base, 0);
    if (part.boundary.size() == part.verts.size())
        dst.boundary.insert(dst.boundary.end(), part.boundary.begin(), part.boundary.end());
    else
        dst.boundary.resize(dst.verts.size(), 0);
    for (const Tri& t : part.tris) {
        dst.tris.push_back({t[0] + base, t[1] + base, t[2] + base});
        dst.triSource.push_back(source);
    }
}

int locateTop(const Mesh& m, const Vec2& p, double bary[3], double maxDist)
{
    std::vector<std::pair<int, std::array<double, 3>>> hits;
    m.locator.locateAll(p, hits);
    int best = -1;
    for (const auto& h : hits)
        if (h.first > best) {
            best = h.first;
            for (int k = 0; k < 3; ++k) bary[k] = h.second[k];
        }
    if (best < 0 && maxDist > 0.0) best = m.locator.nearest(p, maxDist, bary);
    return best;
}

bool buildMesh(const MaskGrid& m, const MeshParams& prm, Mesh& out, MeshStats* st)
{
    const auto tStart = std::chrono::steady_clock::now();
    out = Mesh();
    if (st) *st = MeshStats();
    if (m.w < 1 || m.h < 1 || m.v.size() < (size_t)m.w * m.h) return false;

    const double cell = m.cell > 0.0 ? m.cell : 1.0;
    const double L = std::max(0.0, prm.expansion) / cell;  // iso level in grid units
    const int pad = (int)std::ceil(L) + 3;
    const int W = m.w + 2 * pad, H = m.h + 2 * pad;

    std::vector<uint8_t> in((size_t)W * H, 0);
    size_t count = 0;
    for (int y = 0; y < m.h; ++y)
        for (int x = 0; x < m.w; ++x)
            if (m.at(x, y) > prm.threshold) {
                in[(size_t)(y + pad) * W + x + pad] = 1;
                ++count;
            }
    if (count == 0) return false;
    removeSpecks(in, W, H, 3);

    std::vector<double> dIn, dOut;
    edt2d(in, W, H, dIn);
    for (auto& b : in) b = !b;
    edt2d(in, W, H, dOut);
    for (auto& b : in) b = !b;

    std::vector<double> sd(in.size());
    size_t regionCount = 0;
    for (size_t i = 0; i < in.size(); ++i) {
        sd[i] = in[i] ? -std::sqrt(dOut[i]) : std::sqrt(dIn[i]);
        if (sd[i] < L) ++regionCount;
    }
    if (regionCount == 0) return false;

    const int N = std::max(8, prm.targetTriangles);
    double hw = std::sqrt((double)regionCount / (0.4330127 * N));
    hw = std::max(hw, 2.0);

    // Outline loops in grid coordinates.
    std::vector<std::vector<Vec2>> loops = traceContours(sd, W, H, L);
    std::vector<std::vector<Vec2>> polys;
    for (auto& lp : loops) {
        std::vector<Vec2> s = simplifyClosed(lp, 0.3);
        if (s.size() < 3 || std::fabs(signedArea(s)) < 1.0) continue;
        polys.push_back(subdivideClosed(s, hw));
    }
    if (polys.empty()) return false;

    for (auto& p : polys) p = mergeShort(p, 0.4 * hw);

    constexpr double kScale = 256.0;
    auto toI = [&](const Vec2& p) { return IP{(int64_t)std::llround(p.x * kScale), (int64_t)std::llround(p.y * kScale)}; };

    double minx = 1e300, miny = 1e300, maxx = -1e300, maxy = -1e300;
    for (const auto& p : polys)
        for (const Vec2& v : p) {
            minx = std::min(minx, v.x); maxx = std::max(maxx, v.x);
            miny = std::min(miny, v.y); maxy = std::max(maxy, v.y);
        }

    auto sdAt = [&](double x, double y) {
        int ix = (int)std::floor(x), iy = (int)std::floor(y);
        ix = std::max(0, std::min(W - 2, ix));
        iy = std::max(0, std::min(H - 2, iy));
        const double tx = std::min(1.0, std::max(0.0, x - ix)), ty = std::min(1.0, std::max(0.0, y - iy));
        const double a = sd[(size_t)iy * W + ix], b = sd[(size_t)iy * W + ix + 1];
        const double c = sd[(size_t)(iy + 1) * W + ix], d = sd[(size_t)(iy + 1) * W + ix + 1];
        return (a * (1 - tx) + b * tx) * (1 - ty) + (c * (1 - tx) + d * tx) * ty;
    };

    // Interior lattice (anchored at the grid origin so it is stable when the mask changes slightly).
    std::vector<Vec2> interior;
    const double dy = hw * 0.86602540378;
    const int r0 = (int)std::floor(miny / dy), r1 = (int)std::ceil(maxy / dy);
    for (int r = r0; r <= r1; ++r) {
        const double y = r * dy;
        const double off = (r & 1) ? 0.5 * hw : 0.0;
        const int c0 = (int)std::floor((minx - off) / hw), c1 = (int)std::ceil((maxx - off) / hw);
        for (int k = 0; k <= c1 - c0; ++k) {
            const int c = (r & 1) ? c1 - k : c0 + k;
            const double x = c * hw + off;
            if (sdAt(x, y) <= L - 0.5 * hw) interior.push_back({x, y});
        }
    }

    struct Result {
        Delaunay dt;
        std::vector<std::pair<int, int>> segs;
        std::vector<uint8_t> onContour;
        std::vector<int> inside;  // triangle indices inside the outline
        int contourPts = 0, splits = 0, missing = 0;
    };

    auto triangulate = [&](const std::vector<Vec2>& inner, Result& R) {
        Delaunay& dt = R.dt;
        dt.init((int64_t)std::floor(minx * kScale), (int64_t)std::floor(miny * kScale),
                (int64_t)std::ceil(maxx * kScale), (int64_t)std::ceil(maxy * kScale));
        auto markContour = [&](int v) {
            if (v < 0) return;
            if ((int)R.onContour.size() <= v) R.onContour.resize(v + 1, 0);
            R.onContour[v] = 1;
        };
        for (const auto& p : polys) {
            std::vector<int> ids;
            ids.reserve(p.size());
            for (const Vec2& v : p) {
                const int id = dt.insert(toI(v));
                markContour(id);
                ids.push_back(id);
                ++R.contourPts;
            }
            for (size_t i = 0; i < ids.size(); ++i) {
                const int a = ids[i], b = ids[(i + 1) % ids.size()];
                if (a >= 0 && b >= 0 && a != b) R.segs.push_back({a, b});
            }
        }
        for (const Vec2& v : inner) dt.insert(toI(v));

        // Recover outline edges by splitting the missing ones (conforming Delaunay).
        std::unordered_set<uint64_t> edges;
        for (int round = 0; round < 48; ++round) {
            dt.edgeSet(edges);
            std::vector<std::pair<int, int>> next;
            next.reserve(R.segs.size());
            bool any = false;
            for (const auto& sg : R.segs) {
                const int a = sg.first, b = sg.second;
                if (edges.count(edgeKey(a, b))) { next.push_back(sg); continue; }
                const IP A = dt.P[a], B = dt.P[b];
                if (A.x - B.x <= 1 && B.x - A.x <= 1 && A.y - B.y <= 1 && B.y - A.y <= 1) { next.push_back(sg); continue; }
                const IP Mid{(A.x + B.x) / 2, (A.y + B.y) / 2};
                const int mi = dt.insert(Mid);
                ++R.splits;
                markContour(mi);
                if (mi < 0 || mi == a || mi == b) { next.push_back(sg); continue; }
                next.push_back({a, mi});
                next.push_back({mi, b});
                any = true;
            }
            R.segs.swap(next);
            if (!any) break;
        }
        dt.edgeSet(edges);
        R.missing = 0;
        for (const auto& sg : R.segs)
            if (!edges.count(edgeKey(sg.first, sg.second))) ++R.missing;
        if ((int)R.onContour.size() < (int)dt.P.size()) R.onContour.resize(dt.P.size(), 0);

        // Keep the triangles inside the outline.
        std::vector<std::pair<Vec2, Vec2>> segGeom;
        segGeom.reserve(R.segs.size());
        for (const auto& sg : R.segs) {
            const IP A = dt.P[sg.first], B = dt.P[sg.second];
            segGeom.push_back({Vec2(A.x / kScale, A.y / kScale), Vec2(B.x / kScale, B.y / kScale)});
        }
        SegmentIndex region(segGeom, hw);
        for (size_t t = 0; t < dt.tri.size(); ++t) {
            if (!dt.alive[t]) continue;
            const auto& tr = dt.tri[t];
            if (Delaunay::isSuper(tr.v[0]) || Delaunay::isSuper(tr.v[1]) || Delaunay::isSuper(tr.v[2])) continue;
            const IP &a = dt.P[tr.v[0]], &b = dt.P[tr.v[1]], &c = dt.P[tr.v[2]];
            if (orient2(a, b, c) <= 0) continue;
            const Vec2 cen((a.x + b.x + c.x) / (3.0 * kScale), (a.y + b.y + c.y) / (3.0 * kScale));
            if (region.inside(cen)) R.inside.push_back((int)t);
        }
    };

    auto R = std::make_unique<Result>();
    triangulate(interior, *R);

    // Relax the interior vertices towards the centroid of their neighbours and triangulate
    // again: this evens out the triangles where the lattice meets the outline.
    if (!interior.empty()) {
        const Delaunay& dt = R->dt;
        const int nv = (int)dt.P.size();
        std::vector<std::vector<int>> adj(nv);
        std::vector<uint8_t> used(nv, 0);
        for (int t : R->inside) {
            const auto& tr = dt.tri[t];
            for (int k = 0; k < 3; ++k) {
                adj[tr.v[k]].push_back(tr.v[(k + 1) % 3]);
                adj[tr.v[k]].push_back(tr.v[(k + 2) % 3]);
                used[tr.v[k]] = 1;
            }
        }
        std::vector<Vec2> pos(nv), next(nv);
        for (int v = 0; v < nv; ++v) pos[v] = Vec2(dt.P[v].x / kScale, dt.P[v].y / kScale);
        for (int it = 0; it < 4; ++it) {
            next = pos;
            for (int v = 3; v < nv; ++v) {
                if (!used[v] || R->onContour[v] || adj[v].empty()) continue;
                Vec2 c;
                for (int u : adj[v]) c += pos[u];
                c = c / (double)adj[v].size();
                if (sdAt(c.x, c.y) <= L - 0.3 * hw) next[v] = c;
            }
            pos.swap(next);
        }
        std::vector<Vec2> relaxed;
        relaxed.reserve(interior.size());
        for (int v = 3; v < nv; ++v)
            if (used[v] && !R->onContour[v]) relaxed.push_back(pos[v]);
        auto R2 = std::make_unique<Result>();
        triangulate(relaxed, *R2);
        if (R2->missing <= R->missing && !R2->inside.empty()) R = std::move(R2);
    }

    const Delaunay& dt = R->dt;
    std::vector<int> remap(dt.P.size(), -1);
    auto toCanon = [&](const IP& p) {
        const double gx = p.x / kScale - pad, gy = p.y / kScale - pad;
        return Vec2(m.x0 + (gx + 0.5) * cell, m.y0 + (gy + 0.5) * cell);
    };
    for (int t : R->inside) {
        const auto& tr = dt.tri[t];
        Tri out3;
        for (int k = 0; k < 3; ++k) {
            const int v = tr.v[k];
            if (remap[v] < 0) {
                remap[v] = (int)out.verts.size();
                out.verts.push_back(toCanon(dt.P[v]));
                out.boundary.push_back(R->onContour[v]);
            }
            out3[k] = remap[v];
        }
        out.tris.push_back(out3);
    }
    const int interiorPts = (int)interior.size();
    const int contourPts = R->contourPts, splits = R->splits, missing = R->missing;
    if (out.tris.empty()) return false;

    out.edgeLength = hw * cell;
    out.finalize();

    if (st) {
        st->contourPoints = contourPts;
        st->interiorPoints = interiorPts;
        st->splitPoints = splits;
        st->missingConstraints = missing;
        st->loops = (int)polys.size();
        st->ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - tStart).count();
    }
    return true;
}

uint64_t hashMask(const MaskGrid& m)
{
    uint64_t h = 1469598103934665603ull;
    auto mix = [&](const void* data, size_t n) {
        const uint8_t* p = (const uint8_t*)data;
        for (size_t i = 0; i < n; ++i) {
            h ^= p[i];
            h *= 1099511628211ull;
        }
    };
    mix(&m.w, sizeof m.w);
    mix(&m.h, sizeof m.h);
    mix(&m.x0, sizeof m.x0);
    mix(&m.y0, sizeof m.y0);
    mix(&m.cell, sizeof m.cell);
    std::vector<uint8_t> q(m.v.size());
    for (size_t i = 0; i < m.v.size(); ++i) q[i] = (uint8_t)std::lround(std::min(1.0f, std::max(0.0f, m.v[i])) * 255.0f);
    mix(q.data(), q.size());
    return h;
}

}  // namespace pp
