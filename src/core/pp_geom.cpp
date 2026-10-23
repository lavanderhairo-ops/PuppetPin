#include "pp_geom.h"

#include <limits>

namespace pp {

void TriLocator::build(const std::vector<Vec2>& pos, const std::vector<Tri>& tris)
{
    pos_ = pos;
    tris_ = tris;
    start_.clear();
    items_.clear();
    nx_ = ny_ = 0;
    if (tris_.empty() || pos_.empty()) return;

    double minx = std::numeric_limits<double>::max(), miny = minx;
    double maxx = -minx, maxy = -minx;
    double edgeSum = 0.0;
    for (const Tri& t : tris_) {
        for (int k = 0; k < 3; ++k) {
            const Vec2& p = pos_[t[k]];
            minx = std::min(minx, p.x); maxx = std::max(maxx, p.x);
            miny = std::min(miny, p.y); maxy = std::max(maxy, p.y);
        }
        edgeSum += length(pos_[t[1]] - pos_[t[0]]);
    }
    // Cells roughly two average edges wide, capped so huge (folded/exploded) meshes stay bounded.
    double cell = std::max(1e-6, 2.0 * edgeSum / tris_.size());
    const double w = maxx - minx, h = maxy - miny;
    while ((w / cell + 1.0) * (h / cell + 1.0) > 4.0 * tris_.size() + 4096.0) cell *= 1.5;

    gx0_ = minx;
    gy0_ = miny;
    inv_ = 1.0 / cell;
    nx_ = std::max(1, (int)std::floor(w * inv_) + 1);
    ny_ = std::max(1, (int)std::floor(h * inv_) + 1);

    std::vector<int> counts((size_t)nx_ * ny_ + 1, 0);
    auto cellRange = [&](const Tri& t, int& cx0, int& cy0, int& cx1, int& cy1) {
        double x0 = std::min({pos_[t[0]].x, pos_[t[1]].x, pos_[t[2]].x});
        double x1 = std::max({pos_[t[0]].x, pos_[t[1]].x, pos_[t[2]].x});
        double y0 = std::min({pos_[t[0]].y, pos_[t[1]].y, pos_[t[2]].y});
        double y1 = std::max({pos_[t[0]].y, pos_[t[1]].y, pos_[t[2]].y});
        cx0 = std::max(0, std::min(nx_ - 1, (int)std::floor((x0 - gx0_) * inv_)));
        cx1 = std::max(0, std::min(nx_ - 1, (int)std::floor((x1 - gx0_) * inv_)));
        cy0 = std::max(0, std::min(ny_ - 1, (int)std::floor((y0 - gy0_) * inv_)));
        cy1 = std::max(0, std::min(ny_ - 1, (int)std::floor((y1 - gy0_) * inv_)));
    };
    for (const Tri& t : tris_) {
        int cx0, cy0, cx1, cy1;
        cellRange(t, cx0, cy0, cx1, cy1);
        for (int cy = cy0; cy <= cy1; ++cy)
            for (int cx = cx0; cx <= cx1; ++cx) counts[(size_t)cy * nx_ + cx + 1]++;
    }
    for (size_t i = 1; i < counts.size(); ++i) counts[i] += counts[i - 1];
    start_ = counts;
    items_.assign(counts.back(), 0);
    std::vector<int> fill(counts.begin(), counts.end() - 1);
    for (int ti = 0; ti < (int)tris_.size(); ++ti) {
        int cx0, cy0, cx1, cy1;
        cellRange(tris_[ti], cx0, cy0, cx1, cy1);
        for (int cy = cy0; cy <= cy1; ++cy)
            for (int cx = cx0; cx <= cx1; ++cx) items_[fill[(size_t)cy * nx_ + cx]++] = ti;
    }
}

template <class F>
void TriLocator::forCells(double x0, double y0, double x1, double y1, F&& f) const
{
    if (nx_ == 0) return;
    int cx0 = (int)std::floor((x0 - gx0_) * inv_), cx1 = (int)std::floor((x1 - gx0_) * inv_);
    int cy0 = (int)std::floor((y0 - gy0_) * inv_), cy1 = (int)std::floor((y1 - gy0_) * inv_);
    cx0 = std::max(cx0, 0); cy0 = std::max(cy0, 0);
    cx1 = std::min(cx1, nx_ - 1); cy1 = std::min(cy1, ny_ - 1);
    for (int cy = cy0; cy <= cy1; ++cy)
        for (int cx = cx0; cx <= cx1; ++cx) {
            const size_t c = (size_t)cy * nx_ + cx;
            for (int i = start_[c]; i < start_[c + 1]; ++i) f(items_[i]);
        }
}

int TriLocator::locate(const Vec2& p, double bary[3], double eps) const
{
    int found = -1;
    forCells(p.x, p.y, p.x, p.y, [&](int ti) {
        if (found >= 0) return;
        const Tri& t = tris_[ti];
        double b[3];
        if (!barycentric(pos_[t[0]], pos_[t[1]], pos_[t[2]], p, b)) return;
        if (b[0] >= -eps && b[1] >= -eps && b[2] >= -eps) {
            found = ti;
            bary[0] = b[0]; bary[1] = b[1]; bary[2] = b[2];
        }
    });
    return found;
}

void TriLocator::locateAll(const Vec2& p, std::vector<std::pair<int, std::array<double, 3>>>& out, double eps) const
{
    out.clear();
    forCells(p.x, p.y, p.x, p.y, [&](int ti) {
        const Tri& t = tris_[ti];
        double b[3];
        if (!barycentric(pos_[t[0]], pos_[t[1]], pos_[t[2]], p, b)) return;
        if (b[0] >= -eps && b[1] >= -eps && b[2] >= -eps) out.push_back({ti, {b[0], b[1], b[2]}});
    });
}

int TriLocator::nearest(const Vec2& p, double maxDist, double bary[3], double* dist) const
{
    int best = -1;
    double bestD2 = maxDist * maxDist;
    Vec2 bestPt;
    forCells(p.x - maxDist, p.y - maxDist, p.x + maxDist, p.y + maxDist, [&](int ti) {
        const Tri& t = tris_[ti];
        const Vec2 &a = pos_[t[0]], &b = pos_[t[1]], &c = pos_[t[2]];
        double bc[3];
        if (!barycentric(a, b, c, p, bc)) return;
        if (bc[0] >= 0.0 && bc[1] >= 0.0 && bc[2] >= 0.0) {
            if (bestD2 > 0.0 || best < 0) { best = ti; bestD2 = 0.0; bestPt = p; }
            return;
        }
        Vec2 q;
        double d2 = segmentDist2(p, a, b, &q);
        Vec2 q2;
        double e2 = segmentDist2(p, b, c, &q2);
        if (e2 < d2) { d2 = e2; q = q2; }
        e2 = segmentDist2(p, c, a, &q2);
        if (e2 < d2) { d2 = e2; q = q2; }
        if (d2 <= bestD2 && (best < 0 || d2 < bestD2)) { best = ti; bestD2 = d2; bestPt = q; }
    });
    if (best >= 0) {
        const Tri& t = tris_[best];
        barycentric(pos_[t[0]], pos_[t[1]], pos_[t[2]], bestPt, bary);
        for (int k = 0; k < 3; ++k) bary[k] = std::min(1.0, std::max(0.0, bary[k]));
        const double s = bary[0] + bary[1] + bary[2];
        if (s > 0.0) for (int k = 0; k < 3; ++k) bary[k] /= s;
        if (dist) *dist = std::sqrt(bestD2);
    }
    return best;
}

}  // namespace pp
