#include "pp_render.h"

#include <atomic>
#include <cstring>
#include <numeric>
#include <thread>
#include <unordered_set>

namespace pp {

void parallelFor(int n, int threads, const std::function<void(int)>& f)
{
    if (n <= 0) return;
    int nt = threads > 0 ? threads : (int)std::thread::hardware_concurrency();
    nt = std::max(1, std::min(nt, std::min(n, 64)));
    if (nt == 1) {
        for (int i = 0; i < n; ++i) f(i);
        return;
    }
    std::atomic<int> next{0};
    auto worker = [&]() {
        for (;;) {
            const int i = next.fetch_add(1);
            if (i >= n) break;
            f(i);
        }
    };
    std::vector<std::thread> pool;
    pool.reserve(nt - 1);
    for (int t = 1; t < nt; ++t) pool.emplace_back(worker);
    worker();
    for (auto& th : pool) th.join();
}

void buildMaskGrid(const ImageView& src, const Xform& toSrc, double fx, double fy, double fw, double fh,
                   double cell, bool fullFrame, MaskGrid& out)
{
    out.cell = cell;
    out.x0 = fx;
    out.y0 = fy;
    out.w = std::max(1, (int)std::ceil(fw / cell - 1e-9));
    out.h = std::max(1, (int)std::ceil(fh / cell - 1e-9));
    out.v.assign((size_t)out.w * out.h, fullFrame ? 1.0f : 0.0f);
    if (fullFrame || !src.valid()) return;

    const double pxPerCell = cell * std::max(std::fabs(toSrc.sx), std::fabs(toSrc.sy));
    // Max over a few samples per cell, so no pixel above the threshold is ever dropped.
    const int k = std::max(1, std::min(4, (int)std::ceil(pxPerCell - 1e-6)));
    parallelFor(out.h, 0, [&](int j) {
        float* row = &out.v[(size_t)j * out.w];
        for (int i = 0; i < out.w; ++i) {
            float m = 0.0f;
            for (int b = 0; b < k; ++b)
                for (int a = 0; a < k; ++a) {
                    const Vec2 p = toSrc.apply({fx + (i + (a + 0.5) / k) * cell, fy + (j + (b + 0.5) / k) * cell});
                    const int px = (int)std::floor(p.x), py = (int)std::floor(p.y);
                    if (src.contains(px, py)) m = std::max(m, src.pixel(px, py)[3]);
                }
            row[i] = m;
        }
    });
}

namespace {

constexpr int64_t kSub = 256;

struct RTri {
    int64_t X[3], Y[3];  // destination, 1/256 pixel fixed point, counter-clockwise
    // Affine map from destination pixel position to source pixel position.
    double au, bu, cu, av, bv, cv;
    int64_t area2;
    int bx0, by0, bx1, by1;
    bool topLeft[3];
};

bool setupTri(const Vec2 d[3], const Vec2 s[3], RTri& t)
{
    for (int k = 0; k < 3; ++k) {
        t.X[k] = (int64_t)std::llround(std::min(1e6, std::max(-1e6, d[k].x)) * kSub);
        t.Y[k] = (int64_t)std::llround(std::min(1e6, std::max(-1e6, d[k].y)) * kSub);
    }
    int64_t a = (t.X[1] - t.X[0]) * (t.Y[2] - t.Y[0]) - (t.Y[1] - t.Y[0]) * (t.X[2] - t.X[0]);
    if (a == 0) return false;
    if (a < 0) {
        std::swap(t.X[1], t.X[2]);
        std::swap(t.Y[1], t.Y[2]);
        a = -a;
    }
    // Texture mapping in double precision from the unquantised positions, so an
    // undeformed triangle maps every pixel exactly onto itself.
    const double m00 = d[1].x - d[0].x, m01 = d[2].x - d[0].x;
    const double m10 = d[1].y - d[0].y, m11 = d[2].y - d[0].y;
    const double det = m00 * m11 - m01 * m10;
    if (std::fabs(det) < 1e-12) return false;
    const double du1 = s[1].x - s[0].x, du2 = s[2].x - s[0].x;
    const double dv1 = s[1].y - s[0].y, dv2 = s[2].y - s[0].y;
    t.au = (du1 * m11 - du2 * m10) / det;
    t.bu = (du2 * m00 - du1 * m01) / det;
    t.cu = s[0].x - t.au * d[0].x - t.bu * d[0].y;
    t.av = (dv1 * m11 - dv2 * m10) / det;
    t.bv = (dv2 * m00 - dv1 * m01) / det;
    t.cv = s[0].y - t.av * d[0].x - t.bv * d[0].y;
    t.area2 = a;
    for (int k = 0; k < 3; ++k) {
        const int A = (k + 1) % 3, B = (k + 2) % 3;
        const int64_t dx = t.X[B] - t.X[A], dy = t.Y[B] - t.Y[A];
        t.topLeft[k] = dy < 0 || (dy == 0 && dx > 0);
    }
    const int64_t minX = std::min({t.X[0], t.X[1], t.X[2]}), maxX = std::max({t.X[0], t.X[1], t.X[2]});
    const int64_t minY = std::min({t.Y[0], t.Y[1], t.Y[2]}), maxY = std::max({t.Y[0], t.Y[1], t.Y[2]});
    t.bx0 = (int)std::floor((double)minX / kSub) - 1;
    t.bx1 = (int)std::floor((double)maxX / kSub) + 1;
    t.by0 = (int)std::floor((double)minY / kSub) - 1;
    t.by1 = (int)std::floor((double)maxY / kSub) + 1;
    return true;
}

// Visits pixels of [x0,x1) x [y0,y1) whose sample point (pixel corner + offset/256) is
// inside t, with the top-left fill rule so shared edges are covered exactly once.
template <class F>
void rasterize(const RTri& t, int x0, int x1, int y0, int y1, int64_t ox, int64_t oy, F&& f)
{
    const int xa = std::max(x0, t.bx0), xb = std::min(x1 - 1, t.bx1);
    const int ya = std::max(y0, t.by0), yb = std::min(y1 - 1, t.by1);
    if (xa > xb || ya > yb) return;
    int64_t dX[3], dY[3], row[3];
    const int64_t px0 = (int64_t)xa * kSub + ox, py0 = (int64_t)ya * kSub + oy;
    for (int k = 0; k < 3; ++k) {
        const int A = (k + 1) % 3, B = (k + 2) % 3;
        const int64_t ex = t.X[B] - t.X[A], ey = t.Y[B] - t.Y[A];
        dX[k] = -ey * kSub;
        dY[k] = ex * kSub;
        row[k] = ex * (py0 - t.Y[A]) - ey * (px0 - t.X[A]);
    }
    const int64_t bias0 = t.topLeft[0] ? 0 : 1, bias1 = t.topLeft[1] ? 0 : 1, bias2 = t.topLeft[2] ? 0 : 1;
    for (int y = ya; y <= yb; ++y) {
        int64_t e0 = row[0], e1 = row[1], e2 = row[2];
        for (int x = xa; x <= xb; ++x) {
            if (e0 >= bias0 && e1 >= bias1 && e2 >= bias2) f(x, y);
            e0 += dX[0];
            e1 += dX[1];
            e2 += dX[2];
        }
        row[0] += dY[0];
        row[1] += dY[1];
        row[2] += dY[2];
    }
}

inline void sampleBilinear(const ImageView& src, double u, double v, float out[4])
{
    out[0] = out[1] = out[2] = out[3] = 0.0f;
    u = std::min(1e7, std::max(-1e7, u));
    v = std::min(1e7, std::max(-1e7, v));
    const double fx = u - 0.5, fy = v - 0.5;
    const double flx = std::floor(fx), fly = std::floor(fy);
    const int ix = (int)flx, iy = (int)fly;
    const float tx = (float)(fx - flx), ty = (float)(fy - fly);
    auto tap = [&](int x, int y, float w) {
        if (w <= 0.0f || !src.contains(x, y)) return;
        const float* p = src.pixel(x, y);
        out[0] += w * p[0];
        out[1] += w * p[1];
        out[2] += w * p[2];
        out[3] += w * p[3];
    };
    tap(ix, iy, (1.0f - tx) * (1.0f - ty));
    tap(ix + 1, iy, tx * (1.0f - ty));
    tap(ix, iy + 1, (1.0f - tx) * ty);
    tap(ix + 1, iy + 1, tx * ty);
}

RectI clipWindow(const RectI& w, const ImageView& img)
{
    RectI r;
    r.x1 = std::max(w.x1, img.x1);
    r.y1 = std::max(w.y1, img.y1);
    r.x2 = std::min(w.x2, img.x2);
    r.y2 = std::min(w.y2, img.y2);
    return r;
}

}  // namespace

void copyImage(const ImageView& src, const ImageView& dst, const RectI& window)
{
    const RectI w = clipWindow(window, dst);
    for (int y = w.y1; y < w.y2; ++y) {
        float* o = dst.pixel(w.x1, y);
        for (int x = w.x1; x < w.x2; ++x, o += 4) {
            if (src.contains(x, y))
                std::memcpy(o, src.pixel(x, y), 4 * sizeof(float));
            else
                o[0] = o[1] = o[2] = o[3] = 0.0f;
        }
    }
}

void renderPuppet(const RenderJob& job)
{
    const Mesh& M = *job.mesh;
    const std::vector<Vec2>& D = *job.deformed;
    const ImageView& src = *job.src;
    const ImageView& dst = *job.dst;
    const RectI w = clipWindow(job.window, dst);
    if (w.x2 <= w.x1 || w.y2 <= w.y1) return;
    const int aa = std::max(1, std::min(4, job.aa));
    const bool attr = job.attribute && job.attribute->size() == M.verts.size();
    const bool layered = M.triSource.size() == M.tris.size();
    auto sourceOf = [&](size_t t) { return layered ? M.triSource[t] : 0; };

    std::vector<RTri> tris;
    std::vector<int> layer;
    std::vector<const ImageView*> image;
    tris.reserve(M.tris.size());
    for (size_t t = 0; t < M.tris.size(); ++t) {
        const int so = sourceOf(t);
        if (job.onlySource >= 0 && so != job.onlySource) continue;
        const Tri& tr = M.tris[t];
        Vec2 d[3], s[3];
        for (int k = 0; k < 3; ++k) {
            d[k] = job.toDst.apply(D[tr[k]]);
            s[k] = attr ? (*job.attribute)[tr[k]] : job.toSrc.apply(M.verts[tr[k]]);
        }
        RTri rt;
        if (!setupTri(d, s, rt)) continue;
        tris.push_back(rt);
        layer.push_back(job.triLayer ? (*job.triLayer)[t] : 0);
        const ImageView* img = job.sources && so < (int)job.sources->size() ? (*job.sources)[so] : nullptr;
        image.push_back(img && img->valid() ? img : &src);
    }
    std::vector<int> order(tris.size());
    std::iota(order.begin(), order.end(), 0);
    std::stable_sort(order.begin(), order.end(), [&](int a, int b) { return layer[a] < layer[b]; });

    // Pixels of the first layer outside its rest mesh pass through unchanged.
    const bool keep = job.keepUnmeshed && !attr && job.onlySource <= 0;
    std::vector<RTri> rest;
    if (keep) {
        rest.reserve(M.tris.size());
        for (size_t t = 0; t < M.tris.size(); ++t) {
            if (sourceOf(t) != 0) continue;
            const Tri& tr = M.tris[t];
            Vec2 d[3];
            for (int k = 0; k < 3; ++k) d[k] = job.toDst.apply(M.verts[tr[k]]);
            RTri rt;
            if (setupTri(d, d, rt)) rest.push_back(rt);
        }
    }

    const int W = w.x2 - w.x1;
    const int bandH = 16;
    const int nb = (w.y2 - w.y1 + bandH - 1) / bandH;
    parallelFor(nb, job.threads, [&](int bi) {
        const int y0 = w.y1 + bi * bandH, y1 = std::min(w.y2, y0 + bandH);
        const int bh = y1 - y0;
        const size_t npx = (size_t)W * bh;
        std::vector<float> base(npx * 4, 0.0f);
        if (keep) {
            std::vector<uint8_t> cov(npx, 0);
            for (const RTri& t : rest)
                rasterize(t, w.x1, w.x2, y0, y1, kSub / 2, kSub / 2,
                          [&](int x, int y) { cov[(size_t)(y - y0) * W + (x - w.x1)] = 1; });
            for (int y = y0; y < y1; ++y)
                for (int x = w.x1; x < w.x2; ++x) {
                    const size_t i = (size_t)(y - y0) * W + (x - w.x1);
                    if (cov[i]) continue;
                    const Vec2 c = job.toDst.invert({x + 0.5, y + 0.5});
                    const Vec2 sp = job.toSrc.apply(c);
                    sampleBilinear(src, sp.x, sp.y, &base[i * 4]);
                }
        }
        std::vector<float> buf(npx * 4), acc;
        if (aa > 1) acc.assign(npx * 4, 0.0f);
        for (int sy = 0; sy < aa; ++sy)
            for (int sx = 0; sx < aa; ++sx) {
                const int64_t ox = ((2 * sx + 1) * kSub) / (2 * aa), oy = ((2 * sy + 1) * kSub) / (2 * aa);
                buf = base;
                for (int idx : order) {
                    const RTri& t = tris[idx];
                    const ImageView& tex = *image[idx];
                    rasterize(t, w.x1, w.x2, y0, y1, ox, oy, [&](int x, int y) {
                        // Coverage is per subsample, but the image is read at the pixel centre
                        // (through this triangle's map), so antialiasing only smooths edges and
                        // never softens the picture: undeformed areas stay bit-exact.
                        const double px = x + 0.5, py = y + 0.5;
                        const double u = t.au * px + t.bu * py + t.cu;
                        const double v = t.av * px + t.bv * py + t.cv;
                        float* o = &buf[((size_t)(y - y0) * W + (x - w.x1)) * 4];
                        if (attr) {  // the interpolated per-vertex value, front triangle wins
                            o[0] = (float)u;
                            o[1] = (float)v;
                            o[2] = 0.0f;
                            o[3] = 1.0f;
                            return;
                        }
                        float s[4];
                        sampleBilinear(tex, u, v, s);
                        const float ia = 1.0f - s[3];
                        o[0] = s[0] + o[0] * ia;
                        o[1] = s[1] + o[1] * ia;
                        o[2] = s[2] + o[2] * ia;
                        o[3] = s[3] + o[3] * ia;
                    });
                }
                if (aa > 1)
                    for (size_t i = 0; i < npx * 4; ++i) acc[i] += buf[i];
            }
        const float* res = aa > 1 ? acc.data() : buf.data();
        const float norm = 1.0f / (aa * aa);
        for (int y = y0; y < y1; ++y) {
            float* o = dst.pixel(w.x1, y);
            const float* r = res + (size_t)(y - y0) * W * 4;
            if (aa > 1)
                for (int i = 0; i < W * 4; ++i) o[i] = r[i] * norm;
            else
                std::memcpy(o, r, (size_t)W * 4 * sizeof(float));
        }
    });
}

void drawBurnIn(const ImageView& dst, const RectI& window, const Mesh* mesh, const std::vector<Vec2>* positions,
                const std::vector<OverlayPin>& pins, const Xform& toDst, double lineAlpha)
{
    const RectI w = clipWindow(window, dst);
    auto blend = [&](int x, int y, float r, float g, float b, float a) {
        if (x < w.x1 || x >= w.x2 || y < w.y1 || y >= w.y2) return;
        float* o = dst.pixel(x, y);
        const float ia = 1.0f - a;
        o[0] = r * a + o[0] * ia;
        o[1] = g * a + o[1] * ia;
        o[2] = b * a + o[2] * ia;
        o[3] = a + o[3] * ia;
    };
    if (mesh && positions && lineAlpha > 0.0) {
        std::unordered_set<uint64_t> done;
        const float a = (float)lineAlpha;
        for (const Tri& t : mesh->tris)
            for (int k = 0; k < 3; ++k) {
                int i = t[k], j = t[(k + 1) % 3];
                const uint64_t key = ((uint64_t)(uint32_t)std::min(i, j) << 32) | (uint32_t)std::max(i, j);
                if (!done.insert(key).second) continue;
                const Vec2 p = toDst.apply((*positions)[i]), q = toDst.apply((*positions)[j]);
                const double len = std::max(std::fabs(q.x - p.x), std::fabs(q.y - p.y));
                if (len > 20000.0) continue;
                const int steps = std::max(1, (int)std::ceil(len));
                for (int s = 0; s <= steps; ++s) {
                    const Vec2 c = p + (q - p) * ((double)s / steps);
                    blend((int)std::floor(c.x), (int)std::floor(c.y), 1.0f, 0.82f, 0.15f, a);
                }
            }
    }
    for (const OverlayPin& pin : pins) {
        const Vec2 c = toDst.apply(pin.pos);
        const double r = 5.0;
        for (int y = (int)std::floor(c.y - r - 2); y <= (int)std::ceil(c.y + r + 2); ++y)
            for (int x = (int)std::floor(c.x - r - 2); x <= (int)std::ceil(c.x + r + 2); ++x) {
                const double d = std::hypot(x + 0.5 - c.x, y + 0.5 - c.y);
                if (d <= r)
                    blend(x, y, pin.selected ? 1.0f : 1.0f, pin.selected ? 0.25f : 0.85f, pin.selected ? 0.2f : 0.1f, 1.0f);
                else if (d <= r + 1.5)
                    blend(x, y, 0.0f, 0.0f, 0.0f, 0.8f);
            }
    }
}

}  // namespace pp
