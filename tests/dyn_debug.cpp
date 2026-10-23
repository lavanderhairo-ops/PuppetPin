// Debug driver for the dynamics: a vertical bar held at the bottom, its middle pin jerked sideways
// (the Resolve test scene). Prints the free tip's path and the worst triangle distortion per frame
// for several lag slope limits.
#include "../src/core/pp_render.h"
#include "../src/core/pp_solver.h"
#include "test_util.h"

#include <cstdio>
#include <cstdlib>
#include <memory>

using namespace pp;

namespace {
// Smallest deformed/rest signed-area ratio over all triangles (< 0: folded over).
Vec2 gWorstAt;
double minAreaRatio(const Mesh& M, const std::vector<Vec2>& pos)
{
    double worst = 1e300;
    for (const Tri& t : M.tris) {
        const double a0 = cross(M.verts[t[1]] - M.verts[t[0]], M.verts[t[2]] - M.verts[t[0]]);
        const double a1 = cross(pos[t[1]] - pos[t[0]], pos[t[2]] - pos[t[0]]);
        if (std::fabs(a0) > 1e-9 && a1 / a0 < worst) {
            worst = a1 / a0;
            gWorstAt = (M.verts[t[0]] + M.verts[t[1]] + M.verts[t[2]]) / 3.0;
        }
    }
    return worst;
}
}  // namespace

int main(int argc, char** argv)
{
    // 1920x1080 frame, bar 0.07 x 0.55 of it centred at (0.5, 0.45) (y up).
    MaskGrid g;
    g.w = 1920;
    g.h = 1080;
    g.v.assign((size_t)g.w * g.h, 0.0f);
    const int x0 = 893, x1 = 1027, y0 = 189, y1 = 783;
    for (int y = y0; y < y1; ++y)
        for (int x = x0; x < x1; ++x) g.v[(size_t)y * g.w + x] = 1.0f;
    auto bar = std::make_shared<Mesh>();
    MeshParams mp;
    mp.targetTriangles = argc > 1 ? std::atoi(argv[1]) : 600;
    buildMesh(g, mp, *bar, nullptr);
    PuppetSolver solver(bar);
    int tip = 0;
    for (size_t v = 0; v < bar->verts.size(); ++v)
        if (bar->verts[v].y > bar->verts[tip].y) tip = (int)v;
    std::printf("verts %zu tris %zu, tip (%.1f %.1f)\n", bar->verts.size(), bar->tris.size(), bar->verts[tip].x,
                bar->verts[tip].y);
    PinInput a, b;
    a.src = a.dst = {960, 216};
    b.src = b.dst = {960, 486};
    SolverParams sp;
    {
        // Static: top anchored by a weight map, middle point pin pulled 230 px sideways (the Resolve spike).
        std::vector<float> anchor(bar->verts.size(), 0.0f);
        for (size_t v = 0; v < bar->verts.size(); ++v)
            if (bar->verts[v].y >= 637.0) anchor[v] = 1.0f;
        std::vector<PinInput> pins = {a, b};
        pins[1].dst = {960 + 230.4, 486};
        for (double stiffen : {1.0, 4.0, 8.0, 16.0, 32.0}) {
            SolverParams s2;
            s2.anchor = &anchor;
            s2.pinStiffen = stiffen;
            Deformation d;
            solver.solve(pins, s2, d);
            double smax = 0.0, smin = 1e300;
            for (const Tri& t : bar->tris) {
                const Vec2 r1 = bar->verts[t[1]] - bar->verts[t[0]], r2 = bar->verts[t[2]] - bar->verts[t[0]];
                const Vec2 d1 = d.pos[t[1]] - d.pos[t[0]], d2 = d.pos[t[2]] - d.pos[t[0]];
                const double det = cross(r1, r2);
                if (std::fabs(det) < 1e-9) continue;
                // F = [d1 d2] * inverse([r1 r2])
                const double i00 = r2.y / det, i01 = -r2.x / det, i10 = -r1.y / det, i11 = r1.x / det;
                const double f00 = d1.x * i00 + d2.x * i10, f01 = d1.x * i01 + d2.x * i11;
                const double f10 = d1.y * i00 + d2.y * i10, f11 = d1.y * i01 + d2.y * i11;
                const double a2 = f00 * f00 + f10 * f10, b2 = f00 * f01 + f10 * f11, c2 = f01 * f01 + f11 * f11;
                const double tr = a2 + c2, disc = std::sqrt(std::max(0.0, (a2 - c2) * (a2 - c2) / 4.0 + b2 * b2));
                smax = std::max(smax, std::sqrt(tr / 2.0 + disc));
                smin = std::min(smin, std::sqrt(std::max(0.0, tr / 2.0 - disc)));
            }
            std::printf("anchored pull, pin stiffen %4.1f: max stretch %.2f, min %.2f, pin at (%.1f %.1f)\n", stiffen, smax,
                        smin, pins[1].dst.x, pins[1].dst.y);
            if (argc > 2) {
                // Grid-textured bar, cropped around the bar, written for a visual check.
                ppt::Image src(g.w, g.h), out(g.w, g.h);
                for (int y = y0; y < y1; ++y)
                    for (int x = x0; x < x1; ++x) {
                        const bool line = (x - x0) % 16 < 2 || (y - y0) % 16 < 2;
                        float* p = src.at(x, y);
                        p[0] = line ? 0.1f : 0.95f;
                        p[1] = line ? 0.1f : 0.6f + 0.3f * (float)(y - y0) / (y1 - y0);
                        p[2] = line ? 0.1f : 0.2f;
                        p[3] = 1.0f;
                    }
                ImageView sv = src.view(), ov = out.view();
                RenderJob job;
                job.src = &sv;
                job.dst = &ov;
                job.window = {0, 0, g.w, g.h};
                job.mesh = bar.get();
                job.deformed = &d.pos;
                job.aa = 2;
                renderPuppet(job);
                ppt::Image crop(560, 640);
                for (int y = 0; y < 640; ++y)
                    for (int x = 0; x < 560; ++x)
                        for (int c = 0; c < 4; ++c) crop.at(x, y)[c] = out.at(820 + x, 180 + y)[c];
                ppt::savePNG(std::string(argv[2]) + "/pull_stiffen" + std::to_string((int)stiffen) + ".png", crop);
            }
        }
    }
    for (double slope : {0.5}) {
        for (int elastic = 1; elastic >= 0; --elastic) {
            DynamicsParams dp;
            dp.elastic = elastic;
            dp.maxSlope = slope;
            DynamicState st;
            Deformation d;
            double worst = 1e300, peak = 0.0;
            int worstF = -1;
            Vec2 worstAt;
            std::printf("slope %.2f %s\n  tip x:", slope, elastic ? "elastic" : "smooth ");
            for (int f = 0; f <= 40; ++f) {
                const double k = std::min(1.0, f / 6.0);
                std::vector<PinInput> pins = {a, b};
                pins[1].dst = {960 + 230.4 * k, 486};
                solver.step(pins, sp, dp, st, d);
                const double r = minAreaRatio(*bar, d.pos);
                if (r < worst) {
                    worst = r;
                    worstF = f;
                    worstAt = gWorstAt;
                }
                peak = std::max(peak, d.pos[tip].x);
                if (f % 3 == 0) std::printf(" %.0f", d.pos[tip].x);
            }
            Deformation s;
            std::vector<PinInput> pins = {a, b};
            pins[1].dst = {960 + 230.4, 486};
            solver.solve(pins, sp, s);
            const double sw = minAreaRatio(*bar, s.pos);
            std::printf("\n  static tip %.0f (worst ratio %.3f), peak %.0f (overshoot %.0f px), worst area ratio %.3f at "
                        "frame %d rest (%.0f %.0f)%s\n",
                        s.pos[tip].x, sw, peak, peak - s.pos[tip].x, worst, worstF, worstAt.x, worstAt.y,
                        worst <= 0.0 ? "  FOLDED" : "");
        }
    }
    return 0;
}
