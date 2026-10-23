// Core engine test: builds meshes from synthetic images, solves pin deformations, renders,
// checks invariants and writes PNGs to tests/out for visual inspection.
#include "../src/core/pp_rig.h"
#include "../src/core/pp_solver.h"
#include "test_util.h"

#include <chrono>
#include <cstdio>
#include <memory>

using namespace pp;
using namespace ppt;

namespace {

int gFailures = 0;

#define CHECK(cond, ...)                                    \
    do {                                                    \
        if (!(cond)) {                                      \
            std::printf("  FAIL: " __VA_ARGS__);            \
            std::printf("\n");                              \
            ++gFailures;                                    \
        }                                                   \
    } while (0)

double nowMs()
{
    using namespace std::chrono;
    return duration<double, std::milli>(steady_clock::now().time_since_epoch()).count();
}

struct Scene {
    std::string name;
    Image src;
    std::vector<PinInput> pins;
    bool fullFrame = false;
    int triangles = 600;
    double expansion = 3.0;
    int aa = 1;
    bool savePngs = true;
};

void runScene(const Scene& sc, const std::string& outDir)
{
    std::printf("[%s] %dx%d, %d triangles target\n", sc.name.c_str(), sc.src.w, sc.src.h, sc.triangles);
    Image src = sc.src;
    const ImageView sv = src.view();
    const double frameMax = std::max(src.w, src.h);
    const double cell = std::max(1.0, frameMax / 1400.0);

    double t0 = nowMs();
    MaskGrid mask;
    buildMaskGrid(sv, Xform{}, 0, 0, src.w, src.h, cell, sc.fullFrame, mask);
    const double tMask = nowMs() - t0;
    t0 = nowMs();
    const uint64_t hsh = hashMask(mask);
    const double tHash = nowMs() - t0;

    auto mesh = std::make_shared<Mesh>();
    MeshParams mp;
    mp.targetTriangles = sc.triangles;
    mp.expansion = sc.expansion;
    MeshStats st;
    const bool ok = buildMesh(mask, mp, *mesh, &st);
    CHECK(ok, "mesh build failed");
    if (!ok) return;
    std::printf("  mask %.1f ms, hash %.1f ms (%016llx), mesh %.1f ms: %zu verts, %zu tris, %d comps, edge %.1f, "
                "loops %d, contour %d, interior %d, splits %d, missing %d\n",
                tMask, tHash, (unsigned long long)hsh, st.ms, mesh->verts.size(), mesh->tris.size(), mesh->numComps,
                mesh->edgeLength, st.loops, st.contourPoints, st.interiorPoints, st.splitPoints, st.missingConstraints);
    CHECK(st.missingConstraints == 0, "%d outline edges missing", st.missingConstraints);

    // Every covered pixel must be inside the rest mesh.
    int misses = 0, covered = 0;
    for (int y = 0; y < src.h; ++y)
        for (int x = 0; x < src.w; ++x) {
            if (src.at(x, y)[3] <= 0.05f) continue;
            ++covered;
            double b[3];
            if (mesh->locator.locate({x + 0.5, y + 0.5}, b, 1e-7) < 0) ++misses;
        }
    std::printf("  coverage: %d / %d covered pixels outside the mesh\n", misses, covered);
    CHECK(misses == 0, "covered pixels outside mesh: %d", misses);

    // Triangle quality.
    double minAngle = 180.0;
    int below10 = 0, below20 = 0;
    for (const Tri& t : mesh->tris) {
        double tmin = 180.0;
        for (int k = 0; k < 3; ++k) {
            const Vec2 u = mesh->verts[t[(k + 1) % 3]] - mesh->verts[t[k]];
            const Vec2 v = mesh->verts[t[(k + 2) % 3]] - mesh->verts[t[k]];
            tmin = std::min(tmin, std::atan2(std::fabs(cross(u, v)), dot(u, v)) * 57.29577951308232);
        }
        minAngle = std::min(minAngle, tmin);
        below10 += tmin < 10.0;
        below20 += tmin < 20.0;
    }
    std::printf("  min triangle angle %.2f deg, %d tris < 10 deg, %d < 20 deg\n", minAngle, below10, below20);

    PuppetSolver solver(mesh);
    SolverParams sp;
    Deformation def;
    t0 = nowMs();
    solver.solve(sc.pins, sp, def);
    const double tSolve1 = nowMs() - t0;
    t0 = nowMs();
    solver.solve(sc.pins, sp, def);
    const double tSolve2 = nowMs() - t0;
    std::printf("  solve: first %.2f ms (incl. factorisation), cached %.2f ms\n", tSolve1, tSolve2);

    {
        // Pins left at rest must not move anything.
        std::vector<PinInput> still = sc.pins;
        for (auto& p : still) {
            p.dst = p.src;
            p.rotation = 0.0;
            p.scale = 1.0;
        }
        Deformation d0;
        solver.solve(still, sp, d0);
        double dev = 0.0;
        for (size_t v = 0; v < d0.pos.size(); ++v) dev = std::max(dev, length(d0.pos[v] - mesh->verts[v]));
        std::printf("  pins at rest: max vertex drift %.6f\n", dev);
        CHECK(dev < 1e-3, "pins at rest moved vertices by %.6f", dev);
    }
    for (size_t i = 0; i < sc.pins.size(); ++i) {
        if (def.pinTri[i] < 0) { std::printf("  pin %zu is off the mesh\n", i); continue; }
        const Tri& t = mesh->tris[def.pinTri[i]];
        double b[3];
        barycentric(mesh->verts[t[0]], mesh->verts[t[1]], mesh->verts[t[2]], def.pinRest[i], b);
        const Vec2 p = def.pos[t[0]] * b[0] + def.pos[t[1]] * b[1] + def.pos[t[2]] * b[2];
        const double err = length(p - sc.pins[i].dst);
        const double tol = sc.pins[i].strength >= 0.99 ? 0.2 : 1e9;
        CHECK(err < tol, "pin %zu misses its target by %.3f px", i, err);
    }

    Image out(src.w, src.h);
    ImageView ov = out.view();
    RenderJob job;
    job.src = &sv;
    job.dst = &ov;
    job.window = {0, 0, src.w, src.h};
    job.mesh = mesh.get();
    job.deformed = &def.pos;
    job.triLayer = &def.triLayer;
    job.aa = sc.aa;
    t0 = nowMs();
    renderPuppet(job);
    const double tRender = nowMs() - t0;
    std::printf("  render (aa %d): %.1f ms\n", sc.aa, tRender);

    // Identity render must reproduce the source exactly.
    Image ident(src.w, src.h);
    ImageView iv = ident.view();
    job.dst = &iv;
    job.deformed = &mesh->verts;
    job.aa = 1;
    renderPuppet(job);
    double maxDiff = 0.0;
    for (size_t i = 0; i < src.px.size(); ++i) maxDiff = std::max(maxDiff, (double)std::fabs(src.px[i] - ident.px[i]));
    std::printf("  identity max abs diff %.6f\n", maxDiff);
    CHECK(maxDiff < 1e-4, "identity render differs from source by %.6f", maxDiff);

    // With antialiasing the interior must still be exact (only edge coverage may be smoothed).
    {
        Image ident2(src.w, src.h);
        ImageView iv2 = ident2.view();
        job.dst = &iv2;
        job.aa = 2;
        renderPuppet(job);
        double interiorDiff = 0.0;
        for (int y = 1; y < src.h - 1; ++y)
            for (int x = 1; x < src.w - 1; ++x) {
                bool inside = true;
                for (int dy = -1; dy <= 1 && inside; ++dy)
                    for (int dx = -1; dx <= 1; ++dx)
                        if (src.px[((size_t)(y + dy) * src.w + (x + dx)) * 4 + 3] < 0.999f) { inside = false; break; }
                if (!inside) continue;
                for (int c = 0; c < 4; ++c) {
                    const size_t i = ((size_t)y * src.w + x) * 4 + c;
                    interiorDiff = std::max(interiorDiff, (double)std::fabs(src.px[i] - ident2.px[i]));
                }
            }
        std::printf("  identity (aa 2) interior max abs diff %.6f\n", interiorDiff);
        CHECK(interiorDiff < 1e-4, "antialiased identity render softens the interior by %.6f", interiorDiff);
    }

    if (sc.savePngs) {
        std::vector<OverlayPin> restPins, defPins;
        for (size_t i = 0; i < sc.pins.size(); ++i) {
            restPins.push_back({sc.pins[i].src, false});
            defPins.push_back({sc.pins[i].dst, false});
        }
        savePNG(outDir + "/" + sc.name + "_0_input.png", src);
        Image m1 = src;
        drawBurnIn(m1.view(), {0, 0, src.w, src.h}, mesh.get(), &mesh->verts, restPins, Xform{}, 0.55);
        savePNG(outDir + "/" + sc.name + "_1_restmesh.png", m1);
        savePNG(outDir + "/" + sc.name + "_2_output.png", out);
        Image m2 = out;
        drawBurnIn(m2.view(), {0, 0, src.w, src.h}, mesh.get(), &def.pos, defPins, Xform{}, 0.55);
        savePNG(outDir + "/" + sc.name + "_3_outmesh.png", m2);
    }
}

PinInput pin(double sx, double sy, double dx, double dy, double radius = 0.0)
{
    PinInput p;
    p.src = {sx, sy};
    p.dst = {dx, dy};
    p.radius = radius;
    return p;
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc > 2 && std::string(argv[1]) == "--assets") {
        const std::string dir = argv[2];
        saveRGBA(dir + "/puppet_character_1920.png", makeCharacter(1920, 1080));
        saveRGBA(dir + "/puppet_card_1920.png", makeCard(1920, 1080));
        std::printf("assets written to %s\n", dir.c_str());
        return 0;
    }
    const std::string outDir = argc > 1 ? argv[1] : "tests/out";

    {
        std::printf("[rig]\n");
        auto fresh = [] {
            std::vector<RigPin> p(8);
            auto set = [&](int s, double x, double y) {
                p[s].valid = true;
                p[s].rest = p[s].pos = {x, y};
            };
            set(1, 0, 0);       // hip
            set(2, 10, -100);   // knee (slightly forward, so the bend side is defined)
            set(3, 0, -200);    // foot
            set(4, 0, 100);
            set(5, 50, 100);
            return p;
        };
        const double l1 = std::hypot(10, 100), l2 = std::hypot(10, 100);
        // IK: the foot moves, the knee keeps both bone lengths.
        {
            auto p = fresh();
            Deformer d;
            d.type = DeformerType::IK;
            d.pins = {1, 2, 3};
            p[3].pos = {80, -120};
            applyDeformers(p, {d}, 0, 24, nullptr);
            CHECK(std::fabs(length(p[2].pos - p[1].pos) - l1) < 1e-6, "IK upper bone length changed");
            CHECK(std::fabs(length(p[3].pos - p[2].pos) - l2) < 1e-6, "IK lower bone length changed");
            CHECK(length(p[3].pos - Vec2(80, -120)) < 1e-9, "IK moved a reachable end pin");
            CHECK(p[2].driven, "IK joint should be marked driven");
            const double side0 = cross(p[2].rest - p[1].rest, p[3].rest - p[1].rest);
            const double side1 = cross(p[2].pos - p[1].pos, p[3].pos - p[1].pos);
            CHECK(side0 * side1 > 0, "IK flipped the bend");
            p = fresh();
            p[3].pos = {0, -500};
            applyDeformers(p, {d}, 0, 24, nullptr);
            CHECK(std::fabs(length(p[3].pos - p[1].pos) - (l1 + l2)) < 1e-3, "IK out of reach should fully extend");
        }
        // Follower with delay reads the leader at an earlier time.
        {
            auto p = fresh();
            Deformer d;
            d.type = DeformerType::Follower;
            d.pins = {4, 5};
            d.amount = 0.5;
            d.delay = 2;
            double asked = -1;
            applyDeformers(p, {d}, 10, 24, [&](double t, const std::vector<int>& slots, std::vector<RigPin>& q) {
                asked = t;
                for (int s : slots)
                    if (s == 4) {  // leader 20 px right of its rest, two frames ago
                        q[4].valid = true;
                        q[4].rest = Vec2(0, 100);
                        q[4].pos = Vec2(20, 100);
                    }
            });
            CHECK(asked == 8, "follower delay should read frame 8, read %g", asked);
            CHECK(length(p[5].pos - Vec2(60, 100)) < 1e-9, "follower moved to %.2f %.2f", p[5].pos.x, p[5].pos.y);
        }
        // Wave with a static 90 degree bend.
        {
            auto p = fresh();
            p[2].rest = p[2].pos = {0, -100};
            Deformer d;
            d.type = DeformerType::Wave;
            d.pins = {1, 2, 3};
            d.angle = 90;
            d.amount = 0;
            applyDeformers(p, {d}, 0, 24, nullptr);
            CHECK(length(p[2].pos - Vec2(100, 0)) < 1e-9 && length(p[3].pos - Vec2(100, 100)) < 1e-9,
                  "wave bend wrong: %.2f %.2f / %.2f %.2f", p[2].pos.x, p[2].pos.y, p[3].pos.x, p[3].pos.y);
        }
        // FK: rotate at the root, counter-rotate at the knee.
        {
            auto p = fresh();
            p[2].rest = p[2].pos = {0, -100};
            Deformer d;
            d.type = DeformerType::FK;
            d.pins = {1, 2, 3};
            d.angle = 90;
            d.angle2 = -90;
            applyDeformers(p, {d}, 0, 24, nullptr);
            CHECK(length(p[2].pos - Vec2(100, 0)) < 1e-9 && length(p[3].pos - Vec2(100, -100)) < 1e-9,
                  "FK wrong: %.2f %.2f / %.2f %.2f", p[2].pos.x, p[2].pos.y, p[3].pos.x, p[3].pos.y);
        }
        // A whole skeleton in one list: each pin hangs off the closest pin listed before it, so a
        // wave sways the body and every bone keeps its length.
        {
            std::vector<RigPin> p(8);
            const Vec2 at[8] = {{0, 0}, {0, 0}, {0, 100}, {0, 160}, {-40, 90}, {-55, 40}, {-60, -10}, {40, 90}};
            for (int s = 1; s < 8; ++s) {
                p[s].valid = true;
                p[s].rest = p[s].pos = at[s];
            }
            std::vector<Vec2> rest(at + 1, at + 8);
            const std::vector<int> parent = chainParents(rest);
            CHECK((parent == std::vector<int>{-1, 0, 1, 1, 3, 4, 1}), "skeleton parents %d %d %d %d %d %d", parent[1],
                  parent[2], parent[3], parent[4], parent[5], parent[6]);
            for (int type = 0; type < 3; ++type) {
                auto q = p;
                Deformer d;
                d.type = type == 0 ? DeformerType::Wave : (type == 1 ? DeformerType::Wind : DeformerType::FK);
                d.pins = {1, 2, 3, 4, 5, 6, 7};
                d.angle = d.angle2 = d.angle3 = 30;
                d.amount = 0;
                d.strength = 40;
                applyDeformers(q, {d}, 5, 24, nullptr);
                double worst = 0;
                for (int i = 1; i < 7; ++i) {
                    const int c = d.pins[i], par = d.pins[parent[i]];
                    worst = std::max(worst, std::fabs(length(q[c].pos - q[par].pos) - length(at[c] - at[par])));
                }
                CHECK(worst < 1e-6, "skeleton %d changed a bone length by %.3f", type, worst);
                CHECK(length(q[7].pos - q[7].rest) > 1.0, "skeleton %d did not move the other shoulder", type);
            }
            // Wave bends at every joint: the right shoulder turns with hips and chest (60 degrees).
            auto q = p;
            Deformer d;
            d.type = DeformerType::Wave;
            d.pins = {1, 2, 3, 4, 5, 6, 7};
            d.angle = 30;
            d.amount = 0;
            applyDeformers(q, {d}, 0, 24, nullptr);
            const Vec2 want = q[2].pos + rotate(at[7] - at[2], 0.5, std::sqrt(3.0) / 2.0);
            CHECK(length(q[7].pos - want) < 1e-6, "skeleton wave: right shoulder at %.2f %.2f, expected %.2f %.2f",
                  q[7].pos.x, q[7].pos.y, want.x, want.y);
        }
        // Wiggle: pins wander smoothly within about their size, each its own way, repeatably.
        {
            Deformer d;
            d.type = DeformerType::Wiggle;
            d.pins = {4, 5};
            d.size = 20;
            d.frequency = 1;
            double maxOff = 0, maxStep = 0, sameDiff = 0;
            Vec2 prev;
            for (int f = 0; f <= 240; ++f) {
                auto p = fresh();
                applyDeformers(p, {d}, f, 24, nullptr);
                const Vec2 off = p[4].pos - p[4].rest;
                maxOff = std::max(maxOff, length(off));
                if (f > 0) maxStep = std::max(maxStep, length(off - prev));
                prev = off;
                sameDiff = std::max(sameDiff, length((p[5].pos - p[5].rest) - off));
            }
            auto again = fresh(), once = fresh();
            applyDeformers(again, {d}, 37, 24, nullptr);
            applyDeformers(once, {d}, 37, 24, nullptr);
            std::printf("  wiggle: max offset %.1f, max step per frame %.2f\n", maxOff, maxStep);
            CHECK(maxOff > 8 && maxOff < 30, "wiggle offset %.1f not around its size 20", maxOff);
            CHECK(maxStep < 6, "wiggle jumps %.2f px between frames", maxStep);
            CHECK(sameDiff > 5, "wiggled pins should move independently");
            CHECK(length(again[4].pos - once[4].pos) == 0.0, "wiggle is not repeatable");
        }
        // Wind: a hanging chain leans downwind and keeps its bone lengths.
        {
            Deformer d;
            d.type = DeformerType::Wind;
            d.pins = {1, 2, 3};
            d.direction = 0;  // blowing to the right
            d.strength = 25;
            d.gust = 0.6;
            double minX = 1e9, maxX = -1e9;
            bool lengths = true;
            for (int f = 0; f <= 240; f += 3) {
                auto p = fresh();
                p[2].rest = p[2].pos = {0, -100};
                applyDeformers(p, {d}, f, 24, nullptr);
                minX = std::min(minX, p[3].pos.x);
                maxX = std::max(maxX, p[3].pos.x);
                lengths = lengths && std::fabs(length(p[3].pos - p[2].pos) - 100) < 1e-6 &&
                          std::fabs(length(p[2].pos - p[1].pos) - 100) < 1e-6;
            }
            std::printf("  wind: foot x between %.1f and %.1f\n", minX, maxX);
            CHECK(minX > 5, "wind should always push the chain downwind (min x %.1f)", minX);
            CHECK(maxX - minX > 10, "wind should gust (x range %.1f)", maxX - minX);
            CHECK(lengths, "wind changed bone lengths");
        }
        // Spring: the follower trails a leader that jumps 100 px, overshoots and settles.
        {
            auto leaderAt = [](double t, const std::vector<int>& slots, std::vector<RigPin>& q) {
                for (int s : slots)
                    if (s == 4) {
                        q[4].valid = true;
                        q[4].rest = Vec2(0, 100);
                        q[4].pos = Vec2(t >= 10 ? 100.0 : 0.0, 100.0);
                    }
            };
            auto run = [&](double damping, double& peak, double& at12, double& late) {
                Deformer d;
                d.type = DeformerType::Spring;
                d.pins = {4, 5};
                d.amount = 1;
                d.firmness = 2;
                d.damping = damping;
                peak = 0;
                for (int f = 0; f <= 200; ++f) {
                    auto p = fresh();
                    applyDeformers(p, {d}, f, 24, leaderAt);
                    const double x = p[5].pos.x - p[5].rest.x;
                    peak = std::max(peak, x);
                    if (f == 12) at12 = x;
                    if (f == 200) late = x;
                }
            };
            double peak, at12, late, peakD, at12D, lateD;
            run(0.15, peak, at12, late);
            run(1.0, peakD, at12D, lateD);
            std::printf("  spring: frame 12 %.1f, peak %.1f, frame 200 %.1f | damped peak %.1f\n", at12, peak, late,
                        peakD);
            CHECK(at12 > 1 && at12 < 80, "spring should lag the leader (%.1f two frames after the jump)", at12);
            CHECK(peak > 110, "spring should overshoot (peak %.1f)", peak);
            CHECK(std::fabs(late - 100) < 2, "spring should settle at the leader's motion (%.1f)", late);
            CHECK(peakD < 100.5, "a fully damped spring must not overshoot (%.1f)", peakD);
        }
        // A Spring trails a pin that a Wave above it swings (no keyframes anywhere): the follower
        // swings too, a little late.
        {
            std::vector<RigPin> base(8);
            auto set = [&](int s, double x, double y) {
                base[s].valid = true;
                base[s].rest = base[s].pos = {x, y};
            };
            set(1, 0, 0);    // base of the swinging chain
            set(4, 0, 100);  // swung by the wave, leads the spring
            set(5, 0, 160);  // trails pin 4 on the spring
            auto raw = [&](double, const std::vector<int>& slots, std::vector<RigPin>& q) {
                for (int s : slots)
                    if (s > 0 && s < (int)base.size()) q[s] = base[s];
            };
            Deformer wave;
            wave.type = DeformerType::Wave;
            wave.pins = {1, 4};
            wave.amount = 30;  // degrees
            wave.frequency = 1;
            wave.angle = 0;
            Deformer spring;
            spring.type = DeformerType::Spring;
            spring.pins = {4, 5};
            spring.amount = 1;
            spring.firmness = 3;
            spring.damping = 0.3;
            double leadMax = 0, followMax = 0, lagSum = 0;
            for (int f = 0; f <= 96; ++f) {
                auto p = base;
                applyDeformers(p, {wave, spring}, f, 24, raw);
                const double lead = p[4].pos.x, follow = p[5].pos.x;
                leadMax = std::max(leadMax, std::fabs(lead));
                if (f >= 48) followMax = std::max(followMax, std::fabs(follow));
                if (f >= 48) lagSum += std::fabs(follow - lead);
            }
            std::printf("  spring after a wave: leader swings %.1f px, follower %.1f px, mean gap %.1f px\n", leadMax,
                        followMax, lagSum / 49);
            CHECK(leadMax > 40 && followMax > 20, "a spring should trail a pin swung by a wave (%.1f / %.1f)", leadMax,
                  followMax);
            CHECK(lagSum / 49 > 2, "the spring follower should lag the waved leader");
        }
        const auto list = parsePinList("3, 4,5 7;x 40", 32);
        CHECK((list == std::vector<int>{3, 4, 5, 7}), "pin list parsing");
    }

    {
        // Image layers: a red body (layer 0) and a blue arm (layer 1) drawn over it. They are separate
        // meshes; a point pin on the overlap takes the arm (the top layer).
        std::printf("[layers]\n");
        const int W = 400, H = 200;
        Image body(W, H), arm(W, H);
        for (int y = 40; y < 160; ++y)
            for (int x = 60; x < 200; ++x) {
                float* p = body.at(x, y);
                p[0] = 0.9f; p[3] = 1.0f;
            }
        for (int y = 90; y < 120; ++y)
            for (int x = 150; x < 360; ++x) {
                float* p = arm.at(x, y);
                p[2] = 0.9f; p[3] = 1.0f;
            }
        auto meshOf = [](const Image& img, Mesh& m) {
            const ImageView v = const_cast<Image&>(img).view();
            MaskGrid g;
            buildMaskGrid(v, Xform{}, 0, 0, img.w, img.h, 1.0, false, g);
            MeshParams mp;
            mp.targetTriangles = 300;
            return buildMesh(g, mp, m, nullptr);
        };
        Mesh mb, ma;
        CHECK(meshOf(body, mb) && meshOf(arm, ma), "layer meshes");
        auto both = std::make_shared<Mesh>();
        appendMesh(*both, mb, 0);
        appendMesh(*both, ma, 1);
        both->finalize();
        CHECK(both->triSource.size() == both->tris.size() && both->numComps == 2, "merged layers: %d components",
              both->numComps);
        double b[3];
        const int top = locateTop(*both, {170, 105}, b);
        CHECK(top >= 0 && both->triSource[top] == 1, "a point on the overlap should find the top layer");
        // Lift the arm by its tip and at the overlap: the body must not move.
        PuppetSolver solver(both);
        PinInput tip, shoulder;
        tip.src = {350, 105};
        tip.dst = {350, 150};
        shoulder.src = shoulder.dst = {170, 105};
        Deformation d;
        solver.solve({shoulder, tip}, SolverParams(), d);
        double bodyMove = 0.0, armMove = 0.0;
        for (size_t v = 0; v < both->verts.size(); ++v) {
            const double m = length(d.pos[v] - both->verts[v]);
            (v < mb.verts.size() ? bodyMove : armMove) = std::max(v < mb.verts.size() ? bodyMove : armMove, m);
        }
        std::printf("  body moved %.4f px, arm up to %.1f px\n", bodyMove, armMove);
        CHECK(bodyMove == 0.0 && armMove > 30.0, "pins on the arm layer moved the body (%.4f)", bodyMove);
        // Render: each layer from its own image, the arm on top; only-layer and vector output.
        Image out(W, H);
        ImageView bv = body.view(), av = arm.view(), ov = out.view();
        std::vector<const ImageView*> srcs = {&bv, &av};
        RenderJob job;
        job.src = &bv;
        job.sources = &srcs;
        job.dst = &ov;
        job.window = {0, 0, W, H};
        job.mesh = both.get();
        job.deformed = &both->verts;
        renderPuppet(job);
        const float* over = out.at(170, 105);
        const float* bodyOnly = out.at(100, 60);
        CHECK(over[2] > 0.8f && over[0] < 0.1f, "the top layer should cover the overlap (r %.2f b %.2f)", over[0], over[2]);
        CHECK(bodyOnly[0] > 0.8f, "the body layer is missing");
        job.onlySource = 0;
        renderPuppet(job);
        CHECK(out.at(170, 105)[0] > 0.8f && out.at(300, 105)[3] < 0.01f, "single layer output shows other layers");
        job.onlySource = -1;
        std::vector<Vec2> vel(both->verts.size());
        for (size_t v = 0; v < vel.size(); ++v) vel[v] = Vec2(3.0, -2.0) + both->verts[v] * 0.01;
        job.attribute = &vel;
        renderPuppet(job);
        const float* vec = out.at(300, 105);
        std::printf("  vectors at 300,105: %.3f %.3f (expected %.3f %.3f)\n", vec[0], vec[1], 3.0 + 3.005, -2.0 + 1.055);
        CHECK(std::fabs(vec[0] - 6.005f) < 0.02f && std::fabs(vec[1] + 0.945f) < 0.02f && vec[3] == 1.0f,
              "attribute output wrong");
        CHECK(out.at(10, 10)[3] == 0.0f, "attribute output outside the mesh should be empty");
        savePNG(outDir + "/layers.png", out);
    }

    {
        // An arm hanging beside the body (joined at the shoulder) swings across it: the arm must cover
        // the body as a whole, not triangle by triangle; a higher pin layer on the body puts it on top.
        std::printf("[overlap]\n");
        const int W = 300, H = 300;
        Image full(W, H), armOnly(W, H), bodyOnly(W, H);
        auto paint = [&](int x0, int y0, int x1, int y1, bool isArm) {
            for (int y = y0; y < y1; ++y)
                for (int x = x0; x < x1; ++x) {
                    float* p = full.at(x, y);
                    p[isArm ? 2 : 0] = 0.9f;
                    p[3] = 1.0f;
                    float* q = (isArm ? armOnly : bodyOnly).at(x, y);
                    q[isArm ? 2 : 0] = 0.9f;
                    q[3] = 1.0f;
                }
        };
        paint(130, 40, 210, 260, false);  // body
        paint(70, 70, 95, 260, true);     // arm
        paint(95, 235, 130, 260, true);   // shoulder
        auto mesh = std::make_shared<Mesh>();
        {
            const ImageView v = full.view();
            MaskGrid g;
            buildMaskGrid(v, Xform{}, 0, 0, W, H, 1.0, false, g);
            MeshParams mp;
            mp.targetTriangles = 600;
            CHECK(buildMesh(g, mp, *mesh, nullptr), "overlap mesh");
        }
        PuppetSolver solver(mesh);
        PinInput bodyPin, shoulderPin, handPin;
        bodyPin.src = bodyPin.dst = {170, 150};
        shoulderPin.src = shoulderPin.dst = {82, 245};
        handPin.src = {82, 80};
        handPin.dst = {180, 110};
        Image out(W, H), armCov(W, H), bodyCov(W, H);
        auto armShare = [&](const Deformation& d, bool useOrder, int& overlap) {
            ImageView ov = out.view(), av = armCov.view(), bv = bodyCov.view();
            ImageView fv = full.view(), aiv = armOnly.view(), biv = bodyOnly.view();
            RenderJob job;
            job.window = {0, 0, W, H};
            job.mesh = mesh.get();
            job.deformed = &d.pos;
            job.keepUnmeshed = false;
            job.triLayer = useOrder ? &d.triLayer : nullptr;
            job.src = &fv;
            job.dst = &ov;
            renderPuppet(job);
            job.src = &aiv;
            job.dst = &av;
            renderPuppet(job);
            job.src = &biv;
            job.dst = &bv;
            renderPuppet(job);
            int blue = 0;
            overlap = 0;
            for (int y = 0; y < H; ++y)
                for (int x = 0; x < W; ++x) {
                    if (armCov.at(x, y)[3] < 0.99f || bodyCov.at(x, y)[3] < 0.99f) continue;
                    ++overlap;
                    if (out.at(x, y)[2] > 0.5f) ++blue;
                }
            return overlap ? (double)blue / overlap : 0.0;
        };
        Deformation d;
        solver.solve({bodyPin, shoulderPin, handPin}, SolverParams(), d);
        int overlap = 0;
        const double meshOrder = armShare(d, false, overlap);
        const double autoOrder = armShare(d, true, overlap);
        savePNG(outDir + "/overlap.png", out);
        bodyPin.layer = 1;  // (an animated layer: same pins otherwise, so the same binding)
        solver.solve({bodyPin, shoulderPin, handPin}, SolverParams(), d);
        int overlap2 = 0;
        const double bodyOnTop = armShare(d, true, overlap2);
        std::printf("  %d px overlap; arm shown on %.1f%% in mesh order, %.1f%% auto, %.1f%% with the body on layer 1\n",
                    overlap, 100 * meshOrder, 100 * autoOrder, 100 * bodyOnTop);
        CHECK(overlap > 400, "the arm should swing over the body (%d px overlap)", overlap);
        CHECK(autoOrder > 0.995, "the arm should cover the body where it swings over it (%.3f)", autoOrder);
        CHECK(bodyOnTop < 0.005, "a higher layer on the body pin should put the body on top (%.3f)", bodyOnTop);
    }

    {
        std::printf("[influence]\n");
        // A 400 x 60 bar (x 60..460, y 70..130). A pin with an influence only pulls the area within
        // that distance (along the mesh); everything further away keeps the pose without that pin.
        MaskGrid g;
        g.w = 520;
        g.h = 200;
        g.v.assign((size_t)g.w * g.h, 0.0f);
        for (int y = 70; y < 130; ++y)
            for (int x = 60; x < 460; ++x) g.v[(size_t)y * g.w + x] = 1.0f;
        auto bar = std::make_shared<Mesh>();
        MeshParams mp;
        mp.targetTriangles = 600;
        CHECK(buildMesh(g, mp, *bar, nullptr), "bar mesh");
        PuppetSolver solver(bar);
        const std::vector<Vec2>& rest = bar->verts;
        PinInput tipPin;
        tipPin.src = {440, 100};
        tipPin.dst = {440, 140};
        tipPin.influence = 120.0;
        tipPin.falloff = 0.5;
        Deformation d;
        solver.solve({tipPin}, SolverParams(), d);
        double farMove = 0.0, nearMove = 1e300;
        for (size_t v = 0; v < rest.size(); ++v) {
            const double moved = length(d.pos[v] - rest[v]);
            if (rest[v].x < 440 - 125) farMove = std::max(farMove, moved);
            if (length(rest[v] - tipPin.src) < 40) nearMove = std::min(nearMove, moved);
        }
        Vec2 pinNow;
        {
            double b[3];
            const int t = bar->locator.locate(tipPin.src, b);
            CHECK(t >= 0, "pin on the bar");
            const Tri& tr = bar->tris[t];
            pinNow = d.pos[tr[0]] * b[0] + d.pos[tr[1]] * b[1] + d.pos[tr[2]] * b[2];
        }
        std::printf("  one pin, influence 120: far side moved %.4f px, near the pin >= %.1f px, pin error %.2f px\n",
                    farMove, nearMove, length(pinNow - tipPin.dst));
        CHECK(farMove == 0.0, "the image beyond the influence moved (%.4f px)", farMove);
        CHECK(nearMove > 25.0, "the area at the pin should follow it (%.1f px)", nearMove);
        CHECK(length(pinNow - tipPin.dst) < 1.0, "the pin should hold its point (%.2f px off)", length(pinNow - tipPin.dst));

        // With an ordinary pin elsewhere, the far side keeps exactly the pose of that pin alone.
        PinInput left;
        left.src = {80, 100};
        left.dst = {80, 70};
        left.radius = 15.0;
        Deformation both, only;
        solver.solve({left, tipPin}, SolverParams(), both);
        solver.solve({left}, SolverParams(), only);
        double farDiff = 0.0;
        for (size_t v = 0; v < rest.size(); ++v)
            if (rest[v].x < 440 - 125) farDiff = std::max(farDiff, length(both.pos[v] - only.pos[v]));
        std::printf("  plus a normal pin: far side differs from that pin alone by %.4f px\n", farDiff);
        CHECK(farDiff == 0.0, "a pin's influence leaked past its reach (%.4f px)", farDiff);

        // Dynamics keep the unreached area still too.
        DynamicsParams dp;
        DynamicState st;
        double farDyn = 0.0;
        for (int f = 0; f <= 12; ++f) {
            PinInput p = tipPin;
            p.dst = tipPin.src + (tipPin.dst - tipPin.src) * std::min(1.0, f / 4.0);
            solver.step({p}, SolverParams(), dp, st, d);
            for (size_t v = 0; v < rest.size(); ++v)
                if (rest[v].x < 440 - 125) farDyn = std::max(farDyn, length(d.pos[v] - rest[v]));
        }
        std::printf("  elastic: far side moved %.4f px\n", farDyn);
        CHECK(farDyn < 1e-9, "elastic motion leaked past the influence (%.4f px)", farDyn);
    }

    {
        std::printf("[dynamics]\n");
        {
            // A tall bar held at the bottom, its middle pin jerked 230 px sideways in 6 frames: the lag
            // must never fold the mesh over next to the moving pin (it did before the lag slope limit).
            MaskGrid g;
            g.w = 1920;
            g.h = 1080;
            g.v.assign((size_t)g.w * g.h, 0.0f);
            for (int y = 189; y < 783; ++y)
                for (int x = 893; x < 1027; ++x) g.v[(size_t)y * g.w + x] = 1.0f;
            for (int tris : {600, 1500}) {
                auto bar = std::make_shared<Mesh>();
                MeshParams mp;
                mp.targetTriangles = tris;
                CHECK(buildMesh(g, mp, *bar, nullptr), "tall bar mesh");
                PuppetSolver solver(bar);
                PinInput a, b;
                a.src = a.dst = {960, 216};
                b.src = b.dst = {960, 486};
                for (int elastic = 0; elastic < 2; ++elastic) {
                    DynamicsParams dp;
                    dp.elastic = elastic;
                    DynamicState st;
                    Deformation d;
                    double worst = 1e300;
                    for (int f = 0; f <= 30; ++f) {
                        std::vector<PinInput> pins = {a, b};
                        pins[1].dst = {960 + 230.4 * std::min(1.0, f / 6.0), 486};
                        solver.step(pins, SolverParams(), dp, st, d);
                        for (const Tri& t : bar->tris) {
                            const double a0 = cross(bar->verts[t[1]] - bar->verts[t[0]], bar->verts[t[2]] - bar->verts[t[0]]);
                            const double a1 = cross(d.pos[t[1]] - d.pos[t[0]], d.pos[t[2]] - d.pos[t[0]]);
                            if (std::fabs(a0) > 1e-9) worst = std::min(worst, a1 / a0);
                        }
                    }
                    std::printf("  jerked bar (%d tris, %s): smallest triangle area ratio %.3f\n", tris,
                                elastic ? "elastic" : "smooth", worst);
                    CHECK(worst > 0.25, "%s lag folds or crushes the mesh (area ratio %.3f)", elastic ? "elastic" : "smooth", worst);
                }
            }
        }
        // A 400 x 60 bar; its left end is locked by a radius pin that jumps up 60 px at frame 1.
        MaskGrid g;
        g.w = 520;
        g.h = 200;
        g.v.assign((size_t)g.w * g.h, 0.0f);
        for (int y = 70; y < 130; ++y)
            for (int x = 60; x < 460; ++x) g.v[(size_t)y * g.w + x] = 1.0f;
        auto bar = std::make_shared<Mesh>();
        MeshParams mp;
        mp.targetTriangles = 400;
        CHECK(buildMesh(g, mp, *bar, nullptr), "bar mesh");
        PuppetSolver solver(bar);
        int tip = 0, root = 0;
        for (size_t v = 0; v < bar->verts.size(); ++v) {
            if (bar->verts[v].x > bar->verts[tip].x) tip = (int)v;
            if (bar->verts[v].x < bar->verts[root].x) root = (int)v;
        }
        PinInput anchor;
        anchor.src = anchor.dst = {80, 100};
        anchor.radius = 40;
        std::vector<PinInput> pins0 = {anchor}, pins1 = {anchor};
        pins1[0].dst = {80, 160};
        SolverParams sp;
        Deformation stat;
        solver.solve(pins1, sp, stat);
        const double target = stat.pos[tip].y;  // where the tip ends up at rest

        auto run = [&](bool elastic, double firmness, double damping, std::vector<double>& tipY, double& rootErr) {
            DynamicsParams dp;
            dp.elastic = elastic;
            dp.firmness = firmness;
            dp.damping = damping;
            DynamicState st;
            Deformation d;
            solver.step(pins0, sp, dp, st, d);  // frame 0: at rest
            tipY.clear();
            rootErr = 0;
            for (int f = 1; f <= 60; ++f) {
                solver.step(pins1, sp, dp, st, d);
                tipY.push_back(d.pos[tip].y);
                rootErr = std::max(rootErr, std::fabs(d.pos[root].y - stat.pos[root].y));
            }
        };
        std::vector<double> el, sm, firm;
        double rootEl, rootSm, rootFirm;
        run(true, 2.0, 0.15, el, rootEl);
        run(false, 2.0, 0.0, sm, rootSm);
        run(true, 400.0, 0.15, firm, rootFirm);
        const double rise = target - 100.0;
        double maxEl = -1e9;
        for (double y : el) maxEl = std::max(maxEl, y);
        bool smMonotone = true;
        for (size_t i = 1; i < sm.size(); ++i) smMonotone &= sm[i] >= sm[i - 1] - 1e-6;
        std::printf("  target tip y %.1f | elastic f1 %.1f peak %.1f f60 %.1f | smooth f1 %.1f f10 %.1f f60 %.1f | "
                    "firm f1 %.1f\n",
                    target, el[0], maxEl, el.back(), sm[0], sm[9], sm.back(), firm[0]);
        CHECK(el[0] < target - 0.3 * rise, "elastic tip should lag behind at first (%.1f)", el[0]);
        CHECK(maxEl > target + 0.05 * rise, "elastic tip should overshoot (peak %.1f, target %.1f)", maxEl, target);
        CHECK(std::fabs(el.back() - target) < 1.0, "elastic tip should settle (%.2f vs %.2f)", el.back(), target);
        CHECK(sm[0] < target - 0.3 * rise && smMonotone, "smooth follow should lag without bouncing");
        CHECK(std::fabs(sm.back() - target) < 1.0, "smooth follow should arrive (%.2f vs %.2f)", sm.back(), target);
        CHECK(std::fabs(firm[0] - target) < 0.05 * rise, "a very firm material should follow at once (%.1f)", firm[0]);
        CHECK(rootEl < 0.5 && rootSm < 0.5, "the locked end must follow its pin exactly (%.3f, %.3f)", rootEl, rootSm);

        // Weight map, anchor: the right half is locked in place, so the tip stays put.
        std::vector<float> anchorMap(bar->verts.size(), 0.0f);
        for (size_t v = 0; v < bar->verts.size(); ++v) anchorMap[v] = bar->verts[v].x > 260 ? 1.0f : 0.0f;
        SolverParams spa;
        spa.anchor = &anchorMap;
        Deformation da;
        solver.solve(pins1, spa, da);
        double rightMove = 0.0;
        for (size_t v = 0; v < bar->verts.size(); ++v)
            if (bar->verts[v].x > 260) rightMove = std::max(rightMove, length(da.pos[v] - bar->verts[v]));
        // Grey holds part of the motion back.
        for (float& a : anchorMap) a = a > 0.5f ? 0.5f : 0.0f;
        Deformation dg;
        solver.solve(pins1, spa, dg);
        const double greyMove = length(dg.pos[tip] - bar->verts[tip]), freeMove = length(stat.pos[tip] - bar->verts[tip]);
        std::printf("  anchor: locked half moved %.3f px; tip moves %.1f px with grey, %.1f px free\n", rightMove,
                    greyMove, freeMove);
        CHECK(rightMove < 0.5, "anchored vertices moved %.3f px", rightMove);
        CHECK(greyMove < 0.8 * freeMove && greyMove > 0.05 * freeMove, "grey anchor should reduce the motion");

        // Weight map, stiffen: pin both ends and push the middle; a stiff middle bends less.
        PinInput left, right, mid;
        left.src = left.dst = {80, 100};
        right.src = right.dst = {440, 100};
        mid.src = {260, 100};
        mid.dst = {260, 150};
        const std::vector<PinInput> bend = {left, right, mid};
        auto distortion = [&](const Deformation& d, double x0, double x1) {
            double worst = 0.0;
            for (const Tri& t : bar->tris) {
                const Vec2 c = (bar->verts[t[0]] + bar->verts[t[1]] + bar->verts[t[2]]) / 3.0;
                if (c.x < x0 || c.x > x1) continue;
                for (int k = 0; k < 3; ++k) {
                    const double r = length(bar->verts[t[k]] - bar->verts[t[(k + 1) % 3]]);
                    const double l = length(d.pos[t[k]] - d.pos[t[(k + 1) % 3]]);
                    worst = std::max(worst, std::fabs(l - r) / r);
                }
            }
            return worst;
        };
        Deformation soft, stiff;
        solver.solve(bend, sp, soft);
        std::vector<float> stiffMap(bar->tris.size(), 1.0f);
        for (size_t t = 0; t < bar->tris.size(); ++t) {
            const Vec2 c = (bar->verts[bar->tris[t][0]] + bar->verts[bar->tris[t][1]] + bar->verts[bar->tris[t][2]]) / 3.0;
            if (c.x > 200 && c.x < 320) stiffMap[t] = 31.0f;
        }
        SolverParams sps;
        sps.stiffness = &stiffMap;
        solver.solve(bend, sps, stiff);
        const double dSoft = distortion(soft, 200, 320), dStiff = distortion(stiff, 200, 320);
        std::printf("  stiffen: middle edge strain %.4f free, %.4f stiffened\n", dSoft, dStiff);
        CHECK(dStiff < 0.6 * dSoft, "a stiffened region should keep its shape better (%.4f vs %.4f)", dStiff, dSoft);
    }

    {
        Scene sc;
        sc.name = "a_wave";
        sc.src = makeCharacter(960, 540);
        sc.pins = {pin(440, 52, 440, 52), pin(520, 52, 520, 52), pin(480, 200, 480, 200), pin(480, 335, 470, 335),
                   pin(576, 222, 650, 430)};
        runScene(sc, outDir);
    }
    {
        Scene sc;
        sc.name = "b_kick";
        sc.src = makeCharacter(960, 540);
        PinInput head = pin(480, 392, 455, 380, 30.0);
        head.rotation = 0.35;
        sc.pins = {pin(440, 52, 440, 52, 22.0), pin(520, 52, 640, 170), pin(480, 200, 470, 205), head,
                   pin(384, 222, 330, 300), pin(760, 120, 700, 300)};
        runScene(sc, outDir);
    }
    {
        Scene sc;
        sc.name = "c_card";
        sc.src = makeCard(960, 540);
        sc.fullFrame = true;
        sc.aa = 2;
        sc.pins = {pin(10, 10, 10, 10), pin(950, 10, 950, 10), pin(10, 530, 10, 530), pin(950, 530, 950, 530),
                   pin(480, 270, 600, 360)};
        runScene(sc, outDir);
    }
    {
        Scene sc;
        sc.name = "d_hd";
        sc.src = makeCharacter(1920, 1080);
        sc.triangles = 1500;
        sc.savePngs = false;
        sc.pins = {pin(880, 104, 880, 104), pin(1040, 104, 1040, 104), pin(960, 400, 960, 400),
                   pin(960, 670, 940, 670), pin(1152, 444, 1300, 860)};
        runScene(sc, outDir);
    }
    {
        Scene sc;
        sc.name = "e_uhd";
        sc.src = makeCharacter(3840, 2160);
        sc.triangles = 3000;
        sc.savePngs = false;
        sc.aa = 2;
        sc.pins = {pin(1760, 208, 1760, 208), pin(2080, 208, 2080, 208), pin(1920, 800, 1920, 800),
                   pin(1920, 1340, 1880, 1340), pin(2304, 888, 2600, 1720)};
        runScene(sc, outDir);
    }

    std::printf(gFailures ? "\n%d FAILURE(S)\n" : "\nALL OK\n", gFailures);
    return gFailures ? 1 : 0;
}
