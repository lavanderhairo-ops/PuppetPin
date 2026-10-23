// Debug driver for pin influence: renders the test character with a pin on its right hand dragged
// up and out, with and without an influence, into <outdir>/influence_*.png (cropped, with mesh).
#include "../src/core/pp_mesh.h"
#include "../src/core/pp_render.h"
#include "../src/core/pp_solver.h"
#include "test_util.h"

#include <cstdio>
#include <memory>
#include <string>

using namespace pp;

int main(int argc, char** argv)
{
    const std::string outDir = argc > 1 ? argv[1] : ".";
    ppt::Image src = ppt::makeCharacter(960, 540);
    const ImageView sv = src.view();
    MaskGrid mask;
    buildMaskGrid(sv, Xform{}, 0, 0, src.w, src.h, 1.0, false, mask);
    auto mesh = std::make_shared<Mesh>();
    MeshParams mp;
    mp.targetTriangles = 800;
    buildMesh(mask, mp, *mesh, nullptr);
    PuppetSolver solver(mesh);

    PinInput hand;  // right hand, dragged up and out
    hand.src = {576, 222};
    hand.dst = {640, 300};
    PinInput footL, footR, head;
    footL.src = footL.dst = {440, 52};
    footR.src = footR.dst = {520, 52};
    head.src = head.dst = {480, 392};

    struct Case {
        const char* name;
        double influence, falloff;
        bool holds;
    };
    for (const Case& c : {Case{"none_holds", 0.0, 0.5, true}, Case{"r90", 90.0, 0.5, false},
                          Case{"r140", 140.0, 0.5, false}, Case{"r140_hard", 140.0, 0.1, false},
                          Case{"r140_soft", 140.0, 0.9, false}, Case{"r140_holds", 140.0, 0.5, true}}) {
        PinInput h = hand;
        h.influence = c.influence;
        h.falloff = c.falloff;
        std::vector<PinInput> pins = {h};
        if (c.holds) pins.insert(pins.end(), {footL, footR, head});
        Deformation d;
        solver.solve(pins, SolverParams(), d);
        double worst = 1e300;
        for (const Tri& t : mesh->tris) {
            const double a0 = cross(mesh->verts[t[1]] - mesh->verts[t[0]], mesh->verts[t[2]] - mesh->verts[t[0]]);
            const double a1 = cross(d.pos[t[1]] - d.pos[t[0]], d.pos[t[2]] - d.pos[t[0]]);
            if (std::fabs(a0) > 1e-9) worst = std::min(worst, a1 / a0);
        }
        std::printf("%-12s smallest area ratio %.3f\n", c.name, worst);
        ppt::Image out(src.w, src.h);
        ImageView ov = out.view();
        RenderJob job;
        job.src = &sv;
        job.dst = &ov;
        job.window = {0, 0, src.w, src.h};
        job.mesh = mesh.get();
        job.deformed = &d.pos;
        job.aa = 2;
        renderPuppet(job);
        std::vector<OverlayPin> op;
        for (const PinInput& p : pins) op.push_back({p.dst, false});
        drawBurnIn(ov, {0, 0, src.w, src.h}, mesh.get(), &d.pos, op, Xform{}, 0.35);
        ppt::Image crop(400, 400);
        for (int y = 0; y < 400; ++y)
            for (int x = 0; x < 400; ++x)
                for (int k = 0; k < 4; ++k) crop.at(x, y)[k] = out.at(330 + x, 20 + y)[k];
        ppt::savePNG(outDir + "/influence_" + c.name + ".png", crop);
    }
    return 0;
}
