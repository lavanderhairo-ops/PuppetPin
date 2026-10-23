#include "pp_solver.h"

#include <algorithm>
#include <cstring>
#include <queue>

namespace pp {

namespace {
constexpr double kPinWeight = 1e5;  // constraint weight at full strength
constexpr double kRegularize = 1e-9;

double pinWeight(double strength)
{
    const double s = std::min(1.0, std::max(0.0, strength));
    return kPinWeight * s * s * s * s;
}

// Weight-map anchors: white (>= kHardAnchor) locks a vertex in place inside the solve; a grey
// value a removes exactly that share of the vertex's motion afterwards (see blendAnchors).
constexpr float kHardAnchor = 0.99f;

double anchorWeight(double a) { return a >= kHardAnchor ? kPinWeight : 0.0; }

void blendAnchors(const Mesh& M, const std::vector<float>* anchor, std::vector<Vec2>& pos)
{
    if (!anchor || anchor->size() != M.verts.size()) return;
    for (size_t v = 0; v < pos.size(); ++v) {
        const double a = std::min(1.0f, std::max(0.0f, (*anchor)[v]));
        if (a >= kHardAnchor)
            pos[v] = M.verts[v];  // exactly in place (the solve only held it with a strong spring)
        else if (a > 0.0)
            pos[v] = pos[v] + (M.verts[v] - pos[v]) * a;
    }
}

// Stiffness of the area well inside a pin's influence, relative to its falloff ring.
constexpr double kReachCoreStiffness = 40.0;

// A pin whose pull is confined to its influence.
bool limitedPin(const PinInput& p) { return p.influence > 0.0 && pinWeight(p.strength) > 1e-12; }

using Adjacency = std::vector<std::vector<std::pair<int, double>>>;

struct Seed {
    int v;
    double d;
    int owner;
};

// Distance along the mesh edges from the nearest seed, and which seed's owner that is.
void meshDistances(const Adjacency& adj, const std::vector<Seed>& seeds, std::vector<double>& dist,
                   std::vector<int>& owner)
{
    dist.assign(adj.size(), 1e300);
    owner.assign(adj.size(), -1);
    using Item = std::pair<double, int>;
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> pq;
    auto relax = [&](int v, double d, int o) {
        if (d < dist[v]) {
            dist[v] = d;
            owner[v] = o;
            pq.push({d, v});
        }
    };
    for (const Seed& s : seeds) relax(s.v, s.d, s.owner);
    while (!pq.empty()) {
        const auto [d, v] = pq.top();
        pq.pop();
        if (d > dist[v]) continue;
        for (const auto& [u, l] : adj[v]) relax(u, d + l, owner[v]);
    }
}

// Draw order of the triangles where parts overlap (later on top). Every triangle belongs to the pin
// nearest to it along the mesh (owner, -1: none), and whole parts are stacked: a later image layer
// on top, then the part whose pin lies further out from the middle of the body (its thickest point),
// so hands, feet and heads go over the torso. Within a part the further-out triangles go on top.
// Overlapping parts so never mix triangle by triangle. The pins' Layer (which may be animated) is
// applied on top of this order for each pose, see fillInfo.
void drawOrder(const Mesh& M, const Adjacency& adj, const std::vector<int>& pinTri, const std::vector<Vec2>& pinRest,
               std::vector<int>& owner, std::vector<int>& order)
{
    const int nv = (int)M.verts.size(), nt = (int)M.tris.size(), np = (int)pinTri.size();
    // The outline: edges of a single triangle.
    std::vector<std::pair<int, int>> edges;
    edges.reserve((size_t)nt * 3);
    for (const Tri& tr : M.tris)
        for (int k = 0; k < 3; ++k) {
            const int a = tr[k], b = tr[(k + 1) % 3];
            edges.push_back({std::min(a, b), std::max(a, b)});
        }
    std::sort(edges.begin(), edges.end());
    std::vector<Seed> seeds;
    for (size_t i = 0; i < edges.size();) {
        size_t j = i + 1;
        while (j < edges.size() && edges[j] == edges[i]) ++j;
        if (j - i == 1) {
            seeds.push_back({edges[i].first, 0.0, 0});
            seeds.push_back({edges[i].second, 0.0, 0});
        }
        i = j;
    }
    std::vector<double> inner, out, near;
    std::vector<int> unused;
    meshDistances(adj, seeds, inner, unused);
    std::vector<int> core(std::max(0, M.numComps), -1);
    for (int v = 0; v < nv; ++v) {
        const int c = v < (int)M.vertComp.size() ? M.vertComp[v] : -1;
        if (c >= 0 && c < (int)core.size() && (core[c] < 0 || inner[v] > inner[core[c]])) core[c] = v;
    }
    seeds.clear();
    for (int v : core)
        if (v >= 0) seeds.push_back({v, 0.0, 0});
    meshDistances(adj, seeds, out, unused);
    seeds.clear();
    for (int i = 0; i < np; ++i) {
        if (pinTri[i] < 0) continue;
        for (int k = 0; k < 3; ++k) {
            const int v = M.tris[pinTri[i]][k];
            seeds.push_back({v, length(M.verts[v] - pinRest[i]), i});
        }
    }
    std::vector<int> vertOwner;
    meshDistances(adj, seeds, near, vertOwner);

    std::vector<double> pinOut(np, 0.0);
    for (int i = 0; i < np; ++i) {
        if (pinTri[i] < 0) continue;
        double best = 1e300;
        for (int k = 0; k < 3; ++k) {
            const int v = M.tris[pinTri[i]][k];
            best = std::min(best, out[v] + length(M.verts[v] - pinRest[i]));
        }
        pinOut[i] = best;
    }
    std::vector<int> byOut(np);
    for (int i = 0; i < np; ++i) byOut[i] = i;
    std::stable_sort(byOut.begin(), byOut.end(), [&](int a, int b) { return pinOut[a] < pinOut[b]; });
    std::vector<int> rank(np, 0);
    for (int r = 0; r < np; ++r) rank[byOut[r]] = r + 1;

    struct Key {
        int source, part;
        double out;
    };
    const bool layered = M.triSource.size() == M.tris.size();
    std::vector<Key> key(nt);
    owner.assign(nt, -1);
    for (int t = 0; t < nt; ++t) {
        const Tri& tr = M.tris[t];
        const Vec2 cen = (M.verts[tr[0]] + M.verts[tr[1]] + M.verts[tr[2]]) / 3.0;
        int o = -1;
        double best = 1e300;
        for (int k = 0; k < 3; ++k) {
            const int v = tr[k];
            if (vertOwner[v] < 0) continue;
            const double d = near[v] + length(M.verts[v] - cen);
            if (d < best) {
                best = d;
                o = vertOwner[v];
            }
        }
        owner[t] = o;
        key[t] = {layered ? M.triSource[t] : 0, o >= 0 ? rank[o] : 0, (out[tr[0]] + out[tr[1]] + out[tr[2]]) / 3.0};
    }
    order.resize(nt);
    for (int t = 0; t < nt; ++t) order[t] = t;
    std::stable_sort(order.begin(), order.end(), [&](int a, int b) {
        const Key &p = key[a], &q = key[b];
        if (p.source != q.source) return p.source < q.source;
        if (p.part != q.part) return p.part < q.part;
        return p.out < q.out;
    });
}

uint64_t hashFloats(const std::vector<float>* v)
{
    if (!v) return 0;
    uint64_t h = 1469598103934665603ull ^ v->size();
    for (float f : *v) {
        uint32_t q = (uint32_t)std::lround(std::min(1000.0f, std::max(-1000.0f, f)) * 1024.0f);
        h ^= q;
        h *= 1099511628211ull;
    }
    return h;
}
}  // namespace

struct PuppetSolver::Binding {
    struct Pin {
        int tri = -1;
        double bary[3] = {0.0, 0.0, 0.0};
        Vec2 restPt;
        double w = 0.0;
        int comp = -1;
        std::vector<int> locked;
    };
    std::vector<Pin> pins;
    std::vector<uint8_t> compMode;            // 0 static, 1 translate only, 2 full solve
    std::vector<int> local;                   // vertex -> index in the solve system (-1: not solved)
    std::vector<int> verts;                   // solve index -> vertex
    std::vector<int> tris;                    // triangles of fully solved components
    std::vector<int> triOwner;                // per triangle: the pin whose part it is (-1: none)
    std::vector<int> drawOrder;               // triangles bottom to top, before the pins' layers
    std::vector<int> drawRank;                // its inverse
    std::vector<std::array<double, 3>> tw;    // per mesh triangle: edge weights (cotangent x stiffness)
    std::vector<double> anchorW;              // per solve vertex: pull towards the rest pose
    EnvelopeCholesky asap;                    // 2n x 2n, interleaved (x, y)
    EnvelopeCholesky arap;                    // n x n
    double reg = 0.0;                         // Tikhonov weight pulling towards the rest pose
    bool ok = false;
};

PuppetSolver::PuppetSolver(std::shared_ptr<const Mesh> mesh) : mesh_(std::move(mesh))
{
    const Mesh& M = *mesh_;
    cot_.resize(M.tris.size());
    adj_.assign(M.verts.size(), {});
    for (size_t t = 0; t < M.tris.size(); ++t) {
        const Tri& tr = M.tris[t];
        for (int k = 0; k < 3; ++k) {
            const Vec2 u = M.verts[tr[(k + 1) % 3]] - M.verts[tr[k]];
            const Vec2 v = M.verts[tr[(k + 2) % 3]] - M.verts[tr[k]];
            const double cr = std::fabs(cross(u, v));
            double c = cr > 1e-12 ? dot(u, v) / cr : 0.0;
            c = 0.5 * c;
            cot_[t][k] = std::min(50.0, std::max(0.01, c));
        }
        area_ += 0.5 * std::fabs(cross(M.verts[tr[1]] - M.verts[tr[0]], M.verts[tr[2]] - M.verts[tr[0]]));
        for (int k = 0; k < 3; ++k) {
            const int a = tr[k], b = tr[(k + 1) % 3];
            const double l = length(M.verts[a] - M.verts[b]);
            adj_[a].push_back({b, l});
            adj_[b].push_back({a, l});
        }
    }
}

std::shared_ptr<const PuppetSolver::Binding> PuppetSolver::binding(const std::vector<PinInput>& pins,
                                                                   const SolverParams& sp) const
{
    std::vector<double> key;
    key.reserve(pins.size() * 5 + 3);
    key.push_back((double)pins.size());
    key.push_back(sp.pinStiffen);
    const uint64_t ha = hashFloats(sp.anchor), hs = hashFloats(sp.stiffness);
    double d;
    std::memcpy(&d, &ha, sizeof d);
    key.push_back(d);
    std::memcpy(&d, &hs, sizeof d);
    key.push_back(d);
    for (const PinInput& p : pins) {
        key.push_back(p.src.x);
        key.push_back(p.src.y);
        key.push_back(p.radius);
        key.push_back(p.strength);
    }
    // (influence is not part of the binding: it only changes the anchors, which are hashed above)
    std::lock_guard<std::mutex> lock(mtx_);
    for (size_t i = 0; i < cache_.size(); ++i) {
        if (cache_[i].first == key) {
            auto b = cache_[i].second;
            if (i != 0) std::rotate(cache_.begin(), cache_.begin() + i, cache_.begin() + i + 1);
            return b;
        }
    }
    auto b = buildBinding(pins, sp);
    cache_.insert(cache_.begin(), {std::move(key), b});
    if (cache_.size() > 8) cache_.pop_back();
    return b;
}

std::shared_ptr<const PuppetSolver::Binding> PuppetSolver::buildBinding(const std::vector<PinInput>& pins,
                                                                        const SolverParams& sp) const
{
    const Mesh& M = *mesh_;
    auto B = std::make_shared<Binding>();
    const int nv = (int)M.verts.size();
    const int nt = (int)M.tris.size();
    const int nc = M.numComps;
    const double h = M.edgeLength > 0.0 ? M.edgeLength : 1.0;

    B->tw = cot_;
    if (sp.stiffness && (int)sp.stiffness->size() == nt)
        for (int t = 0; t < nt; ++t) {
            const double s = std::min(1e4, std::max(1e-3, (double)(*sp.stiffness)[t]));
            for (int k = 0; k < 3; ++k) B->tw[t][k] *= s;
        }
    std::vector<double> anchorV(nv, 0.0);
    if (sp.anchor && (int)sp.anchor->size() == nv)
        for (int v = 0; v < nv; ++v) anchorV[v] = anchorWeight((*sp.anchor)[v]);

    // Bind pins to the rest mesh.
    B->pins.resize(pins.size());
    for (size_t i = 0; i < pins.size(); ++i) {
        const PinInput& in = pins[i];
        Binding::Pin& bp = B->pins[i];
        bp.w = pinWeight(in.strength);
        if (bp.w <= 1e-12) continue;
        double bary[3];
        const int t = locateTop(M, in.src, bary, 0.75 * h);  // top layer where layers overlap
        if (t >= 0) {
            bp.tri = t;
            for (int k = 0; k < 3; ++k) bp.bary[k] = bary[k];
            const Tri& tr = M.tris[t];
            bp.restPt = M.verts[tr[0]] * bary[0] + M.verts[tr[1]] * bary[1] + M.verts[tr[2]] * bary[2];
            bp.comp = M.triComp[t];
        }
        if (in.radius > 0.0) {
            const double r2 = in.radius * in.radius;
            for (int v = 0; v < nv; ++v)
                if (length2(M.verts[v] - in.src) <= r2) bp.locked.push_back(v);
        }
    }

    // Triangles within ~3 edge lengths of a point pin get stiffer, falling off with distance.
    std::vector<double> pinStiff(nt, 1.0);
    if (sp.pinStiffen > 1.0) {
        const double R = 3.0 * h;
        for (const auto& bp : B->pins) {
            if (bp.tri < 0) continue;
            for (int t = 0; t < nt; ++t) {
                const Tri& tr = M.tris[t];
                if (M.triComp[t] != bp.comp) continue;
                const Vec2 cen = (M.verts[tr[0]] + M.verts[tr[1]] + M.verts[tr[2]]) / 3.0;
                const double d = length(cen - bp.restPt) / R;
                if (d >= 1.0) continue;
                const double f = 1.0 + (sp.pinStiffen - 1.0) * (1.0 - d * d) * (1.0 - d * d);
                pinStiff[t] = std::max(pinStiff[t], f);
            }
        }
        for (int t = 0; t < nt; ++t)
            for (int k = 0; k < 3; ++k) B->tw[t][k] *= pinStiff[t];
    }

    // Decide how each component is solved, from the spread of its constraint points.
    std::vector<double> bx0(nc, 1e300), by0(nc, 1e300), bx1(nc, -1e300), by1(nc, -1e300);
    std::vector<int> cnt(nc, 0);
    auto addPt = [&](int c, const Vec2& p) {
        bx0[c] = std::min(bx0[c], p.x); bx1[c] = std::max(bx1[c], p.x);
        by0[c] = std::min(by0[c], p.y); by1[c] = std::max(by1[c], p.y);
        cnt[c]++;
    };
    for (const auto& bp : B->pins) {
        if (bp.tri >= 0) addPt(bp.comp, bp.restPt);
        for (int v : bp.locked) addPt(M.vertComp[v], M.verts[v]);
    }
    for (int v = 0; v < nv; ++v)
        if (anchorV[v] > 0.0) addPt(M.vertComp[v], M.verts[v]);
    B->compMode.assign(nc, 0);
    for (int c = 0; c < nc; ++c) {
        if (cnt[c] == 0) continue;
        const double spread = std::hypot(bx1[c] - bx0[c], by1[c] - by0[c]);
        B->compMode[c] = spread >= 0.05 * h ? 2 : 1;
    }

    B->local.assign(nv, -1);
    for (int v = 0; v < nv; ++v)
        if (B->compMode[M.vertComp[v]] == 2) {
            B->local[v] = (int)B->verts.size();
            B->verts.push_back(v);
            B->anchorW.push_back(anchorV[v]);
        }
    for (int t = 0; t < nt; ++t)
        if (B->compMode[M.triComp[t]] == 2) B->tris.push_back(t);

    // Which part goes on top where parts overlap.
    {
        std::vector<int> pinTri(pins.size());
        std::vector<Vec2> pinRest(pins.size());
        for (size_t i = 0; i < pins.size(); ++i) {
            pinTri[i] = B->pins[i].tri;
            pinRest[i] = B->pins[i].restPt;
        }
        drawOrder(M, adj_, pinTri, pinRest, B->triOwner, B->drawOrder);
        B->drawRank.resize(B->drawOrder.size());
        for (size_t r = 0; r < B->drawOrder.size(); ++r) B->drawRank[B->drawOrder[r]] = (int)r;
    }

    const int n = (int)B->verts.size();
    if (n == 0) {
        B->ok = true;
        return B;
    }

    const bool withAsap = true;
    std::vector<double> triStiff = pinStiff;
    if (sp.stiffness && (int)sp.stiffness->size() == nt)
        for (int t = 0; t < nt; ++t) triStiff[t] *= std::min(1e4, std::max(1e-3, (double)(*sp.stiffness)[t]));

    for (double reg : {kRegularize, 1e-6, 1e-3}) {
        SparseSymBuilder G(withAsap ? 2 * n : 0), Lap(n);
        assembleLaplacian(*B, reg, Lap);
        for (int t : B->tris) {
            const Tri& tr = M.tris[t];
            if (withAsap) {
                // As-similar-as-possible energy: each vertex expressed in the frame of the opposite edge.
                for (int k = 0; k < 3; ++k) {
                    const int a = tr[k], b = tr[(k + 1) % 3], c = tr[(k + 2) % 3];
                    const Vec2 e = M.verts[b] - M.verts[a];
                    const Vec2 d = M.verts[c] - M.verts[a];
                    const double l2 = length2(e);
                    if (l2 <= 1e-20) continue;
                    const double u = dot(d, e) / l2, v = cross(e, d) / l2;
                    const int la = B->local[a], lb = B->local[b], lc = B->local[c];
                    const int idx[6] = {2 * la, 2 * la + 1, 2 * lb, 2 * lb + 1, 2 * lc, 2 * lc + 1};
                    const double cx[6] = {-1.0 + u, -v, -u, v, 1.0, 0.0};
                    const double cy[6] = {v, -1.0 + u, -v, -u, 0.0, 1.0};
                    for (int p = 0; p < 6; ++p)
                        for (int q = p; q < 6; ++q) {
                            const double val = triStiff[t] * (cx[p] * cx[q] + cy[p] * cy[q]);
                            if (val != 0.0) G.add(idx[p], idx[q], val);
                        }
                }
            }
        }
        for (const auto& bp : B->pins) {
            const double w = bp.w;
            if (bp.tri >= 0 && B->compMode[bp.comp] == 2) {
                const Tri& tr = M.tris[bp.tri];
                for (int p = 0; p < 3; ++p)
                    for (int q = p; q < 3; ++q) {
                        const double val = w * bp.bary[p] * bp.bary[q];
                        const int lp = B->local[tr[p]], lq = B->local[tr[q]];
                        if (withAsap) {
                            G.add(2 * lp, 2 * lq, val);
                            G.add(2 * lp + 1, 2 * lq + 1, val);
                        }
                    }
            }
            for (int v : bp.locked) {
                const int lv = B->local[v];
                if (lv < 0) continue;
                if (withAsap) {
                    G.add(2 * lv, 2 * lv, w);
                    G.add(2 * lv + 1, 2 * lv + 1, w);
                }
            }
        }
        for (int l = 0; l < n && withAsap; ++l) {
            G.add(2 * l, 2 * l, B->anchorW[l] + reg);
            G.add(2 * l + 1, 2 * l + 1, B->anchorW[l] + reg);
        }
        const bool okAsap = !withAsap || B->asap.factor(G);
        if (okAsap && B->arap.factor(Lap)) {
            B->reg = reg;
            B->ok = true;
            break;
        }
    }
    return B;
}

// Global ARAP matrix: cotangent Laplacian + pin, lock and anchor springs + regulariser.
void PuppetSolver::assembleLaplacian(const Binding& B, double reg, SparseSymBuilder& lap) const
{
    const Mesh& M = *mesh_;
    for (int t : B.tris) {
        const Tri& tr = M.tris[t];
        for (int k = 0; k < 3; ++k) {
            const int a = B.local[tr[(k + 1) % 3]], b = B.local[tr[(k + 2) % 3]];
            const double w = B.tw[t][k];
            lap.add(a, a, w);
            lap.add(b, b, w);
            lap.add(a, b, -w);
        }
    }
    for (const auto& bp : B.pins) {
        if (bp.tri >= 0 && B.compMode[bp.comp] == 2) {
            const Tri& tr = M.tris[bp.tri];
            for (int p = 0; p < 3; ++p)
                for (int q = p; q < 3; ++q) lap.add(B.local[tr[p]], B.local[tr[q]], bp.w * bp.bary[p] * bp.bary[q]);
        }
        for (int v : bp.locked)
            if (B.local[v] >= 0) lap.add(B.local[v], B.local[v], bp.w);
    }
    for (int l = 0; l < (int)B.verts.size(); ++l) lap.add(l, l, B.anchorW[l] + reg);
}

void PuppetSolver::fillInfo(const Binding& B, const std::vector<PinInput>& pins, Deformation& out) const
{
    // Draw order: the parts' order, regrouped by their pins' layers (these may be animated).
    bool layers = false;
    for (const PinInput& p : pins) layers = layers || p.layer != 0;
    if (!layers) {
        out.triLayer = B.drawRank;
    } else {
        std::vector<int> order = B.drawOrder;
        auto layerOf = [&](int t) {
            const int o = B.triOwner[t];
            return o >= 0 && o < (int)pins.size() ? pins[o].layer : 0;
        };
        std::stable_sort(order.begin(), order.end(), [&](int a, int b) { return layerOf(a) < layerOf(b); });
        out.triLayer.resize(order.size());
        for (size_t r = 0; r < order.size(); ++r) out.triLayer[order[r]] = (int)r;
    }
    out.pinTri.resize(pins.size());
    out.pinRest.resize(pins.size());
    for (size_t i = 0; i < pins.size(); ++i) {
        out.pinTri[i] = B.pins[i].tri;
        out.pinRest[i] = B.pins[i].tri >= 0 ? B.pins[i].restPt : pins[i].src;
    }
}

// Right-hand side of the pin, anchor and regulariser terms (solve vertices only).
void PuppetSolver::constraintRhs(const Binding& B, const std::vector<PinInput>& pins,
                                 const std::vector<Vec2>* anchorTarget, std::vector<double>& cx,
                                 std::vector<double>& cy) const
{
    const Mesh& M = *mesh_;
    const int n = (int)B.verts.size();
    cx.assign(n, 0.0);
    cy.assign(n, 0.0);
    for (size_t i = 0; i < pins.size() && i < B.pins.size(); ++i) {
        const auto& bp = B.pins[i];
        const double w = bp.w;
        if (bp.tri >= 0 && B.compMode[bp.comp] == 2) {
            const Tri& tr = M.tris[bp.tri];
            for (int k = 0; k < 3; ++k) {
                const int l = B.local[tr[k]];
                const double f = w * bp.bary[k];
                cx[l] += f * pins[i].dst.x;
                cy[l] += f * pins[i].dst.y;
            }
        }
        if (!bp.locked.empty()) {
            const double sc = std::max(1e-3, pins[i].scale);
            const double c = std::cos(pins[i].rotation) * sc, s = std::sin(pins[i].rotation) * sc;
            for (int v : bp.locked) {
                const int l = B.local[v];
                if (l < 0) continue;
                const Vec2 tgt = pins[i].dst + rotate(M.verts[v] - pins[i].src, c, s);
                cx[l] += w * tgt.x;
                cy[l] += w * tgt.y;
            }
        }
    }
    // Anchors pull towards their target (the rest pose unless given); the regulariser towards the rest
    // pose (so it never biases).
    for (int l = 0; l < n; ++l) {
        const int v = B.verts[l];
        const Vec2& r = M.verts[v];
        const Vec2& a = anchorTarget ? (*anchorTarget)[v] : r;
        cx[l] += B.anchorW[l] * a.x + B.reg * r.x;
        cy[l] += B.anchorW[l] * a.y + B.reg * r.y;
    }
}

// Local step (best rotation per triangle) and its contribution to the global right-hand side.
void PuppetSolver::rotationRhs(const Binding& B, const std::vector<double>& X, const std::vector<double>& Y,
                               double rigidity, std::vector<double>& rx, std::vector<double>& ry) const
{
    const Mesh& M = *mesh_;
    rigidity = std::min(1.0, std::max(0.0, rigidity));
    for (int t : B.tris) {
        const Tri& tr = M.tris[t];
        double s00 = 0, s01 = 0, s10 = 0, s11 = 0, den = 0;
        int li[3];
        for (int k = 0; k < 3; ++k) li[k] = B.local[tr[k]];
        for (int k = 0; k < 3; ++k) {
            const int a = (k + 1) % 3, b = (k + 2) % 3;
            const double w = B.tw[t][k];
            const Vec2 e = M.verts[tr[a]] - M.verts[tr[b]];
            const double fx = X[li[a]] - X[li[b]], fy = Y[li[a]] - Y[li[b]];
            s00 += w * fx * e.x; s01 += w * fx * e.y;
            s10 += w * fy * e.x; s11 += w * fy * e.y;
            den += w * length2(e);
        }
        const double tc = s00 + s11, ts = s10 - s01;
        const double mag = std::hypot(tc, ts);
        double c = 1.0, s = 0.0;
        if (mag > 1e-300) { c = tc / mag; s = ts / mag; }
        if (rigidity < 1.0 && den > 0.0) {
            const double sc = std::min(100.0, std::max(0.01, mag / den));
            const double k = std::pow(sc, 1.0 - rigidity);
            c *= k;
            s *= k;
        }
        for (int k = 0; k < 3; ++k) {
            const int a = (k + 1) % 3, b = (k + 2) % 3;
            const double w = B.tw[t][k];
            const Vec2 e = rotate(M.verts[tr[a]] - M.verts[tr[b]], c, s) * w;
            rx[li[a]] += e.x; ry[li[a]] += e.y;
            rx[li[b]] -= e.x; ry[li[b]] -= e.y;
        }
    }
}

// Per vertex: 0 where a pin with a limited influence reaches fully, 1 where none reaches, soft in
// between. Distances are measured along the mesh, so a hand pin does not reach a face next to it.
std::shared_ptr<const std::vector<float>> PuppetSolver::reachMap(const std::vector<PinInput>& pins) const
{
    std::vector<double> key;
    for (const PinInput& p : pins)
        if (limitedPin(p))
            for (double v : {p.src.x, p.src.y, p.influence, p.falloff, p.radius}) key.push_back(v);
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (reach_ && reachKey_ == key) return reach_;
    }
    const Mesh& M = *mesh_;
    const int nv = (int)M.verts.size();
    const double h = M.edgeLength > 0.0 ? M.edgeLength : 1.0;
    std::vector<float> inside(nv, 0.0f);
    std::vector<double> dist(nv);
    using Item = std::pair<double, int>;
    for (const PinInput& p : pins) {
        if (!limitedPin(p)) continue;
        const double R = p.influence;
        std::fill(dist.begin(), dist.end(), 1e300);
        std::priority_queue<Item, std::vector<Item>, std::greater<Item>> pq;
        auto seed = [&](int v, double d) {
            if (d < dist[v] && d < R) {
                dist[v] = d;
                pq.push({d, v});
            }
        };
        double bary[3];
        const int t = locateTop(M, p.src, bary, 0.75 * h);
        if (t >= 0) {
            const Tri& tr = M.tris[t];
            const Vec2 at = M.verts[tr[0]] * bary[0] + M.verts[tr[1]] * bary[1] + M.verts[tr[2]] * bary[2];
            for (int k = 0; k < 3; ++k) seed(tr[k], length(M.verts[tr[k]] - at));
        }
        if (p.radius > 0.0)
            for (int v = 0; v < nv; ++v)
                if (length2(M.verts[v] - p.src) <= p.radius * p.radius) seed(v, 0.0);
        while (!pq.empty()) {
            const auto [d, v] = pq.top();
            pq.pop();
            if (d > dist[v]) continue;
            for (const auto& [u, l] : adj_[v]) seed(u, d + l);
        }
        const double r0 = R * (1.0 - std::min(1.0, std::max(0.0, p.falloff)));
        for (int v = 0; v < nv; ++v) {
            const double d = dist[v];
            double in = 0.0;
            if (d <= r0)
                in = 1.0;
            else if (d < R) {
                const double x = 1.0 - (d - r0) / (R - r0);
                in = x * x * (3.0 - 2.0 * x);
            }
            inside[v] = std::max(inside[v], (float)in);
        }
    }
    auto out = std::make_shared<std::vector<float>>(nv);
    for (int v = 0; v < nv; ++v) (*out)[v] = 1.0f - inside[v];
    std::lock_guard<std::mutex> lock(mtx_);
    reachKey_ = std::move(key);
    reach_ = out;
    return out;
}

// With limited pins: `reach` (see reachMap), `held` = the weight-map anchor combined with it, and
// (optionally) `stiff` = the weight-map stiffness with the cores of the reaches made stiff, so the
// area well inside a reach moves along with its pin and the falloff ring takes the stretch.
bool PuppetSolver::heldAnchor(const std::vector<PinInput>& pins, const SolverParams& sp,
                              std::shared_ptr<const std::vector<float>>& reach, std::vector<float>& held,
                              std::vector<float>* stiff) const
{
    if (std::none_of(pins.begin(), pins.end(), limitedPin)) return false;
    reach = reachMap(pins);
    held = *reach;
    if (sp.anchor && sp.anchor->size() == held.size())
        for (size_t v = 0; v < held.size(); ++v) held[v] = std::max(held[v], std::min(1.0f, (*sp.anchor)[v]));
    if (stiff) {
        const Mesh& M = *mesh_;
        stiff->resize(M.tris.size());
        for (size_t t = 0; t < M.tris.size(); ++t) {
            const Tri& tr = M.tris[t];
            const double in = 1.0 - ((*reach)[tr[0]] + (*reach)[tr[1]] + (*reach)[tr[2]]) / 3.0;
            const double user = sp.stiffness && sp.stiffness->size() == M.tris.size() ? (*sp.stiffness)[t] : 1.0;
            (*stiff)[t] = (float)(user * (1.0 + kReachCoreStiffness * in * in * in));
        }
    }
    return true;
}

void PuppetSolver::solve(const std::vector<PinInput>& pins, const SolverParams& sp, Deformation& out) const
{
    std::shared_ptr<const std::vector<float>> reach;
    std::vector<float> held, stiff;
    if (!heldAnchor(pins, sp, reach, held, &stiff)) {
        solveCore(pins, sp, nullptr, sp.anchor, out);
        return;
    }
    // Pins with a limited influence only pull the area within their reach: beyond every reach the
    // image keeps the pose it has without those pins (posed by the other pins, or at rest).
    std::vector<PinInput> others;
    for (const PinInput& p : pins)
        if (!limitedPin(p)) others.push_back(p);
    Deformation base;
    solveCore(others, sp, nullptr, sp.anchor, base);
    SolverParams full = sp;
    full.anchor = &held;
    full.stiffness = &stiff;
    solveCore(pins, full, &base.pos, sp.anchor, out);
    for (size_t v = 0; v < out.pos.size(); ++v)
        if ((*reach)[v] >= kHardAnchor) out.pos[v] = base.pos[v];  // held by a stiff spring only
}

// One solve. `sp.anchor` gives the anchor weights, `anchorTarget` where anchors pull to (default:
// the rest pose) and `restBlend` the weight-map values blended towards the rest pose afterwards.
void PuppetSolver::solveCore(const std::vector<PinInput>& pins, const SolverParams& sp,
                             const std::vector<Vec2>* anchorTarget, const std::vector<float>* restBlend,
                             Deformation& out) const
{
    const Mesh& M = *mesh_;
    auto B = binding(pins, sp);
    out.pos = M.verts;
    fillInfo(*B, pins, out);

    // Components held by a single point simply translate.
    const int nc = M.numComps;
    std::vector<Vec2> tsum(nc);
    std::vector<double> wsum(nc, 0.0);
    for (size_t i = 0; i < pins.size(); ++i) {
        const auto& bp = B->pins[i];
        if (bp.tri >= 0 && B->compMode[bp.comp] == 1) {
            tsum[bp.comp] += (pins[i].dst - bp.restPt) * bp.w;
            wsum[bp.comp] += bp.w;
        }
        if (!bp.locked.empty()) {
            const double sc = std::max(1e-3, pins[i].scale);
            const double c = std::cos(pins[i].rotation) * sc, s = std::sin(pins[i].rotation) * sc;
            for (int v : bp.locked) {
                const int cmp = M.vertComp[v];
                if (B->compMode[cmp] != 1) continue;
                const Vec2 tgt = pins[i].dst + rotate(M.verts[v] - pins[i].src, c, s);
                tsum[cmp] += (tgt - M.verts[v]) * bp.w;
                wsum[cmp] += bp.w;
            }
        }
    }
    for (size_t v = 0; v < M.verts.size(); ++v) {
        const int c = M.vertComp[v];
        if (B->compMode[c] == 1 && wsum[c] > 0.0) out.pos[v] = M.verts[v] + tsum[c] / wsum[c];
    }

    const int n = (int)B->verts.size();
    if (n == 0 || !B->ok) {
        blendAnchors(M, restBlend, out.pos);
        return;
    }

    std::vector<double> cx, cy;
    constraintRhs(*B, pins, anchorTarget, cx, cy);
    std::vector<double> b2(2 * n), x2(2 * n);
    for (int l = 0; l < n; ++l) {
        b2[2 * l] = cx[l];
        b2[2 * l + 1] = cy[l];
    }
    B->asap.solve(b2.data(), x2.data());
    std::vector<double> X(n), Y(n), rx, ry;
    for (int l = 0; l < n; ++l) {
        X[l] = x2[2 * l];
        Y[l] = x2[2 * l + 1];
    }
    for (int it = 0; it < sp.iterations; ++it) {
        rx = cx;
        ry = cy;
        rotationRhs(*B, X, Y, sp.rigidity, rx, ry);
        B->arap.solve(rx.data(), X.data());
        B->arap.solve(ry.data(), Y.data());
    }
    for (int l = 0; l < n; ++l) out.pos[B->verts[l]] = Vec2(X[l], Y[l]);
    blendAnchors(M, restBlend, out.pos);
}

std::vector<double> PuppetSolver::jiggleWeights(const Binding& B, const SolverParams& sp, double reach,
                                                std::vector<double>* distOut) const
{
    // Distance along the mesh from the nearest pin or locked vertex; the jiggle fades in over
    // `reach`. Anchored (weight map) vertices and stiffened areas jiggle less.
    const Mesh& M = *mesh_;
    const int nv = (int)M.verts.size();
    std::vector<double> dist(nv, 1e300);
    using Item = std::pair<double, int>;
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> pq;
    auto seed = [&](int v, double d) {
        if (d < dist[v]) {
            dist[v] = d;
            pq.push({d, v});
        }
    };
    for (const auto& bp : B.pins) {
        if (bp.tri >= 0)
            for (int k = 0; k < 3; ++k) {
                const int v = M.tris[bp.tri][k];
                seed(v, length(M.verts[v] - bp.restPt));
            }
        for (int v : bp.locked) seed(v, 0.0);
    }
    if (sp.anchor && (int)sp.anchor->size() == nv)
        for (int v = 0; v < nv; ++v)
            if ((*sp.anchor)[v] >= kHardAnchor) seed(v, 0.0);
    while (!pq.empty()) {
        const auto [d, v] = pq.top();
        pq.pop();
        if (d > dist[v]) continue;
        for (const auto& [u, l] : adj_[v]) seed(u, d + l);
    }
    const double lambda = std::max(1e-6, reach * std::sqrt(std::max(1.0, area_)));
    if (distOut) *distOut = dist;
    std::vector<double> w(nv);
    for (int v = 0; v < nv; ++v) {
        const double q = dist[v] / lambda;
        w[v] = dist[v] >= 1e299 ? 1.0 : 1.0 - std::exp(-q * q);
        if (sp.anchor && (int)sp.anchor->size() == nv) w[v] *= 1.0 - std::min(1.0, std::max(0.0, (double)(*sp.anchor)[v]));
    }
    if (sp.stiffness && sp.stiffness->size() == M.tris.size()) {
        std::vector<double> sum(nv, 0.0), cnt(nv, 0.0);
        for (size_t t = 0; t < M.tris.size(); ++t)
            for (int k = 0; k < 3; ++k) {
                sum[M.tris[t][k]] += std::max(1.0, (double)(*sp.stiffness)[t]);
                cnt[M.tris[t][k]] += 1.0;
            }
        for (int v = 0; v < nv; ++v)
            if (cnt[v] > 0.0) w[v] /= sum[v] / cnt[v];
    }
    return w;
}

void PuppetSolver::step(const std::vector<PinInput>& pins, const SolverParams& sp, const DynamicsParams& dp,
                        DynamicState& state, Deformation& out) const
{
    const size_t nv = mesh_->verts.size();
    solve(pins, sp, out);  // the exact pinned pose of this frame
    if (state.empty() || state.pos.size() != nv || state.target.size() != nv) {
        state.pos = out.pos;
        state.vel.assign(nv, Vec2());
        state.target = out.pos;
        return;
    }
    if (state.vel.size() != nv) state.vel.assign(nv, Vec2());
    std::shared_ptr<const std::vector<float>> reach;
    std::vector<float> held, stiff;
    SolverParams bound = sp;  // the binding solve() used (pins are seeds of the jiggle distance)
    if (heldAnchor(pins, sp, reach, held, &stiff)) {
        bound.anchor = &held;
        bound.stiffness = &stiff;
    }
    auto B = binding(pins, bound);
    std::vector<double> dist;
    const std::vector<double> w = jiggleWeights(*B, sp, dp.reach, &dist);

    const double fps = dp.fps > 0.0 ? dp.fps : 24.0;
    const double omega = 2.0 * 3.14159265358979323846 * std::max(0.01, dp.firmness) / fps;  // per frame
    const double zeta = dp.elastic ? 0.03 + 0.97 * std::min(1.0, std::max(0.0, dp.damping)) : 1.0;
    const int sub = std::max(1, dp.substeps);
    const double h = 1.0 / sub, k = omega * omega, c = 2.0 * zeta * omega;
    const double denom = 1.0 + h * c + h * h * k;
    for (size_t v = 0; v < nv; ++v) {
        Vec2 y = state.pos[v], vel = state.vel[v];
        const Vec2 from = state.target[v], to = out.pos[v];
        for (int s = 1; s <= sub; ++s) {
            const Vec2 target = from + (to - from) * ((double)s / sub);
            vel = (vel + (target - y) * (h * k)) / denom;  // implicit Euler: always stable
            y += vel * h;
        }
        state.pos[v] = y;
        state.vel[v] = vel;
        state.target[v] = to;
        // The lag may grow by at most maxSlope pixels per pixel of distance from the pins (soft limit):
        // a held point drags its surroundings along like a flexible rod instead of tearing them, so the
        // shape can never fold over where the lag fades in.
        Vec2 off = (y - to) * w[v];
        if (dp.maxSlope > 0.0 && dist[v] < 1e299) {
            const double lim = dp.maxSlope * dist[v], m = length(off);
            off = lim > 1e-9 ? off / std::sqrt(1.0 + (m / lim) * (m / lim)) : Vec2();
        }
        out.pos[v] = to + off;
    }
}

}  // namespace pp
