#include "pp_rig.h"

#include <algorithm>
#include <cctype>
#include <cstdint>

namespace pp {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;

double angleOf(const Vec2& v) { return std::atan2(v.y, v.x); }

Vec2 rotateAround(const Vec2& p, const Vec2& pivot, double a)
{
    return pivot + rotate(p - pivot, std::cos(a), std::sin(a));
}

// Smooth random value in [-1, 1] (value noise with two octaves), different for each key/channel.
double hash01(int64_t i, int64_t key)
{
    uint64_t h = (uint64_t)i * 0x9E3779B97F4A7C15ull ^ (uint64_t)key * 0xC2B2AE3D27D4EB4Full;
    h ^= h >> 31;
    h *= 0xBF58476D1CE4E5B9ull;
    h ^= h >> 29;
    h *= 0x94D049BB133111EBull;
    h ^= h >> 32;
    return (double)(h & 0xFFFFFFull) / (double)0xFFFFFF * 2.0 - 1.0;
}

double noise1(double x, int64_t key)
{
    const double f = std::floor(x);
    const double t = x - f;
    const double s = t * t * t * (t * (t * 6.0 - 15.0) + 10.0);  // smootherstep
    const int64_t i = (int64_t)f;
    return hash01(i, key) * (1.0 - s) + hash01(i + 1, key) * s;
}

double noise(double x, int64_t key)
{
    return (0.72 * noise1(x, key) + 0.28 * noise1(2.13 * x + 17.31, key * 31 + 7)) / 0.86;
}

int64_t noiseKey(double seed, int slot, int channel)
{
    return (int64_t)std::llround(seed * 1000.0) * 1000003 + slot * 101 + channel;
}

bool usable(const std::vector<RigPin>& pins, int slot)
{
    return slot > 0 && slot < (int)pins.size() && pins[slot].valid;
}

// Where a leader pin is at another time. Returns false if the pin is not valid then.
using LeaderAt = std::function<bool(int slot, double time, Vec2& pos)>;

// A chain as a tree: the usable pins in list order (parents always come before their children).
struct Tree {
    std::vector<int> slot, parent, depth;
    std::vector<std::vector<int>> children;
};

Tree buildTree(const std::vector<RigPin>& pins, const Deformer& d)
{
    Tree t;
    for (int s : d.pins)
        if (usable(pins, s) && std::find(t.slot.begin(), t.slot.end(), s) == t.slot.end()) t.slot.push_back(s);
    std::vector<Vec2> rest;
    for (int s : t.slot) rest.push_back(pins[s].rest);
    t.parent = chainParents(rest);
    t.depth.assign(t.slot.size(), 0);
    t.children.assign(t.slot.size(), {});
    for (size_t i = 1; i < t.slot.size(); ++i) {
        t.depth[i] = t.depth[t.parent[i]] + 1;
        t.children[t.parent[i]].push_back((int)i);
    }
    return t;
}

// Turns the nodes in `roots` and everything hanging off them by a around pivot.
void turnBranches(std::vector<RigPin>& pins, const Tree& t, std::vector<int> roots, const Vec2& pivot, double a)
{
    while (!roots.empty()) {
        const int j = roots.back();
        roots.pop_back();
        RigPin& p = pins[t.slot[j]];
        p.pos = rotateAround(p.pos, pivot, a);
        p.rotation += a;
        roots.insert(roots.end(), t.children[j].begin(), t.children[j].end());
    }
}

void follower(std::vector<RigPin>& pins, const Deformer& d, double time, const LeaderAt& pinAt)
{
    const int lead = d.pins[0];
    if (!usable(pins, lead) || d.pins.size() < 2) return;
    Vec2 leadPos = pins[lead].pos;
    if (d.delay != 0.0 && pinAt) {
        Vec2 p;
        if (pinAt(lead, time - d.delay, p)) leadPos = p;
    }
    const Vec2 move = (leadPos - pins[lead].rest) * d.amount;
    for (size_t k = 1; k < d.pins.size(); ++k) {
        const int f = d.pins[k];
        if (usable(pins, f) && f != lead) pins[f].pos += move;
    }
}

void wave(std::vector<RigPin>& pins, const Deformer& d, double time, double fps)
{
    const Tree t = buildTree(pins, d);
    if (t.slot.size() < 2) return;
    const double seconds = time / (fps > 0.0 ? fps : 24.0);
    const double lagSeconds = d.delay / (fps > 0.0 ? fps : 24.0);
    for (size_t i = 0; i < t.slot.size(); ++i) {
        if (t.children[i].empty()) continue;
        // Every joint bends what hangs off it; the swing reaches each level a little later.
        const double swing =
            d.amount * std::sin(2.0 * kPi * d.frequency * (seconds - t.depth[i] * lagSeconds) + d.phase * kDeg);
        const double a = (d.angle + swing) * kDeg;
        if (a != 0.0) turnBranches(pins, t, t.children[i], pins[t.slot[i]].pos, a);
    }
}

void ik(std::vector<RigPin>& pins, const Deformer& d)
{
    if (d.pins.size() < 3) return;
    const int r = d.pins[0], j = d.pins[1], e = d.pins[2];
    if (!usable(pins, r) || !usable(pins, j) || !usable(pins, e) || r == j || j == e || r == e) return;
    RigPin &R = pins[r], &J = pins[j], &E = pins[e];
    const double l1 = length(J.rest - R.rest), l2 = length(E.rest - J.rest);
    if (l1 <= 1e-9 || l2 <= 1e-9) return;
    // Bend the same way as in the rest pose, unless flipped: +1 puts the joint counter-clockwise
    // of the root->end line.
    double side = cross(E.rest - R.rest, J.rest - R.rest) >= 0.0 ? 1.0 : -1.0;
    if (d.flip) side = -side;

    Vec2 toEnd = E.pos - R.pos;
    double dist = length(toEnd);
    const Vec2 dir = dist > 1e-9 ? toEnd / dist : (E.rest - R.rest) / std::max(1e-9, length(E.rest - R.rest));
    const double minReach = std::fabs(l1 - l2) + 1e-6, maxReach = l1 + l2 - 1e-6;
    if (dist > maxReach) {
        dist = maxReach;  // out of reach: the limb points at the target, fully extended
        E.pos = R.pos + dir * dist;
    } else if (dist < minReach) {
        dist = minReach;
        E.pos = R.pos + dir * dist;
    }
    const double c = std::min(1.0, std::max(-1.0, (l1 * l1 + dist * dist - l2 * l2) / (2.0 * l1 * dist)));
    const double a = std::acos(c) * side;
    const Vec2 jointPos = R.pos + rotate(dir, std::cos(a), std::sin(a)) * l1;

    // Areas locked by radius pins turn with their bone.
    const double turn1 = angleOf(jointPos - R.pos) - angleOf(J.rest - R.rest);
    const double turn2 = angleOf(E.pos - jointPos) - angleOf(E.rest - J.rest);
    J.pos = jointPos;
    J.rotation += turn1;
    J.driven = true;
    E.rotation += turn2;
}

void fk(std::vector<RigPin>& pins, const Deformer& d)
{
    const Tree t = buildTree(pins, d);
    if (t.slot.size() < 2) return;
    // Each pin keeps its rest offset from its parent, turned by all the joints above it: the first
    // pin's joint by Root Angle, the next level by Joint 2 Angle, every level below by Joint 3+ Angle.
    std::vector<double> total(t.slot.size(), 0.0);
    for (size_t i = 1; i < t.slot.size(); ++i) {
        const int par = t.parent[i];
        const int level = t.depth[par];
        const double joint = level == 0 ? d.angle : (level == 1 ? d.angle2 : d.angle3);
        total[i] = total[par] + joint * kDeg;
        RigPin& p = pins[t.slot[i]];
        const RigPin& q = pins[t.slot[par]];
        p.pos = q.pos + rotate(p.rest - q.rest, std::cos(total[i]), std::sin(total[i]));
        p.rotation += total[i];
        p.driven = true;
    }
}

void wiggle(std::vector<RigPin>& pins, const Deformer& d, double time, double fps)
{
    const double x = time / (fps > 0.0 ? fps : 24.0) * d.frequency;
    for (int s : d.pins) {
        if (!usable(pins, s)) continue;
        RigPin& p = pins[s];
        p.pos += Vec2(noise(x, noiseKey(d.seed, s, 0)), noise(x, noiseKey(d.seed, s, 1))) * d.size;
        p.rotation += d.spin * kDeg * noise(x, noiseKey(d.seed, s, 2));
    }
}

void wind(std::vector<RigPin>& pins, const Deformer& d, double time, double fps)
{
    const Tree t = buildTree(pins, d);
    if (t.slot.size() < 2) return;
    const double rate = fps > 0.0 ? fps : 24.0;
    const double seconds = time / rate, lagSeconds = d.delay / rate;
    const Vec2 w(std::cos(d.direction * kDeg), std::sin(d.direction * kDeg));
    const double gust = std::min(1.0, std::max(0.0, d.gust));
    for (size_t i = 0; i < t.slot.size(); ++i) {
        if (t.children[i].empty()) continue;
        // Gust strength reaches each level a little later than the one above it.
        const double tt = seconds - t.depth[i] * lagSeconds;
        const double g = (1.0 - gust) + gust * (0.5 + 0.5 * noise(tt * d.frequency, noiseKey(d.seed, 0, 3)));
        const double flutter = 0.15 * gust * std::sin(2.0 * kPi * 3.0 * d.frequency * tt + d.seed);
        const Vec2 pivot = pins[t.slot[i]].pos;
        for (int c : t.children[i]) {
            const Vec2 seg = pins[t.slot[c]].pos - pivot;
            const double len = length(seg);
            if (len <= 1e-9) continue;
            // Bones across the wind bend the most, bones along it not at all.
            const double a = d.strength * kDeg * (g + flutter) * cross(seg / len, w);
            if (a != 0.0) turnBranches(pins, t, {c}, pivot, a);
        }
    }
}

void spring(std::vector<RigPin>& pins, const Deformer& d, double time, double fps, const LeaderAt& pinAt)
{
    const int lead = d.pins[0];
    if (!usable(pins, lead) || d.pins.size() < 2 || !pinAt) return;
    const Vec2 rest = pins[lead].rest;
    auto moveAt = [&](double t, Vec2& m) {
        Vec2 p;
        if (!pinAt(lead, t, p)) return false;
        m = p - rest;
        return true;
    };
    // The followers' share of the leader's motion runs through a damped spring (implicit Euler,
    // always stable), simulated from the start frame - at most 240 frames back, where an earlier
    // start has long settled.
    const double rate = fps > 0.0 ? fps : 24.0;
    const double first = std::max(d.start, std::floor(time) - 240.0);
    Vec2 y, v, target;
    if (!moveAt(first, y)) return;
    const double omega = 2.0 * kPi * std::max(0.01, d.firmness) / rate;
    const double zeta = 0.03 + 0.97 * std::min(1.0, std::max(0.0, d.damping));
    const int sub = 4;
    const double h = 1.0 / sub, k = omega * omega, c = 2.0 * zeta * omega, denom = 1.0 + h * c + h * h * k;
    Vec2 from = y;
    for (double f = first + 1.0; f <= time + 1e-9; f += 1.0) {
        if (!moveAt(f, target)) target = from;
        for (int s = 1; s <= sub; ++s) {
            const Vec2 tg = from + (target - from) * ((double)s / sub);
            v = (v + (tg - y) * (h * k)) / denom;
            y += v * h;
        }
        from = target;
    }
    const Vec2 move = y * d.amount;
    for (size_t j = 1; j < d.pins.size(); ++j) {
        const int f = d.pins[j];
        if (usable(pins, f) && f != lead) pins[f].pos += move;
    }
}

}  // namespace

void applyDeformers(std::vector<RigPin>& pins, const std::vector<Deformer>& deformers, double time, double fps,
                    const PinsAtTime& pinsAt)
{
    for (size_t j = 0; j < deformers.size(); ++j) {
        const Deformer& d = deformers[j];
        if (d.pins.empty()) continue;
        LeaderAt pinAt;
        if (pinsAt && (d.type == DeformerType::Follower || d.type == DeformerType::Spring)) {
            // The leader at another time: its own animation, moved by the deformers above this one that
            // reach it (directly or through the pins they share), which need their own pins too.
            std::vector<int> need = {d.pins[0]};
            std::vector<Deformer> above;
            for (size_t i = j; i-- > 0;) {
                const Deformer& e = deformers[i];
                bool reaches = false;
                for (int s : e.pins) reaches = reaches || std::find(need.begin(), need.end(), s) != need.end();
                if (!reaches) continue;
                above.insert(above.begin(), e);
                for (int s : e.pins)
                    if (std::find(need.begin(), need.end(), s) == need.end()) need.push_back(s);
            }
            const size_t n = pins.size();
            pinAt = [&pinsAt, need, above, n, fps](int slot, double t, Vec2& out) {
                std::vector<RigPin> p(n);
                pinsAt(t, need, p);
                if (!usable(p, slot)) return false;
                if (!above.empty()) applyDeformers(p, above, t, fps, pinsAt);
                out = p[slot].pos;
                return true;
            };
        }
        switch (d.type) {
            case DeformerType::Follower: follower(pins, d, time, pinAt); break;
            case DeformerType::Wave: wave(pins, d, time, fps); break;
            case DeformerType::IK: ik(pins, d); break;
            case DeformerType::FK: fk(pins, d); break;
            case DeformerType::Wiggle: wiggle(pins, d, time, fps); break;
            case DeformerType::Wind: wind(pins, d, time, fps); break;
            case DeformerType::Spring: spring(pins, d, time, fps, pinAt); break;
            default: break;
        }
    }
}

std::vector<int> chainParents(const std::vector<Vec2>& rest)
{
    std::vector<int> parent(rest.size(), -1);
    for (size_t j = 1; j < rest.size(); ++j) {
        double best = 1e300;
        for (size_t i = 0; i < j; ++i) {
            const double d2 = length2(rest[j] - rest[i]);
            if (d2 < best) {
                best = d2;
                parent[j] = (int)i;
            }
        }
    }
    return parent;
}

std::vector<int> parsePinList(const std::string& text, int maxSlot)
{
    std::vector<int> out;
    int cur = -1;
    auto flush = [&]() {
        if (cur >= 1 && cur <= maxSlot) out.push_back(cur);
        cur = -1;
    };
    for (char ch : text) {
        if (std::isdigit((unsigned char)ch)) {
            cur = (cur < 0 ? 0 : cur) * 10 + (ch - '0');
            if (cur > 100000) cur = 100000;
        } else {
            flush();
        }
    }
    flush();
    return out;
}

}  // namespace pp
