// Puppet Pin - deformers: procedural rigs that move pins before the mesh is solved.
//
//   Follower  the first pin leads, the others copy (a part of) its motion, optionally late.
//   Wave      a chain swings around its base pin: a static bend plus an oscillation that
//             travels down the chain (tails, capes, hair).
//   IK        root, joint, end: drag the end pin and the joint pin bends like a knee/elbow
//             so both bone lengths stay the same.
//   FK        a chain posed by joint angles instead of pin positions.
//   Wiggle    every listed pin wanders around randomly (smooth noise), radius pins also turn.
//   Wind      a chain (base first) leans into a wind direction, with gusts that travel down it.
//   Spring    like Follower, but the followers trail the leader on a spring: they lag,
//             overshoot and settle (simulated from the start frame).
//
// Pins are referred to by their 1-based slot number, as shown in the UI.
#pragma once

#include "pp_geom.h"

#include <functional>
#include <string>

namespace pp {

enum class DeformerType { Off = 0, Follower = 1, Wave = 2, IK = 3, FK = 4, Wiggle = 5, Wind = 6, Spring = 7 };

struct Deformer {
    DeformerType type = DeformerType::Off;
    std::vector<int> pins;   // slot numbers, meaning depends on the type (see above)
    double amount = 0.5;     // Follower: share of the leader's motion. Wave: swing (degrees)
    double delay = 0.0;      // frames. Follower: lag. Wave: lag per joint down the chain
    double frequency = 1.0;  // Wave: swings, Wiggle: wiggles, Wind: gusts per second
    double phase = 0.0;      // Wave: degrees
    double angle = 0.0;      // Wave: static bend. FK: angle at the root (degrees)
    double angle2 = 0.0;     // FK: angle at the 1st joint
    double angle3 = 0.0;     // FK: angle at the 2nd joint (and further joints)
    bool flip = false;       // IK: bend the other way
    double size = 20.0;      // Wiggle: how far the pins wander (canonical units)
    double spin = 0.0;       // Wiggle: random turning of radius pins (degrees)
    double direction = 0.0;  // Wind: where the wind blows to (degrees, 0 = right, 90 = up)
    double strength = 20.0;  // Wind: bend per joint in a full gust (degrees)
    double gust = 0.5;       // Wind: 0 = steady, 1 = very gusty
    double seed = 0.0;       // Wiggle / Wind: random pattern
    double firmness = 2.0;   // Spring: wobbles per second
    double damping = 0.2;    // Spring: 0 = wobbles long, 1 = no overshoot
    double start = 0.0;      // Spring: frame where it starts at rest
};

struct RigPin {
    bool valid = false;
    Vec2 rest;
    Vec2 pos;
    double rotation = 0.0;  // radians, added to the pin's own rotation
    bool driven = false;    // position is computed by a deformer (dragging it has no effect)
};

// The pins' own animation at another time (before deformers): fills pins[slot] (valid, rest, pos)
// for every slot listed. pins has the size of the one given to applyDeformers.
using PinsAtTime = std::function<void(double time, const std::vector<int>& slots, std::vector<RigPin>& pins)>;

// pins is indexed by slot number (index 0 unused). Deformers run in order. Follower (with a delay)
// and Spring follow where their leader is at other times: its own animation, moved by the deformers
// above them (so a Spring can trail a pin that a Wave swings); they need pinsAt for that.
void applyDeformers(std::vector<RigPin>& pins, const std::vector<Deformer>& deformers, double time, double fps,
                    const PinsAtTime& pinsAt);

// Wave, Wind and FK chains may branch: each pin after the first hangs off the closest pin listed
// before it (rest positions), so a whole skeleton can be listed in one go (hips, chest, head,
// shoulder, elbow, hand, other shoulder, ...). A plain chain listed from its base outwards stays a
// chain. Returns the parent of every listed pin (index into rest, -1 for the first).
std::vector<int> chainParents(const std::vector<Vec2>& rest);

// "3, 4,5 7" -> {3, 4, 5, 7}. Ignores anything that is not a number in [1, maxSlot].
std::vector<int> parsePinList(const std::string& text, int maxSlot);

}  // namespace pp
