// Puppet Pin - an OFX image effect that bends images with pins, After Effects style.
//
// Click on the image in the viewer (Resolve: viewer overlay mode "Open FX Overlay") to add
// pins, drag them to pose. The mesh is generated from the clip's alpha and deformed with
// an as-rigid-as-possible solver, so parts bend naturally instead of smearing.
#include "../core/pp_render.h"
#include "../core/pp_rig.h"
#include "../core/pp_solver.h"

#include "ofxDrawSuite.h"
#include "ofxsImageEffect.h"
#include "ofxsInteract.h"
#include "ofxsSupportPrivate.h"

#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <mutex>
#include <string>
#include <vector>

#define kPluginName "Puppet Pin"
#define kPluginGrouping "Puppet Pin"
#define kPluginDescription                                                                                  \
    "Bend and animate images with pins. Click on the image to add pins (viewer overlay: Open FX Overlay), " \
    "drag them to pose. Pins with a radius lock an area and can rotate and scale it."
#define kPluginIdentifier "com.puppetpin.PuppetPin"
#define kPluginVersionMajor 1
#define kPluginVersionMinor 8

namespace {

constexpr int kMaxPins = 48;
constexpr int kMaxDeformers = 8;
constexpr int kMaxLocators = 8;
constexpr int kMaxLayers = 16;  // Source + Layer 2..16
enum OutputMode { kOutComposite = 0, kOutLayer = 1, kOutVectors = 2 };
std::string layerClipName(int i) { return "Layer" + std::to_string(i + 1); }  // i >= 1
constexpr int kMaxSimFrames = 3000;  // longest simulated range (frames)
enum SolverMode { kSolveStatic = 0, kSolveSmooth = 1, kSolveElastic = 2 };
enum AddMode { kAddDoubleClick = 0, kAddSingleClick = 1, kAddButtonOnly = 2 };
constexpr double kDegToRad = 0.017453292519943295;

enum Mode { kModeAnimate = 0, kModeRest = 1 };

// ---------------------------------------------------------------------------------------
// Optional debug log: create %TEMP%\puppetpin_debug to log to %TEMP%\puppetpin_log.txt.

class DebugLog {
public:
    static DebugLog& get()
    {
        static DebugLog log;
        return log;
    }
    bool on() const { return file_ != nullptr; }
    void write(const char* fmt, ...)
    {
        if (!file_) return;
        std::lock_guard<std::mutex> lock(mtx_);
        const double ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_).count();
        std::fprintf(file_, "%10.1f  ", ms);
        va_list ap;
        va_start(ap, fmt);
        std::vfprintf(file_, fmt, ap);
        va_end(ap);
        std::fputc('\n', file_);
        std::fflush(file_);
    }

private:
    DebugLog()
    {
        const char* tmp = std::getenv("TEMP");
        if (!tmp) tmp = std::getenv("TMP");
        if (!tmp) return;
        const std::string dir(tmp);
        if (FILE* f = std::fopen((dir + "\\puppetpin_debug").c_str(), "rb")) {
            std::fclose(f);
            file_ = std::fopen((dir + "\\puppetpin_log.txt").c_str(), "a");
        }
    }
    FILE* file_ = nullptr;
    std::mutex mtx_;
    std::chrono::steady_clock::time_point start_ = std::chrono::steady_clock::now();
};

#define PPLOG(...)                                                \
    do {                                                          \
        if (DebugLog::get().on()) DebugLog::get().write(__VA_ARGS__); \
    } while (0)

std::string pinParam(int i, const char* what) { return "pin" + std::to_string(i + 1) + "_" + what; }

std::string locParam(int i, const char* what) { return "loc" + std::to_string(i + 1) + "_" + what; }

// Where a rest-space point ends up in a deformation, and how much the mesh turned there (radians).
bool trackPoint(const pp::Mesh& M, const std::vector<pp::Vec2>& def, const pp::Vec2& rest, pp::Vec2& pos,
                double& rot)
{
    if (def.size() != M.verts.size()) return false;
    double b[3];
    const int t = pp::locateTop(M, rest, b, 1.5 * M.edgeLength);
    if (t < 0) return false;
    const pp::Tri& tr = M.tris[t];
    pos = def[tr[0]] * b[0] + def[tr[1]] * b[1] + def[tr[2]] * b[2];
    double c = 0.0, sn = 0.0;
    for (int k = 0; k < 3; ++k) {
        const pp::Vec2 er = M.verts[tr[(k + 1) % 3]] - M.verts[tr[k]];
        const pp::Vec2 ed = def[tr[(k + 1) % 3]] - def[tr[k]];
        const double a = std::atan2(pp::cross(er, ed), pp::dot(er, ed));
        c += std::cos(a);
        sn += std::sin(a);
    }
    rot = std::atan2(sn, c);
    return true;
}

// Drag preview: each mesh triangle is split `level` times (4 children per split), about 4000
// sub-triangles in all; sub-triangles are given as barycentric weights of the triangle's corners.
int previewLevel(size_t tris) { return tris * 16 <= 4000 ? 2 : (tris * 4 <= 4000 ? 1 : 0); }

const std::vector<std::array<std::array<double, 3>, 3>>& subTriangles(int level)
{
    static std::vector<std::array<std::array<double, 3>, 3>> cache[3];
    static std::once_flag once;
    std::call_once(once, [] {
        using B = std::array<double, 3>;
        using T = std::array<B, 3>;
        auto mid = [](const B& a, const B& b) { return B{(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2}; };
        cache[0] = {T{B{1, 0, 0}, B{0, 1, 0}, B{0, 0, 1}}};
        for (int l = 1; l < 3; ++l)
            for (const T& t : cache[l - 1]) {
                const B m01 = mid(t[0], t[1]), m12 = mid(t[1], t[2]), m20 = mid(t[2], t[0]);
                cache[l].push_back({t[0], m01, m20});
                cache[l].push_back({m01, t[1], m12});
                cache[l].push_back({m20, m12, t[2]});
                cache[l].push_back({m01, m12, m20});
            }
    });
    return cache[std::min(2, std::max(0, level))];
}

pp::ImageView viewOf(OFX::Image& img)
{
    pp::ImageView v;
    v.data = static_cast<float*>(img.getPixelData());
    const OfxRectI& b = img.getBounds();
    v.x1 = b.x1; v.y1 = b.y1; v.x2 = b.x2; v.y2 = b.y2;
    v.rowBytes = img.getRowBytes();
    return v;
}

int choiceAt(OFX::ChoiceParam* p, double t)
{
    int v = 0;
    p->getValueAtTime(t, v);
    return v;
}

uint64_t mixHash(uint64_t h, double v)
{
    uint64_t bits;
    std::memcpy(&bits, &v, sizeof bits);
    h ^= bits + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2);
    return h;
}

}  // namespace

// =======================================================================================

class PuppetPinPlugin : public OFX::ImageEffect {
public:
    struct PinParams {
        OFX::GroupParam* group = nullptr;
        OFX::BooleanParam* on = nullptr;
        OFX::Double2DParam* pos = nullptr;
        OFX::Double2DParam* src = nullptr;
        OFX::DoubleParam* radius = nullptr;
        OFX::DoubleParam* rotation = nullptr;
        OFX::DoubleParam* scale = nullptr;
        OFX::DoubleParam* strength = nullptr;
        OFX::IntParam* layer = nullptr;
        OFX::DoubleParam* influence = nullptr;
        OFX::DoubleParam* falloff = nullptr;
    };

    struct DeformerParams {
        OFX::GroupParam* group = nullptr;
        OFX::ChoiceParam* type = nullptr;
        OFX::StringParam* pins = nullptr;
        OFX::DoubleParam* amount = nullptr;
        OFX::DoubleParam* delay = nullptr;
        OFX::DoubleParam* bend = nullptr;
        OFX::DoubleParam* swing = nullptr;
        OFX::DoubleParam* speed = nullptr;
        OFX::DoubleParam* lag = nullptr;
        OFX::DoubleParam* phase = nullptr;
        OFX::BooleanParam* flip = nullptr;
        OFX::DoubleParam* angle1 = nullptr;
        OFX::DoubleParam* angle2 = nullptr;
        OFX::DoubleParam* angle3 = nullptr;
        OFX::DoubleParam* size = nullptr;
        OFX::DoubleParam* spin = nullptr;
        OFX::DoubleParam* direction = nullptr;
        OFX::DoubleParam* strength = nullptr;
        OFX::DoubleParam* gust = nullptr;
        OFX::DoubleParam* seed = nullptr;
        OFX::DoubleParam* firm = nullptr;
        OFX::DoubleParam* damp = nullptr;
    };

    struct MeshEntry {
        uint64_t key = 0;
        std::shared_ptr<pp::Mesh> mesh;
        std::shared_ptr<pp::PuppetSolver> solver;
        // Mesh From Alpha, but the Source is opaque everywhere (a background is merged in): the
        // whole frame became the puppet.
        bool opaqueSource = false;
        // Colours of the rest mesh's sub-triangles from the last rendered frame (straight RGBA): the
        // overlay draws them while a pin is dragged, because hosts only re-render after the drag.
        mutable std::mutex previewMtx;
        mutable std::vector<float> preview;
    };

    struct LocatorParams {
        OFX::GroupParam* group = nullptr;
        OFX::BooleanParam* on = nullptr;
        OFX::Double2DParam* rest = nullptr;  // the point of the image it tracks
        OFX::Double2DParam* pos = nullptr;   // baked: where that point is
        OFX::DoubleParam* rot = nullptr;     // baked: how much the image turned there
    };

    // Weight map sampled on the mesh.
    struct Material {
        std::vector<float> anchor;     // per vertex
        std::vector<float> stiffness;  // per triangle
        uint64_t hash = 0;
    };

    // What the overlay needs to draw and hit-test at a given time.
    struct OverlayState {
        std::shared_ptr<const MeshEntry> entry;
        pp::Deformation def;
        std::vector<int> slots;          // pin slot of each entry in pins
        std::vector<pp::PinInput> pins;  // canonical coordinates, after deformers
        std::vector<uint8_t> driven;     // per entry of pins: position computed by a deformer
        std::vector<pp::Deformer> rigs;
        std::vector<std::pair<int, pp::Vec2>> locators;  // slot, rest point (canonical)
        int mode = kModeAnimate;
        bool pastEdge = false;  // Fusion: the posed puppet reaches past the picture, which cuts it off
    };

    explicit PuppetPinPlugin(OfxImageEffectHandle handle);

    void render(const OFX::RenderArguments& args) override;
    bool isIdentity(const OFX::IsIdentityArguments& args, OFX::Clip*& identityClip, double& identityTime) override;
    bool getRegionOfDefinition(const OFX::RegionOfDefinitionArguments& args, OfxRectD& rod) override;
    void getRegionsOfInterest(const OFX::RegionsOfInterestArguments& args, OFX::RegionOfInterestSetter& rois) override;
    void getClipPreferences(OFX::ClipPreferencesSetter& prefs) override;
    void changedParam(const OFX::InstanceChangedArgs& args, const std::string& name) override;
    void purgeCaches() override;

    // ---- used by the overlay ----
    pp::Vec2 toCanonical(double nx, double ny) const;
    void pinFrame(OfxPointD& off, OfxPointD& size) const;
    pp::Vec2 toNormalised(const pp::Vec2& c) const;
    void overlayState(double time, OverlayState& st);
    bool restPointAt(const OverlayState& st, const pp::Vec2& p, pp::Vec2& rest) const;
    int modeAt(double t) { return choiceAt(mode_, t); }
    bool showMesh() { return showMesh_->getValue(); }
    bool showPins() { return showPins_->getValue(); }
    int selectedSlot() { return selected_->getValue() - 1; }
    int renderCount() const { return renders_.load(); }
    void setSelectedSlot(int slot);
    // Every selected pin, the selected pin (the pivot for E / S) first. Several pins are selected with a
    // box or Shift+click in the viewer, or by typing them into Selected Pins.
    std::vector<int> selectedSlots();
    bool isSelected(int slot);
    void setSelection(std::vector<int> slots, int active);
    bool hasRadius(int slot, double time) { return pins_[slot].radius->getValueAtTime(animTime(time)) > 0.0; }
    static bool animated(OFX::ValueParam& p) { return p.getNumKeys() > 0 || p.getIsAnimating(); }
    // Whether a drag sets a keyframe: only on settings that are already animated (keyframes, or a
    // Fusion path / spline), unless "Every Pin" is chosen.
    bool keyOnDrag(OFX::ValueParam& p) { return choiceAt(keyMode_, 0.0) == 1 || animated(p); }
    bool removeSelectedPin();
    int firstFreeSlot();
    int addMode()
    {
        int v = 0;
        addMode_->getValue(v);
        return v;
    }
    pp::Vec2 rawPin(int slot, double time, bool rest);
    void createPin(int slot, const pp::Vec2& rest, const pp::Vec2& pos, double time);
    void movePin(int slot, const pp::Vec2& pos, double time);
    void moveRest(int slot, const pp::Vec2& rest, double time);
    void setInfluence(int slot, double influence, double time);
    void setLocatorRest(int slot, const pp::Vec2& rest, double time);
    double rawRotation(int slot, double time) { return pins_[slot].rotation->getValueAtTime(animTime(time)); }
    void setRotation(int slot, double degrees, double time);
    double rawScale(int slot, double time) { return pins_[slot].scale->getValueAtTime(animTime(time)); }
    void setScale(int slot, double scale, double time);
    void setRadius(int slot, double radius, double time);
    void ensureRadius(int slot, double time);

    // Viewer helpers, cached until a parameter changes (stale: keep the cache while something is
    // being dragged).
    struct Ghost {
        int offset;                  // steps before (< 0) or after (> 0) the current frame
        std::vector<pp::Vec2> pos;   // posed mesh vertices
    };
    struct Trail {
        int slot;
        int first;                   // frame of pts[0]
        std::vector<pp::Vec2> pts;   // where the pin is on each frame (after deformers)
        std::vector<uint8_t> on, keyed;
    };
    bool onionOn() { return onion_->getValue(); }
    bool trailsOn() { return trails_->getValue(); }
    void onionGhosts(double time, bool stale, std::vector<Ghost>& ghosts, std::vector<std::pair<int, int>>& outline);
    void pinTrails(double time, const std::vector<int>& slots, bool stale, std::vector<Trail>& out);

private:
    // Pins' own values at other times, read once per evaluation (deformers that follow a leader read
    // many frames, and so do Smooth Follow / Elastic).
    struct PinMemo {
        std::map<double, std::pair<std::vector<pp::RigPin>, std::vector<uint8_t>>> frames;
    };
    // Enabled pins at `time` (canonical). With rig, deformers are applied to the positions.
    void readPins(double time, std::vector<pp::PinInput>& pins, std::vector<int>& slots, bool rig = true,
                  std::vector<uint8_t>* driven = nullptr, PinMemo* memo = nullptr);
    // Something changed: viewer helpers recompute.
    void touched() { ++paramGen_; }
    std::atomic<uint64_t> paramGen_{1};
    // Pose tools (Animation section) and rig files.
    void keyAllPins(double time);
    void copyPose(double time);
    void pastePose(double time);
    void mirrorPose(double time);
    void setPose(int slot, const pp::Vec2& normPos, double rotation, double scale, double time);
    void saveRig(double time);
    void loadRig();
    std::string rigPath();
    void updateHelperVisibility();
    void readDeformers(double time, std::vector<pp::Deformer>& out);
    double animTime(double time);
    void updateLoopVisibility();
    int simStartFrame(double time);
    void updateDeformerVisibility();
    pp::SolverParams solverParams(double time, const Material* mat = nullptr);
    // Deformed mesh at `time`: a static solve, or the Smooth Follow / Elastic simulation.
    void pose(double time, const MeshEntry& e, const Material& mat, const std::vector<pp::PinInput>& pins,
              pp::Deformation& def);
    void sampleWeights(OFX::Image& img, const pp::Xform& toPix, const pp::Mesh& mesh, double time, Material& out);
    void updatePreview(const MeshEntry& e, const std::vector<pp::ImageView>& layers, const pp::Xform& toPix);
    void updateSolverVisibility();
    std::atomic<int> renders_{0};
    std::atomic<int> opaqueWarning_{-1};
    // Where the pins' 0..1 range lies in the picture (canonical pixels). Size 0: the whole project
    // frame. "Puppet Pin - Add Room" puts the Source in a bigger picture and sets this to where the
    // original picture went, so every pin (keyframes, paths, expressions) keeps its meaning.
    OFX::Double2DParam* pinFrameOrigin_ = nullptr;
    OFX::Double2DParam* pinFrameSize_ = nullptr;  // what the persistent "opaque source" message shows (-1 unknown)
    void showOpaqueWarning(bool on);
    std::shared_ptr<const MeshEntry> meshFor(const std::vector<pp::ImageView>& layers,
                                             const std::vector<OfxRectD>& rods, const pp::Xform& toPix,
                                             double time);
    void clearPin(int slot);
    void updatePinVisibility();
    void updateOutputVisibility();
    bool anyLayerConnected() const;
    void updateLocatorVisibility();
    void bakeLocators();

    OFX::Clip* dstClip_ = nullptr;
    OFX::Clip* srcClip_ = nullptr;
    OFX::ChoiceParam* mode_ = nullptr;
    OFX::ChoiceParam* keyMode_ = nullptr;  // keyframes when dragging: 0 only keyframed pins, 1 every pin
    OFX::ChoiceParam* loopMode_ = nullptr;
    OFX::IntParam* loopStart_ = nullptr;
    OFX::IntParam* loopEnd_ = nullptr;
    OFX::BooleanParam* showMesh_ = nullptr;
    OFX::BooleanParam* showPins_ = nullptr;
    OFX::IntParam* selected_ = nullptr;
    OFX::StringParam* selection_ = nullptr;  // every selected pin ("8, 9, 10"), the selected pin among them
    bool selecting_ = false;                 // setSelection is changing the two
    OFX::IntParam* triangles_ = nullptr;
    OFX::DoubleParam* expansion_ = nullptr;
    OFX::DoubleParam* threshold_ = nullptr;
    OFX::ChoiceParam* meshSource_ = nullptr;
    OFX::DoubleParam* rigidity_ = nullptr;
    OFX::IntParam* iterations_ = nullptr;
    OFX::ChoiceParam* aa_ = nullptr;
    OFX::BooleanParam* keepUnmeshed_ = nullptr;
    OFX::BooleanParam* burnIn_ = nullptr;
    OFX::ChoiceParam* addMode_ = nullptr;
    OFX::Clip* weightClip_ = nullptr;
    std::vector<OFX::Clip*> layerClips_;  // [0] unused (the Source), [i] = "Layer i+1"
    OFX::ChoiceParam* outputMode_ = nullptr;
    OFX::IntParam* outputLayer_ = nullptr;
    OFX::ChoiceParam* solverMode_ = nullptr;
    OFX::DoubleParam* firmness_ = nullptr;
    OFX::DoubleParam* damping_ = nullptr;
    OFX::DoubleParam* reach_ = nullptr;
    OFX::ChoiceParam* simStartMode_ = nullptr;
    OFX::IntParam* simStart_ = nullptr;
    OFX::ChoiceParam* weightMode_ = nullptr;
    OFX::ChoiceParam* weightChannel_ = nullptr;
    OFX::BooleanParam* weightInvert_ = nullptr;
    OFX::BooleanParam* onion_ = nullptr;
    OFX::IntParam* onionBefore_ = nullptr;
    OFX::IntParam* onionAfter_ = nullptr;
    OFX::IntParam* onionStep_ = nullptr;
    OFX::BooleanParam* trails_ = nullptr;
    OFX::IntParam* trailFrames_ = nullptr;
    OFX::BooleanParam* mblur_ = nullptr;
    OFX::DoubleParam* shutter_ = nullptr;
    OFX::IntParam* mblurSamples_ = nullptr;
    OFX::StringParam* rigFile_ = nullptr;
    std::mutex helperMtx_;
    struct GhostCache {
        uint64_t gen = 0;
        const void* entry = nullptr;
        int frame = 0, before = 0, after = 0, step = 0;
        std::vector<Ghost> ghosts;
        std::vector<std::pair<int, int>> outline;
    } ghostCache_;
    struct TrailCache {
        uint64_t gen = 0;
        int frame = 0, frames = 0;
        std::vector<int> slots;
        std::vector<Trail> trails;
    } trailCache_;
    std::vector<PinParams> pins_;
    std::vector<DeformerParams> defs_;
    std::vector<LocatorParams> locs_;

    std::mutex cacheMtx_;
    std::deque<std::shared_ptr<MeshEntry>> meshCache_;
    std::shared_ptr<const MeshEntry> lastEntry_;
    Material lastMaterial_;  // weight map of the last render (the overlay cannot fetch images)

    struct SimFrame {
        std::vector<pp::PinInput> pins;
        pp::DynamicState state;
        std::vector<pp::Vec2> display;
    };
    std::mutex simMtx_;
    uint64_t simConfig_ = 0;
    std::vector<SimFrame> simFrames_;  // frames from the simulation start
};

PuppetPinPlugin::PuppetPinPlugin(OfxImageEffectHandle handle) : ImageEffect(handle)
{
    dstClip_ = fetchClip(kOfxImageEffectOutputClipName);
    srcClip_ = fetchClip(kOfxImageEffectSimpleSourceClipName);
    mode_ = fetchChoiceParam("mode");
    keyMode_ = fetchChoiceParam("keyMode");
    loopMode_ = fetchChoiceParam("loopMode");
    loopStart_ = fetchIntParam("loopStart");
    loopEnd_ = fetchIntParam("loopEnd");
    showMesh_ = fetchBooleanParam("showMesh");
    pinFrameOrigin_ = fetchDouble2DParam("pinFrameOrigin");
    pinFrameSize_ = fetchDouble2DParam("pinFrameSize");
    showPins_ = fetchBooleanParam("showPins");
    selected_ = fetchIntParam("selectedPin");
    selection_ = fetchStringParam("selectedPins");
    triangles_ = fetchIntParam("triangles");
    expansion_ = fetchDoubleParam("expansion");
    threshold_ = fetchDoubleParam("threshold");
    meshSource_ = fetchChoiceParam("meshSource");
    rigidity_ = fetchDoubleParam("rigidity");
    iterations_ = fetchIntParam("iterations");
    aa_ = fetchChoiceParam("antialias");
    keepUnmeshed_ = fetchBooleanParam("keepUnmeshed");
    burnIn_ = fetchBooleanParam("burnIn");
    addMode_ = fetchChoiceParam("addMode");
    weightClip_ = fetchClip("WeightMap");
    layerClips_.assign(kMaxLayers, nullptr);
    for (int i = 1; i < kMaxLayers; ++i) layerClips_[i] = fetchClip(layerClipName(i));
    outputMode_ = fetchChoiceParam("outputMode");
    outputLayer_ = fetchIntParam("outputLayer");
    solverMode_ = fetchChoiceParam("solverMode");
    firmness_ = fetchDoubleParam("firmness");
    damping_ = fetchDoubleParam("damping");
    reach_ = fetchDoubleParam("jiggleReach");
    simStartMode_ = fetchChoiceParam("simStartMode");
    simStart_ = fetchIntParam("simStart");
    weightMode_ = fetchChoiceParam("weightMode");
    weightChannel_ = fetchChoiceParam("weightChannel");
    weightInvert_ = fetchBooleanParam("weightInvert");
    onion_ = fetchBooleanParam("onionSkin");
    onionBefore_ = fetchIntParam("onionBefore");
    onionAfter_ = fetchIntParam("onionAfter");
    onionStep_ = fetchIntParam("onionStep");
    trails_ = fetchBooleanParam("motionTrails");
    trailFrames_ = fetchIntParam("trailFrames");
    mblur_ = fetchBooleanParam("motionBlur");
    shutter_ = fetchDoubleParam("shutter");
    mblurSamples_ = fetchIntParam("blurSamples");
    rigFile_ = fetchStringParam("rigFile");
    pins_.resize(kMaxPins);
    for (int i = 0; i < kMaxPins; ++i) {
        PinParams& p = pins_[i];
        p.group = fetchGroupParam(pinParam(i, "group"));
        p.on = fetchBooleanParam(pinParam(i, "on"));
        p.pos = fetchDouble2DParam(pinParam(i, "pos"));
        p.src = fetchDouble2DParam(pinParam(i, "rest"));
        p.radius = fetchDoubleParam(pinParam(i, "radius"));
        p.rotation = fetchDoubleParam(pinParam(i, "rotation"));
        p.scale = fetchDoubleParam(pinParam(i, "scale"));
        p.strength = fetchDoubleParam(pinParam(i, "strength"));
        p.influence = fetchDoubleParam(pinParam(i, "influence"));
        p.falloff = fetchDoubleParam(pinParam(i, "falloff"));
        p.layer = fetchIntParam(pinParam(i, "layer"));
    }
    defs_.resize(kMaxDeformers);
    for (int k = 0; k < kMaxDeformers; ++k) {
        DeformerParams& d = defs_[k];
        const std::string n = "def" + std::to_string(k + 1) + "_";
        d.group = fetchGroupParam(n + "group");
        d.type = fetchChoiceParam(n + "type");
        d.pins = fetchStringParam(n + "pins");
        d.amount = fetchDoubleParam(n + "amount");
        d.delay = fetchDoubleParam(n + "delay");
        d.bend = fetchDoubleParam(n + "bend");
        d.swing = fetchDoubleParam(n + "swing");
        d.speed = fetchDoubleParam(n + "speed");
        d.lag = fetchDoubleParam(n + "lag");
        d.phase = fetchDoubleParam(n + "phase");
        d.flip = fetchBooleanParam(n + "flip");
        d.angle1 = fetchDoubleParam(n + "angle1");
        d.angle2 = fetchDoubleParam(n + "angle2");
        d.angle3 = fetchDoubleParam(n + "angle3");
        d.size = fetchDoubleParam(n + "size");
        d.spin = fetchDoubleParam(n + "spin");
        d.direction = fetchDoubleParam(n + "direction");
        d.strength = fetchDoubleParam(n + "strength");
        d.gust = fetchDoubleParam(n + "gust");
        d.seed = fetchDoubleParam(n + "seed");
        d.firm = fetchDoubleParam(n + "firm");
        d.damp = fetchDoubleParam(n + "damp");
    }
    locs_.resize(kMaxLocators);
    for (int k = 0; k < kMaxLocators; ++k) {
        LocatorParams& l = locs_[k];
        l.group = fetchGroupParam(locParam(k, "group"));
        l.on = fetchBooleanParam(locParam(k, "on"));
        l.rest = fetchDouble2DParam(locParam(k, "rest"));
        l.pos = fetchDouble2DParam(locParam(k, "pos"));
        l.rot = fetchDoubleParam(locParam(k, "rot"));
    }
    updatePinVisibility();
    updateLocatorVisibility();
    updateDeformerVisibility();
    updateSolverVisibility();
    updateOutputVisibility();
    updateLoopVisibility();
    updateHelperVisibility();
    PPLOG("instance created: project size %.1f x %.1f offset %.1f %.1f par %.3f", getProjectSize().x,
          getProjectSize().y, getProjectOffset().x, getProjectOffset().y, getProjectPixelAspectRatio());
}

void PuppetPinPlugin::pinFrame(OfxPointD& off, OfxPointD& size) const
{
    off = getProjectOffset();
    size = getProjectSize();
    double sx = 0.0, sy = 0.0;
    pinFrameSize_->getValue(sx, sy);
    if (sx > 0.0 && sy > 0.0) {
        pinFrameOrigin_->getValue(off.x, off.y);
        size = {sx, sy};
    }
}

pp::Vec2 PuppetPinPlugin::toCanonical(double nx, double ny) const
{
    OfxPointD off, size;
    pinFrame(off, size);
    return {off.x + nx * size.x, off.y + ny * size.y};
}

pp::Vec2 PuppetPinPlugin::toNormalised(const pp::Vec2& c) const
{
    OfxPointD off, size;
    pinFrame(off, size);
    return {size.x != 0.0 ? (c.x - off.x) / size.x : 0.5, size.y != 0.0 ? (c.y - off.y) / size.y : 0.5};
}

// Pin animation loop: past Loop End the keyframes between Loop Start and Loop End repeat (Loop) or
// play back and forth (Ping-Pong). Returns the time to read pin values at.
double PuppetPinPlugin::animTime(double time)
{
    const int mode = choiceAt(loopMode_, 0.0);
    if (mode == 0) return time;
    const double a = loopStart_->getValue(), b = loopEnd_->getValue(), len = b - a;
    if (!(len > 0.0) || time <= b) return time;
    if (mode == 1) return a + std::fmod(time - a, len);
    const double q = std::fmod(time - a, 2.0 * len);
    return a + (q <= len ? q : 2.0 * len - q);
}

void PuppetPinPlugin::updateLoopVisibility()
{
    int m = 0;
    loopMode_->getValue(m);
    for (OFX::Param* q : std::initializer_list<OFX::Param*>{loopStart_, loopEnd_})
        if (q->getIsSecret() == (m != 0)) q->setIsSecret(m == 0);
}

// Frame where simulations (Smooth Follow / Elastic, Spring deformers) start at rest.
int PuppetPinPlugin::simStartFrame(double time)
{
    if (choiceAt(simStartMode_, time) == 0) {
        const OfxRangeD range = srcClip_->getFrameRange();
        return (int)std::floor(range.min + 0.5);
    }
    return simStart_->getValue();
}

void PuppetPinPlugin::readDeformers(double t, std::vector<pp::Deformer>& out)
{
    out.clear();
    for (int k = 0; k < kMaxDeformers; ++k) {
        DeformerParams& p = defs_[k];
        const int type = choiceAt(p.type, t);
        if (type <= 0 || type > 7) continue;
        std::string list;
        p.pins->getValueAtTime(t, list);
        pp::Deformer d;
        d.type = (pp::DeformerType)type;
        d.pins = pp::parsePinList(list, kMaxPins);
        if (d.pins.empty()) continue;
        switch (d.type) {
            case pp::DeformerType::Follower:
                d.amount = p.amount->getValueAtTime(t);
                d.delay = p.delay->getValueAtTime(t);
                break;
            case pp::DeformerType::Wave:
                d.angle = p.bend->getValueAtTime(t);
                d.amount = p.swing->getValueAtTime(t);
                d.frequency = p.speed->getValueAtTime(t);
                d.delay = p.lag->getValueAtTime(t);
                d.phase = p.phase->getValueAtTime(t);
                break;
            case pp::DeformerType::IK:
                d.flip = p.flip->getValueAtTime(t);
                break;
            case pp::DeformerType::FK:
                d.angle = p.angle1->getValueAtTime(t);
                d.angle2 = p.angle2->getValueAtTime(t);
                d.angle3 = p.angle3->getValueAtTime(t);
                break;
            case pp::DeformerType::Wiggle:
                d.size = p.size->getValueAtTime(t);
                d.frequency = p.speed->getValueAtTime(t);
                d.spin = p.spin->getValueAtTime(t);
                d.seed = p.seed->getValueAtTime(t);
                break;
            case pp::DeformerType::Wind:
                d.direction = p.direction->getValueAtTime(t);
                d.strength = p.strength->getValueAtTime(t);
                d.gust = p.gust->getValueAtTime(t);
                d.frequency = p.speed->getValueAtTime(t);
                d.delay = p.lag->getValueAtTime(t);
                d.seed = p.seed->getValueAtTime(t);
                break;
            case pp::DeformerType::Spring:
                d.amount = p.amount->getValueAtTime(t);
                d.firmness = p.firm->getValueAtTime(t);
                d.damping = p.damp->getValueAtTime(t);
                d.start = simStartFrame(t);
                break;
            default: break;
        }
        out.push_back(d);
    }
}

void PuppetPinPlugin::readPins(double t, std::vector<pp::PinInput>& out, std::vector<int>& slots, bool rig,
                               std::vector<uint8_t>* driven, PinMemo* memo)
{
    out.clear();
    slots.clear();
    if (driven) driven->clear();
    const double ta = animTime(t);
    for (int i = 0; i < kMaxPins; ++i) {
        PinParams& p = pins_[i];
        if (!p.on->getValueAtTime(ta)) continue;
        pp::PinInput in;
        double x, y;
        p.src->getValueAtTime(ta, x, y);
        in.src = toCanonical(x, y);
        p.pos->getValueAtTime(ta, x, y);
        in.dst = toCanonical(x, y);
        in.radius = std::max(0.0, p.radius->getValueAtTime(ta));
        in.rotation = p.rotation->getValueAtTime(ta) * kDegToRad;
        in.scale = p.scale->getValueAtTime(ta);
        in.strength = p.strength->getValueAtTime(ta);
        in.layer = p.layer->getValueAtTime(ta);
        in.influence = std::max(0.0, p.influence->getValueAtTime(ta));
        in.falloff = p.falloff->getValueAtTime(ta);
        out.push_back(in);
        slots.push_back(i);
    }
    if (driven) driven->assign(out.size(), 0);
    if (!rig || out.empty()) return;
    std::vector<pp::Deformer> defs;
    readDeformers(ta, defs);
    if (defs.empty()) return;
    std::vector<pp::RigPin> rp(kMaxPins + 1);
    for (size_t k = 0; k < out.size(); ++k) {
        pp::RigPin& r = rp[slots[k] + 1];
        r.valid = true;
        r.rest = out[k].src;
        r.pos = out[k].dst;
    }
    PinMemo localMemo;
    PinMemo& mm = memo ? *memo : localMemo;
    auto pinsAt = [&](double tt, const std::vector<int>& want, std::vector<pp::RigPin>& pinsOut) {
        auto& fr = mm.frames[tt];
        if (fr.first.empty()) {
            fr.first.resize(kMaxPins + 1);
            fr.second.assign(kMaxPins + 1, 0);
        }
        const double at = animTime(tt);
        for (int slot : want) {
            if (slot < 1 || slot > kMaxPins || slot >= (int)pinsOut.size()) continue;
            if (!fr.second[slot]) {
                fr.second[slot] = 1;
                PinParams& p = pins_[slot - 1];
                pp::RigPin& r = fr.first[slot];
                if (p.on->getValueAtTime(at)) {
                    double x, y;
                    p.src->getValueAtTime(at, x, y);
                    r.rest = toCanonical(x, y);
                    p.pos->getValueAtTime(at, x, y);
                    r.pos = toCanonical(x, y);
                    r.valid = true;
                }
            }
            pinsOut[slot] = fr.first[slot];
        }
    };
    double fps = getFrameRate();
    if (!(fps > 0.0)) fps = 24.0;
    pp::applyDeformers(rp, defs, t, fps, pinsAt);
    for (size_t k = 0; k < out.size(); ++k) {
        const pp::RigPin& r = rp[slots[k] + 1];
        out[k].dst = r.pos;
        out[k].rotation += r.rotation;
        if (driven) (*driven)[k] = r.driven ? 1 : 0;
    }
}

// Averages the source around each rest sub-triangle's centre into straight RGBA.
void PuppetPinPlugin::updatePreview(const MeshEntry& e, const std::vector<pp::ImageView>& layers,
                                    const pp::Xform& toPix)
{
    const pp::Mesh& M = *e.mesh;
    const auto& subs = subTriangles(previewLevel(M.tris.size()));
    std::vector<float> col;
    col.reserve(M.tris.size() * subs.size() * 4);
    for (size_t ti = 0; ti < M.tris.size(); ++ti) {
        const pp::Tri& t = M.tris[ti];
        const size_t li = M.triSource.size() == M.tris.size() ? (size_t)M.triSource[ti] : 0;
        const pp::ImageView& src = li < layers.size() && layers[li].valid() ? layers[li] : layers[0];
        for (const auto& sub : subs) {
            pp::Vec2 c[3];
            for (int k = 0; k < 3; ++k)
                c[k] = M.verts[t[0]] * sub[k][0] + M.verts[t[1]] * sub[k][1] + M.verts[t[2]] * sub[k][2];
            const pp::Vec2 cen = (c[0] + c[1] + c[2]) / 3.0;
            float acc[4] = {0, 0, 0, 0};
            const pp::Vec2 taps[4] = {cen, (cen + c[0]) / 2.0, (cen + c[1]) / 2.0, (cen + c[2]) / 2.0};
            for (const pp::Vec2& q : taps) {
                const pp::Vec2 px = toPix.apply(q);
                const int x = std::min(src.x2 - 1, std::max(src.x1, (int)std::floor(px.x)));
                const int y = std::min(src.y2 - 1, std::max(src.y1, (int)std::floor(px.y)));
                const float* v = src.pixel(x, y);
                for (int k = 0; k < 4; ++k) acc[k] += 0.25f * v[k];
            }
            const float a = acc[3];
            col.insert(col.end(), {a > 1e-4f ? acc[0] / a : 0.0f, a > 1e-4f ? acc[1] / a : 0.0f,
                                   a > 1e-4f ? acc[2] / a : 0.0f, std::min(1.0f, std::max(0.0f, a))});
        }
    }
    std::lock_guard<std::mutex> lock(e.previewMtx);
    e.preview.swap(col);
}

pp::SolverParams PuppetPinPlugin::solverParams(double time, const Material* mat)
{
    pp::SolverParams sp;
    sp.rigidity = rigidity_->getValueAtTime(time);
    sp.iterations = std::max(0, iterations_->getValueAtTime(time));
    if (mat && !mat->anchor.empty()) sp.anchor = &mat->anchor;
    if (mat && !mat->stiffness.empty()) sp.stiffness = &mat->stiffness;
    return sp;
}

namespace {

bool samePins(const std::vector<pp::PinInput>& a, const std::vector<pp::PinInput>& b)
{
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i) {
        const pp::PinInput &p = a[i], &q = b[i];
        if (p.src.x != q.src.x || p.src.y != q.src.y || p.dst.x != q.dst.x || p.dst.y != q.dst.y ||
            p.radius != q.radius || p.rotation != q.rotation || p.scale != q.scale || p.strength != q.strength ||
            p.layer != q.layer || p.influence != q.influence || p.falloff != q.falloff)
            return false;
    }
    return true;
}

}  // namespace

void PuppetPinPlugin::pose(double time, const MeshEntry& e, const Material& mat, const std::vector<pp::PinInput>& pins,
                           pp::Deformation& def)
{
    const pp::SolverParams sp = solverParams(time, &mat);
    const int mode = choiceAt(solverMode_, time);
    e.solver->solve(pins, sp, def);
    if (mode == kSolveStatic) return;

    pp::DynamicsParams dp;
    dp.elastic = mode == kSolveElastic;
    dp.firmness = firmness_->getValueAtTime(time);
    dp.damping = damping_->getValueAtTime(time);
    dp.reach = reach_->getValueAtTime(time);
    dp.fps = getFrameRate() > 0.0 ? getFrameRate() : 24.0;
    int start = simStartFrame(time);
    const int f = (int)std::floor(time + 0.5);
    if (f <= start) return;  // the first frame is the pinned pose, at rest
    if (f - start > kMaxSimFrames) start = f - kMaxSimFrames;

    uint64_t cfg = mixHash(e.key, (double)mat.hash);
    for (double v : {(double)mode, dp.firmness, dp.damping, dp.reach, dp.fps, sp.rigidity, (double)sp.iterations,
                     (double)start})
        cfg = mixHash(cfg, v);

    std::lock_guard<std::mutex> lock(simMtx_);
    if (simConfig_ != cfg) {
        simConfig_ = cfg;
        simFrames_.clear();
    }
    // Re-simulate from the first frame whose pins differ from the cached run.
    std::vector<pp::PinInput> pk;
    std::vector<int> slots;
    int computed = 0;
    PinMemo memo;
    for (int k = start; k <= f; ++k) {
        const size_t idx = (size_t)(k - start);
        readPins(k, pk, slots, true, nullptr, &memo);
        if (idx < simFrames_.size() && samePins(simFrames_[idx].pins, pk)) continue;
        simFrames_.resize(idx);
        SimFrame fr;
        fr.pins = pk;
        if (idx > 0) fr.state = simFrames_[idx - 1].state;
        pp::Deformation d;
        e.solver->step(pk, sp, dp, fr.state, d);
        fr.display = std::move(d.pos);
        simFrames_.push_back(std::move(fr));
        ++computed;
    }
    const SimFrame& last = simFrames_[(size_t)(f - start)];
    if (last.display.size() == def.pos.size()) def.pos = last.display;
    if (computed > 1) PPLOG("simulated %d frames (%d..%d)", computed, start, f);
}

void PuppetPinPlugin::sampleWeights(OFX::Image& img, const pp::Xform& toPix, const pp::Mesh& mesh, double time,
                                    Material& out)
{
    out = Material();
    const bool alphaOnly = img.getPixelComponents() == OFX::ePixelComponentAlpha;
    const int nc = alphaOnly ? 1 : 4;
    const OfxRectI b = img.getBounds();
    const char* base = static_cast<const char*>(img.getPixelData());
    const int rowBytes = img.getRowBytes();
    const int channel = choiceAt(weightChannel_, time);  // 0 luminance, 1 alpha
    const bool invert = weightInvert_->getValueAtTime(time);
    auto texel = [&](int x, int y) -> float {
        if (x < b.x1 || x >= b.x2 || y < b.y1 || y >= b.y2) return 0.0f;
        const float* p = reinterpret_cast<const float*>(base + (ptrdiff_t)(y - b.y1) * rowBytes) + (ptrdiff_t)(x - b.x1) * nc;
        if (alphaOnly) return p[0];
        return channel == 1 ? p[3] : 0.2126f * p[0] + 0.7152f * p[1] + 0.0722f * p[2];
    };
    std::vector<float> value(mesh.verts.size());
    for (size_t v = 0; v < mesh.verts.size(); ++v) {
        const pp::Vec2 q = toPix.apply(mesh.verts[v]);
        const double fx = q.x - 0.5, fy = q.y - 0.5;
        const int ix = (int)std::floor(fx), iy = (int)std::floor(fy);
        const float tx = (float)(fx - ix), ty = (float)(fy - iy);
        float w = (texel(ix, iy) * (1 - tx) + texel(ix + 1, iy) * tx) * (1 - ty) +
                  (texel(ix, iy + 1) * (1 - tx) + texel(ix + 1, iy + 1) * tx) * ty;
        w = std::min(1.0f, std::max(0.0f, w));
        value[v] = invert ? 1.0f - w : w;
    }
    if (DebugLog::get().on()) {
        int white = 0, grey = 0;
        for (float v : value) {
            white += v >= 0.99f;
            grey += v > 0.0f && v < 0.99f;
        }
        PPLOG("weight map: %zu vertices, %d white, %d grey", value.size(), white, grey);
    }
    if (choiceAt(weightMode_, time) == 0) {
        out.anchor = value;
    } else {
        out.stiffness.resize(mesh.tris.size());
        for (size_t t = 0; t < mesh.tris.size(); ++t) {
            const pp::Tri& tr = mesh.tris[t];
            const float a = (value[tr[0]] + value[tr[1]] + value[tr[2]]) / 3.0f;
            out.stiffness[t] = 1.0f + 30.0f * a;
        }
    }
    uint64_t h = 1469598103934665603ull;
    for (float v : value) {
        h ^= (uint32_t)std::lround(v * 1024.0f);
        h *= 1099511628211ull;
    }
    out.hash = mixHash(h, (double)choiceAt(weightMode_, time));
}

// One mesh for all connected image layers: each layer is meshed from its own alpha (the first one
// from the whole frame with "Mesh From: Full Frame"), later layers on top.
std::shared_ptr<const PuppetPinPlugin::MeshEntry> PuppetPinPlugin::meshFor(const std::vector<pp::ImageView>& layers,
                                                                           const std::vector<OfxRectD>& rods,
                                                                           const pp::Xform& toPix, double time)
{
    const bool fullFrame = choiceAt(meshSource_, time) == 1;
    std::vector<std::pair<int, pp::MaskGrid>> masks;
    uint64_t key = 0x9e3779b97f4a7c15ull;
    for (size_t i = 0; i < layers.size() && i < rods.size(); ++i) {
        if (!layers[i].valid()) continue;
        const OfxRectD& rod = rods[i];
        const double fw = rod.x2 - rod.x1, fh = rod.y2 - rod.y1;
        if (!(fw > 0.0 && fh > 0.0) || fw > 1e6 || fh > 1e6) continue;
        const double cell = std::max(1.0, std::max(fw, fh) / 1400.0);
        masks.push_back({(int)i, pp::MaskGrid()});
        pp::buildMaskGrid(layers[i], toPix, rod.x1, rod.y1, fw, fh, cell, fullFrame && i == 0, masks.back().second);
        key = mixHash(mixHash(key, (double)i), (double)pp::hashMask(masks.back().second));
    }
    if (masks.empty()) return nullptr;
    // Mesh From Alpha with a Source that is solid everywhere (e.g. a Merge with a background in
    // front of Puppet Pin): every cell is covered, so the whole frame becomes the puppet.
    bool opaque = false;
    if (!fullFrame && !layers.empty() && layers[0].valid()) {
        const pp::ImageView& s = layers[0];
        const int step = std::max(1, std::max(s.x2 - s.x1, s.y2 - s.y1) / 300);
        const float th = (float)std::max(0.05, threshold_->getValueAtTime(time));
        size_t covered = 0, total = 0;
        for (int y = s.y1; y < s.y2; y += step)
            for (int x = s.x1; x < s.x2; x += step, ++total) covered += s.pixel(x, y)[3] > th;
        opaque = total > 0 && covered * 1000 >= total * 995;
    }
    pp::MeshParams mp;
    mp.targetTriangles = std::max(8, triangles_->getValueAtTime(time));
    mp.expansion = std::max(0.0, expansion_->getValueAtTime(time));
    mp.threshold = threshold_->getValueAtTime(time);
    key = mixHash(key, mp.targetTriangles);
    key = mixHash(key, fullFrame ? 1.0 : 0.0);
    key = mixHash(key, mp.expansion);
    key = mixHash(key, mp.threshold);
    {
        std::lock_guard<std::mutex> lock(cacheMtx_);
        for (auto it = meshCache_.begin(); it != meshCache_.end(); ++it) {
            if ((*it)->key != key) continue;
            auto e = *it;
            meshCache_.erase(it);
            meshCache_.push_front(e);
            lastEntry_ = e;
            return e;
        }
    }
    auto e = std::make_shared<MeshEntry>();
    e->key = key;
    e->opaqueSource = opaque;
    e->mesh = std::make_shared<pp::Mesh>();
    for (const auto& [layer, mask] : masks) {
        pp::Mesh part;
        if (pp::buildMesh(mask, mp, part, nullptr)) pp::appendMesh(*e->mesh, part, layer);
    }
    if (e->mesh->tris.empty()) {
        PPLOG("mesh build: nothing covered");
        std::lock_guard<std::mutex> lock(cacheMtx_);
        lastEntry_.reset();
        return nullptr;
    }
    e->mesh->finalize();
    e->solver = std::make_shared<pp::PuppetSolver>(e->mesh);
    PPLOG("mesh built: %zu verts, %zu tris, %d comps, %zu layers", e->mesh->verts.size(), e->mesh->tris.size(),
          e->mesh->numComps, masks.size());
    std::lock_guard<std::mutex> lock(cacheMtx_);
    meshCache_.push_front(e);
    while (meshCache_.size() > 4) meshCache_.pop_back();
    lastEntry_ = e;
    return e;
}

void PuppetPinPlugin::render(const OFX::RenderArguments& args)
{
    std::unique_ptr<OFX::Image> dst(dstClip_->fetchImage(args.time));
    if (!dst) OFX::throwSuiteStatusException(kOfxStatFailed);
    if (dst->getPixelDepth() != OFX::eBitDepthFloat || dst->getPixelComponents() != OFX::ePixelComponentRGBA)
        OFX::throwSuiteStatusException(kOfxStatErrUnsupported);
    std::unique_ptr<OFX::Image> src((srcClip_ && srcClip_->isConnected()) ? srcClip_->fetchImage(args.time) : nullptr);
    if (src && (src->getPixelDepth() != OFX::eBitDepthFloat || src->getPixelComponents() != OFX::ePixelComponentRGBA))
        OFX::throwSuiteStatusException(kOfxStatErrUnsupported);

    const pp::ImageView dv = viewOf(*dst);
    pp::ImageView sv;
    if (src) sv = viewOf(*src);
    const pp::RectI win{args.renderWindow.x1, args.renderWindow.y1, args.renderWindow.x2, args.renderWindow.y2};
    double par = src ? src->getPixelAspectRatio() : 1.0;
    if (!(par > 0.0)) par = 1.0;
    const pp::Xform toPix{args.renderScale.x / par, args.renderScale.y, 0.0, 0.0};

    const int mode = choiceAt(mode_, args.time);
    const bool burn = burnIn_->getValueAtTime(args.time);
    std::vector<pp::PinInput> pins;
    std::vector<int> slots;
    readPins(args.time, pins, slots, mode == kModeAnimate);

    // Image layers: [0] = Source, then every connected Layer input.
    std::vector<std::unique_ptr<OFX::Image>> layerImgs(kMaxLayers);
    std::vector<pp::ImageView> layers(kMaxLayers);
    std::vector<OfxRectD> rods(kMaxLayers);
    if (src) {
        layers[0] = sv;
        rods[0] = srcClip_->getRegionOfDefinition(args.time);
    }
    int extraLayers = 0;
    for (int i = 1; i < kMaxLayers; ++i) {
        if (!layerClips_[i] || !layerClips_[i]->isConnected()) continue;
        layerImgs[i].reset(layerClips_[i]->fetchImage(args.time));
        if (!layerImgs[i] || layerImgs[i]->getPixelDepth() != OFX::eBitDepthFloat ||
            layerImgs[i]->getPixelComponents() != OFX::ePixelComponentRGBA)
            continue;
        layers[i] = viewOf(*layerImgs[i]);
        rods[i] = layerClips_[i]->getRegionOfDefinition(args.time);
        ++extraLayers;
    }
    std::shared_ptr<const MeshEntry> entry;
    if (src || extraLayers) entry = meshFor(layers, rods, toPix, args.time);
    if (entry) updatePreview(*entry, layers, toPix);
    showOpaqueWarning(entry && entry->opaqueSource);
    std::vector<const pp::ImageView*> layerPtrs(kMaxLayers);
    for (int i = 0; i < kMaxLayers; ++i) layerPtrs[i] = &layers[i];
    const int outMode = choiceAt(outputMode_, args.time);

    PPLOG("render t=%.2f scale %.3f %.3f window %d %d %d %d src bounds %d %d %d %d pins %zu mode %d", args.time,
          args.renderScale.x, args.renderScale.y, win.x1, win.y1, win.x2, win.y2, sv.x1, sv.y1, sv.x2, sv.y2,
          pins.size(), mode);
    if (!pins.empty()) PPLOG("first pin at %.2f %.2f (rest %.2f %.2f)", pins[0].dst.x, pins[0].dst.y, pins[0].src.x, pins[0].src.y);

    Material mat;
    if (entry && weightClip_ && weightClip_->isConnected()) {
        std::unique_ptr<OFX::Image> wimg(weightClip_->fetchImage(args.time));
        if (wimg && wimg->getPixelDepth() == OFX::eBitDepthFloat) sampleWeights(*wimg, toPix, *entry->mesh, args.time, mat);
    }
    {
        std::lock_guard<std::mutex> lock(cacheMtx_);
        lastMaterial_ = mat;
    }

    pp::Deformation def;
    const bool deform = entry && mode == kModeAnimate && !pins.empty();
    if (deform) pose(args.time, *entry, mat, pins, def);
    if (entry && (deform || extraLayers || outMode != kOutComposite)) {
        pp::RenderJob job;
        job.src = &sv;
        job.sources = &layerPtrs;
        job.dst = &dv;
        job.window = win;
        job.mesh = entry->mesh.get();
        job.deformed = deform ? &def.pos : &entry->mesh->verts;
        job.triLayer = deform ? &def.triLayer : nullptr;
        job.toSrc = toPix;
        job.toDst = toPix;
        job.aa = 1 + choiceAt(aa_, args.time);
        job.keepUnmeshed = keepUnmeshed_->getValueAtTime(args.time);
        if (outMode == kOutLayer) job.onlySource = std::max(1, outputLayer_->getValueAtTime(args.time)) - 1;
        std::vector<pp::Vec2> motion;
        if (outMode == kOutVectors) {
            // Motion since the previous frame, in pixels: red = horizontal, green = vertical.
            motion.assign(entry->mesh->verts.size(), pp::Vec2());
            if (deform) {
                std::vector<pp::PinInput> prevPins;
                std::vector<int> prevSlots;
                readPins(args.time - 1.0, prevPins, prevSlots, true);
                pp::Deformation prev;
                if (prevPins.empty())
                    prev.pos = entry->mesh->verts;
                else
                    pose(args.time - 1.0, *entry, mat, prevPins, prev);
                for (size_t v = 0; v < motion.size() && v < prev.pos.size(); ++v) {
                    const pp::Vec2 d = def.pos[v] - prev.pos[v];
                    motion[v] = {d.x * toPix.sx, d.y * toPix.sy};
                }
            }
            job.attribute = &motion;
        }
        if (!(deform && outMode != kOutVectors && mblur_->getValueAtTime(args.time))) {
            pp::renderPuppet(job);
        } else {
            // Motion blur: the image at moments around this frame, averaged. Each vertex moves on a
            // straight line towards where it is one frame before / after.
            const double shutter = std::min(2.0, std::max(0.0, shutter_->getValueAtTime(args.time)));
            const int n = std::min(64, std::max(2, mblurSamples_->getValueAtTime(args.time)));
            auto poseAt = [&](double tt, std::vector<pp::Vec2>& outPos) {
                std::vector<pp::PinInput> pk;
                std::vector<int> sk;
                readPins(tt, pk, sk, true);
                if (pk.empty()) {
                    outPos = entry->mesh->verts;
                    return;
                }
                pp::Deformation d;
                pose(tt, *entry, mat, pk, d);
                outPos = d.pos;
            };
            std::vector<pp::Vec2> prev, next;
            poseAt(args.time - 1.0, prev);
            poseAt(args.time + 1.0, next);
            const pp::RectI w{std::max(win.x1, dv.x1), std::max(win.y1, dv.y1), std::min(win.x2, dv.x2),
                              std::min(win.y2, dv.y2)};
            if (w.x2 > w.x1 && w.y2 > w.y1) {
                const int W = w.x2 - w.x1, H = w.y2 - w.y1;
                std::vector<float> acc((size_t)W * H * 4, 0.0f), buf((size_t)W * H * 4, 0.0f);
                pp::ImageView tv;
                tv.data = buf.data();
                tv.x1 = w.x1;
                tv.y1 = w.y1;
                tv.x2 = w.x2;
                tv.y2 = w.y2;
                tv.rowBytes = (ptrdiff_t)W * 4 * (ptrdiff_t)sizeof(float);
                std::vector<pp::Vec2> at(def.pos.size());
                job.dst = &tv;
                job.window = w;
                job.deformed = &at;
                for (int i = 0; i < n; ++i) {
                    const double sub = shutter * ((i + 0.5) / n - 0.5);
                    const std::vector<pp::Vec2>& other = sub < 0.0 ? prev : next;
                    const double f = std::fabs(sub);
                    for (size_t v = 0; v < at.size(); ++v)
                        at[v] = other.size() == at.size() ? def.pos[v] + (other[v] - def.pos[v]) * f : def.pos[v];
                    pp::renderPuppet(job);
                    for (size_t k = 0; k < acc.size(); ++k) acc[k] += buf[k];
                }
                const float inv = 1.0f / (float)n;
                for (int y = 0; y < H; ++y) {
                    float* o = dv.pixel(w.x1, w.y1 + y);
                    const float* a = &acc[(size_t)y * W * 4];
                    for (int k = 0; k < W * 4; ++k) o[k] = a[k] * inv;
                }
            }
        }
    } else {
        pp::copyImage(sv, dv, win);
    }

    if (burn) {
        std::vector<pp::OverlayPin> marks;
        const int sel = selectedSlot();
        for (size_t k = 0; k < pins.size(); ++k)
            marks.push_back({mode == kModeRest ? pins[k].src : pins[k].dst, slots[k] == sel});
        const std::vector<pp::Vec2>* positions = nullptr;
        if (entry) positions = deform ? &def.pos : &entry->mesh->verts;
        pp::drawBurnIn(dv, win, entry ? entry->mesh.get() : nullptr, positions, marks, toPix,
                       showMesh_->getValue() ? 0.5 : 0.0);
    }
    ++renders_;
    PPLOG("render done");
}

namespace {

OfxRectD unite(const OfxRectD& a, const OfxRectD& b)
{
    return {std::min(a.x1, b.x1), std::min(a.y1, b.y1), std::max(a.x2, b.x2), std::max(a.y2, b.y2)};
}

}  // namespace

void PuppetPinPlugin::showOpaqueWarning(bool on)
{
    if (opaqueWarning_.exchange(on ? 1 : 0) == (on ? 1 : 0)) return;
    try {
        if (on)
            setPersistentMessage(OFX::Message::eMessageWarning, "",
                                 "Puppet Pin: the Source has no see-through background, so the whole frame is the "
                                 "puppet. Connect the cut-out character (with alpha) to Puppet Pin and merge the "
                                 "background AFTER it, or set Mesh From to Full Frame on purpose.");
        else
            clearPersistentMessage();
    } catch (...) {
        // hosts without persistent messages
    }
    PPLOG("opaque source warning %d", (int)on);
}

// The output reaches as far as the posed puppet: a limb moved past the edge of the Source image
// must not be cut off. Fusion (General context) gives the next node exactly this region; the
// Edit / Color page (Filter context) always works on the timeline frame, so it keeps the default.
bool PuppetPinPlugin::getRegionOfDefinition(const OFX::RegionOfDefinitionArguments& args, OfxRectD& rod)
{
    if (getContext() != OFX::eContextGeneral || !srcClip_ || !srcClip_->isConnected()) return false;
    rod = srcClip_->getRegionOfDefinition(args.time);
    for (int i = 1; i < kMaxLayers; ++i)
        if (layerClips_[i] && layerClips_[i]->isConnected()) rod = unite(rod, layerClips_[i]->getRegionOfDefinition(args.time));
    if (!(rod.x2 > rod.x1 && rod.y2 > rod.y1)) return false;
    if (choiceAt(mode_, args.time) != kModeAnimate) return true;
    std::vector<pp::PinInput> pins;
    std::vector<int> slots;
    readPins(args.time, pins, slots, true);
    if (pins.empty()) return true;

    std::shared_ptr<const MeshEntry> entry;
    Material mat;
    {
        std::lock_guard<std::mutex> lock(cacheMtx_);
        entry = lastEntry_;
        mat = lastMaterial_;
    }
    double x1 = rod.x1, y1 = rod.y1, x2 = rod.x2, y2 = rod.y2;
    auto add = [&](const pp::Vec2& p) {
        x1 = std::min(x1, p.x);
        y1 = std::min(y1, p.y);
        x2 = std::max(x2, p.x);
        y2 = std::max(y2, p.y);
    };
    if (entry) {
        // Where the mesh actually goes (with motion blur, also one frame before and after).
        auto posed = [&](double t, const std::vector<pp::PinInput>* known) {
            std::vector<pp::PinInput> pk;
            std::vector<int> sk;
            if (!known) readPins(t, pk, sk, true);
            const std::vector<pp::PinInput>& use = known ? *known : pk;
            if (use.empty()) return;
            pp::Deformation d;
            pose(t, *entry, mat, use, d);
            for (const pp::Vec2& p : d.pos) add(p);
        };
        posed(args.time, &pins);
        if (mblur_->getValueAtTime(args.time)) {
            posed(args.time - 1.0, nullptr);
            posed(args.time + 1.0, nullptr);
        }
        const double m = 4.0 + 2.0 * entry->mesh->edgeLength;  // antialiasing and a changing source
        x1 -= m;
        y1 -= m;
        x2 += m;
        y2 += m;
    } else {
        // No mesh yet (nothing rendered): each pin carries the image along with its own move, turn
        // and scale, so the corners of the Source moved by every pin bound the result.
        const pp::Vec2 corners[4] = {{rod.x1, rod.y1}, {rod.x2, rod.y1}, {rod.x1, rod.y2}, {rod.x2, rod.y2}};
        for (const pp::PinInput& p : pins) {
            const double c = std::cos(p.rotation), s = std::sin(p.rotation), k = p.scale > 0.0 ? p.scale : 1.0;
            for (const pp::Vec2& q : corners) {
                const pp::Vec2 d = q - p.src;
                add({p.dst.x + k * (c * d.x - s * d.y), p.dst.y + k * (s * d.x + c * d.y)});
            }
        }
    }
    // Never more than the Source's size again on every side (a runaway pin must not ask the host
    // for a giant image).
    const double w = rod.x2 - rod.x1, h = rod.y2 - rod.y1, lim = 2.0 * std::max(w, h);
    rod = {std::floor(std::max(x1, rod.x1 - lim)), std::floor(std::max(y1, rod.y1 - lim)),
           std::ceil(std::min(x2, rod.x2 + lim)), std::ceil(std::min(y2, rod.y2 + lim))};
    PPLOG("region of definition t=%.2f: %.0f %.0f %.0f %.0f (mesh %d)", args.time, rod.x1, rod.y1, rod.x2, rod.y2,
          (int)(entry != nullptr));
    return true;
}

// The mesh is built from the whole Source and any output pixel can show any source pixel, so every
// input is needed in full, whatever part of the output is rendered.
void PuppetPinPlugin::getRegionsOfInterest(const OFX::RegionsOfInterestArguments& args, OFX::RegionOfInterestSetter& rois)
{
    if (srcClip_ && srcClip_->isConnected()) rois.setRegionOfInterest(*srcClip_, srcClip_->getRegionOfDefinition(args.time));
    for (int i = 1; i < kMaxLayers; ++i)
        if (layerClips_[i] && layerClips_[i]->isConnected())
            rois.setRegionOfInterest(*layerClips_[i], layerClips_[i]->getRegionOfDefinition(args.time));
    if (weightClip_ && weightClip_->isConnected())
        rois.setRegionOfInterest(*weightClip_, weightClip_->getRegionOfDefinition(args.time));
}

// Smooth/Elastic keep moving after the last keyframe and Wave/Follower move without keyframes, so the
// host must not reuse a frame just because no parameter changed (Fusion does that otherwise).
void PuppetPinPlugin::getClipPreferences(OFX::ClipPreferencesSetter& prefs)
{
    bool varying = choiceAt(solverMode_, 0.0) != kSolveStatic || choiceAt(loopMode_, 0.0) != 0;
    for (auto& d : defs_) {
        const int type = choiceAt(d.type, 0.0);
        varying = varying || type == (int)pp::DeformerType::Follower || type == (int)pp::DeformerType::Wave ||
                  type == (int)pp::DeformerType::Wiggle || type == (int)pp::DeformerType::Wind ||
                  type == (int)pp::DeformerType::Spring;
    }
    prefs.setOutputFrameVarying(varying);
    PPLOG("clip preferences: frame varying %d", (int)varying);
}

bool PuppetPinPlugin::isIdentity(const OFX::IsIdentityArguments& args, OFX::Clip*& identityClip, double& identityTime)
{
    if (burnIn_->getValueAtTime(args.time)) return false;
    if (anyLayerConnected() || choiceAt(outputMode_, args.time) != kOutComposite) return false;
    // The viewer's mesh is built by render(): if the host skipped rendering, changing Triangles or
    // Expansion would not show until the first pin.
    if (showMesh_->getValue()) return false;
    bool identity = choiceAt(mode_, args.time) == kModeRest;
    if (!identity) {
        std::vector<pp::PinInput> pins;
        std::vector<int> slots;
        readPins(args.time, pins, slots);
        identity = pins.empty();
        if (!identity && choiceAt(solverMode_, args.time) != kSolveStatic) return false;
        if (!identity && mblur_->getValueAtTime(args.time)) return false;
        if (!identity && keepUnmeshed_->getValueAtTime(args.time)) {
            // Every pin at rest -> nothing moves.
            identity = true;
            for (const auto& p : pins)
                if (pp::length2(p.dst - p.src) > 1e-12 || std::fabs(p.rotation) > 1e-9 || std::fabs(p.scale - 1.0) > 1e-9)
                    identity = false;
        }
    }
    if (identity) {
        identityClip = srcClip_;
        identityTime = args.time;
    }
    return identity;
}

bool PuppetPinPlugin::anyLayerConnected() const
{
    for (int i = 1; i < kMaxLayers; ++i)
        if (layerClips_[i] && layerClips_[i]->isConnected()) return true;
    return false;
}

void PuppetPinPlugin::updateOutputVisibility()
{
    int m = 0;
    outputMode_->getValue(m);
    const bool show = m == kOutLayer;
    if (outputLayer_->getIsSecret() == show) outputLayer_->setIsSecret(!show);
}

void PuppetPinPlugin::purgeCaches()
{
    std::lock_guard<std::mutex> lock(cacheMtx_);
    meshCache_.clear();
}

void PuppetPinPlugin::overlayState(double time, OverlayState& st)
{
    st.mode = choiceAt(mode_, time);
    readPins(time, st.pins, st.slots, st.mode == kModeAnimate, &st.driven);
    readDeformers(time, st.rigs);
    st.locators.clear();
    for (int k = 0; k < kMaxLocators; ++k) {
        if (!locs_[k].on->getValueAtTime(time)) continue;
        double x, y;
        locs_[k].rest->getValueAtTime(time, x, y);
        st.locators.push_back({k, toCanonical(x, y)});
    }
    {
        std::lock_guard<std::mutex> lock(cacheMtx_);
        st.entry = lastEntry_;
    }
    st.def = pp::Deformation();
    if (!st.entry) return;
    if (st.mode == kModeAnimate && !st.pins.empty()) {
        Material mat;
        {
            std::lock_guard<std::mutex> lock(cacheMtx_);
            mat = lastMaterial_;
        }
        if (!mat.anchor.empty() && mat.anchor.size() != st.entry->mesh->verts.size()) mat = Material();
        if (!mat.stiffness.empty() && mat.stiffness.size() != st.entry->mesh->tris.size()) mat = Material();
        pose(time, *st.entry, mat, st.pins, st.def);
    } else {
        st.def.pos = st.entry->mesh->verts;
        st.def.triLayer.assign(st.entry->mesh->tris.size(), 0);
    }
    // On Fusion's page the picture stays the size of the Source: anything posed past it is cut off.
    if (getContext() == OFX::eContextGeneral && srcClip_ && srcClip_->isConnected() && st.mode == kModeAnimate) {
        const OfxRectD rod = srcClip_->getRegionOfDefinition(time);
        for (const pp::Vec2& p : st.def.pos)
            if (p.x < rod.x1 - 1.0 || p.y < rod.y1 - 1.0 || p.x > rod.x2 + 1.0 || p.y > rod.y2 + 1.0) {
                st.pastEdge = true;
                break;
            }
    }
}

bool PuppetPinPlugin::restPointAt(const OverlayState& st, const pp::Vec2& p, pp::Vec2& rest) const
{
    if (!st.entry) {
        rest = p;
        return true;
    }
    const pp::Mesh& M = *st.entry->mesh;
    const std::vector<pp::Vec2>& D = st.def.pos.size() == M.verts.size() ? st.def.pos : M.verts;
    pp::TriLocator loc;
    loc.build(D, M.tris);
    std::vector<std::pair<int, std::array<double, 3>>> hits;
    loc.locateAll(p, hits);
    int best = -1;
    std::array<double, 3> bary{};
    for (const auto& h : hits) {
        // The render draws higher layers (then higher indices) on top; pick the visible one.
        const int lb = best >= 0 && !st.def.triLayer.empty() ? st.def.triLayer[best] : 0;
        const int lh = !st.def.triLayer.empty() ? st.def.triLayer[h.first] : 0;
        if (best < 0 || lh > lb || (lh == lb && h.first > best)) {
            best = h.first;
            bary = h.second;
        }
    }
    if (best < 0) {
        double b[3];
        best = loc.nearest(p, 0.5 * M.edgeLength, b);
        if (best < 0) return false;
        bary = {b[0], b[1], b[2]};
    }
    const pp::Tri& t = M.tris[best];
    rest = M.verts[t[0]] * bary[0] + M.verts[t[1]] * bary[1] + M.verts[t[2]] * bary[2];
    return true;
}

pp::Vec2 PuppetPinPlugin::rawPin(int slot, double time, bool rest)
{
    time = animTime(time);
    double x, y;
    (rest ? pins_[slot].src : pins_[slot].pos)->getValueAtTime(time, x, y);
    return toCanonical(x, y);
}

void PuppetPinPlugin::updateDeformerVisibility()
{
    auto show = [](OFX::Param* p, bool visible) {
        if (p->getIsSecret() == visible) p->setIsSecret(!visible);
    };
    bool previousUsed = true;  // the first deformer is always offered
    for (int k = 0; k < kMaxDeformers; ++k) {
        DeformerParams& d = defs_[k];
        int type = 0;
        d.type->getValue(type);
        const bool on = previousUsed || type != 0;
        previousUsed = type != 0;
        show(d.group, on);
        show(d.type, on);
        show(d.pins, on && type != 0);
        show(d.amount, on && (type == 1 || type == 7));
        show(d.delay, on && type == 1);
        show(d.bend, on && type == 2);
        show(d.swing, on && type == 2);
        show(d.speed, on && (type == 2 || type == 5 || type == 6));
        show(d.lag, on && (type == 2 || type == 6));
        show(d.phase, on && type == 2);
        show(d.flip, on && type == 3);
        show(d.angle1, on && type == 4);
        show(d.angle2, on && type == 4);
        show(d.angle3, on && type == 4);
        show(d.size, on && type == 5);
        show(d.spin, on && type == 5);
        show(d.direction, on && type == 6);
        show(d.strength, on && type == 6);
        show(d.gust, on && type == 6);
        show(d.seed, on && (type == 5 || type == 6));
        show(d.firm, on && type == 7);
        show(d.damp, on && type == 7);
    }
}

void PuppetPinPlugin::updateSolverVisibility()
{
    int mode = 0, startMode = 0;
    solverMode_->getValue(mode);
    simStartMode_->getValue(startMode);
    auto show = [](OFX::Param* p, bool visible) {
        if (p->getIsSecret() == visible) p->setIsSecret(!visible);
    };
    show(firmness_, mode != kSolveStatic);
    show(damping_, mode == kSolveElastic);
    show(reach_, mode != kSolveStatic);
    show(simStartMode_, mode != kSolveStatic);
    show(simStart_, mode != kSolveStatic && startMode == 1);
}

void PuppetPinPlugin::setSelectedSlot(int slot)
{
    setSelection(slot >= 0 ? std::vector<int>{slot} : std::vector<int>{}, slot);
}

namespace {
std::string slotList(const std::vector<int>& slots)
{
    std::string s;
    for (int v : slots) s += (s.empty() ? "" : ", ") + std::to_string(v + 1);
    return s;
}
}  // namespace

// Selected Pins only counts while it lists the selected pin; otherwise (Selected Pin was changed on
// its own) the selection is just that pin.
std::vector<int> PuppetPinPlugin::selectedSlots()
{
    std::vector<int> out;
    const int active = selectedSlot();
    if (active < 0 || active >= kMaxPins || !pins_[active].on->getValue()) return out;
    out.push_back(active);
    std::string text;
    selection_->getValue(text);
    std::vector<int> listed;
    for (int v : pp::parsePinList(text, kMaxPins)) listed.push_back(v - 1);
    if (std::find(listed.begin(), listed.end(), active) == listed.end()) return out;
    for (int v : listed)
        if (pins_[v].on->getValue() && std::find(out.begin(), out.end(), v) == out.end()) out.push_back(v);
    return out;
}

bool PuppetPinPlugin::isSelected(int slot)
{
    const std::vector<int> sel = selectedSlots();
    return std::find(sel.begin(), sel.end(), slot) != sel.end();
}

void PuppetPinPlugin::setSelection(std::vector<int> slots, int active)
{
    if (active < 0 && !slots.empty()) active = slots[0];
    if (active < 0) slots.clear();
    else if (std::find(slots.begin(), slots.end(), active) == slots.end()) slots.insert(slots.begin(), active);
    const std::string text = slotList(slots);
    std::string old;
    selection_->getValue(old);
    selecting_ = true;
    if (old != text) selection_->setValue(text);
    if (selected_->getValue() != active + 1) selected_->setValue(active + 1);
    selecting_ = false;
}

int PuppetPinPlugin::firstFreeSlot()
{
    for (int i = 0; i < kMaxPins; ++i)
        if (!pins_[i].on->getValue()) return i;
    return -1;
}

void PuppetPinPlugin::clearPin(int slot)
{
    touched();
    PinParams& p = pins_[slot];
    p.on->setValue(false);
    p.pos->deleteAllKeys();
    p.pos->setValue(0.5, 0.5);
    p.src->deleteAllKeys();
    p.src->setValue(0.5, 0.5);
    p.radius->deleteAllKeys();
    p.radius->setValue(0.0);
    p.rotation->deleteAllKeys();
    p.rotation->setValue(0.0);
    p.scale->deleteAllKeys();
    p.scale->setValue(1.0);
    p.strength->deleteAllKeys();
    p.strength->setValue(1.0);
    p.layer->deleteAllKeys();
    p.layer->setValue(0);
    p.influence->deleteAllKeys();
    p.influence->setValue(0.0);
    p.falloff->deleteAllKeys();
    p.falloff->setValue(0.5);
}

// Deletes the selected pins (Remove Selected Pins, or Delete in the viewer). False if none is selected.
bool PuppetPinPlugin::removeSelectedPin()
{
    const std::vector<int> sel = selectedSlots();
    if (sel.empty()) return false;
    beginEditBlock(sel.size() > 1 ? "Remove Pins" : "Remove Pin");
    for (int slot : sel) clearPin(slot);
    setSelectedSlot(-1);
    endEditBlock();
    updatePinVisibility();
    PPLOG("%s removed", slotList(sel).c_str());
    return true;
}

void PuppetPinPlugin::createPin(int slot, const pp::Vec2& rest, const pp::Vec2& pos, double time)
{
    touched();
    time = animTime(time);
    clearPin(slot);
    PinParams& p = pins_[slot];
    const pp::Vec2 rn = toNormalised(rest), pn = toNormalised(pos);
    p.src->setValue(rn.x, rn.y);
    if (choiceAt(keyMode_, 0.0) == 1 && choiceAt(mode_, time) == kModeAnimate)
        p.pos->setValueAtTime(time, pn.x, pn.y);
    else
        p.pos->setValue(pn.x, pn.y);
    p.on->setValue(true);
    updatePinVisibility();
    PPLOG("pin %d created rest %.4f %.4f pos %.4f %.4f t=%.2f", slot + 1, rn.x, rn.y, pn.x, pn.y, time);
}

void PuppetPinPlugin::movePin(int slot, const pp::Vec2& pos, double time)
{
    touched();
    time = animTime(time);
    PinParams& p = pins_[slot];
    const pp::Vec2 pn = toNormalised(pos);
    if (keyOnDrag(*p.pos))
        p.pos->setValueAtTime(time, pn.x, pn.y);
    else
        p.pos->setValue(pn.x, pn.y);
}

void PuppetPinPlugin::setRotation(int slot, double degrees, double time)
{
    touched();
    time = animTime(time);
    OFX::DoubleParam* p = pins_[slot].rotation;
    if (keyOnDrag(*p))
        p->setValueAtTime(time, degrees);
    else
        p->setValue(degrees);
}

void PuppetPinPlugin::setScale(int slot, double scale, double time)
{
    touched();
    time = animTime(time);
    scale = std::min(100.0, std::max(0.01, scale));
    OFX::DoubleParam* p = pins_[slot].scale;
    if (keyOnDrag(*p))
        p->setValueAtTime(time, scale);
    else
        p->setValue(scale);
}

void PuppetPinPlugin::setRadius(int slot, double radius, double time)
{
    touched();
    time = animTime(time);
    OFX::DoubleParam* p = pins_[slot].radius;
    if (animated(*p))
        p->setValueAtTime(time, radius);
    else
        p->setValue(radius);
}

// Scale and Rotation act on the area inside the radius: a pin without one gets a starting radius
// (about three mesh triangles).
void PuppetPinPlugin::ensureRadius(int slot, double time)
{
    if (pins_[slot].radius->getValueAtTime(animTime(time)) > 0.0) return;
    double r = 30.0;
    {
        std::lock_guard<std::mutex> lock(cacheMtx_);
        if (lastEntry_ && lastEntry_->mesh->edgeLength > 0.0) r = std::max(15.0, 3.0 * lastEntry_->mesh->edgeLength);
    }
    setRadius(slot, r, time);
    PPLOG("pin %d got a starting radius of %.1f", slot + 1, r);
}

void PuppetPinPlugin::setInfluence(int slot, double influence, double time)
{
    touched();
    time = animTime(time);
    OFX::DoubleParam* p = pins_[slot].influence;
    if (animated(*p))
        p->setValueAtTime(time, influence);
    else
        p->setValue(influence);
}

void PuppetPinPlugin::moveRest(int slot, const pp::Vec2& rest, double time)
{
    touched();
    time = animTime(time);
    PinParams& p = pins_[slot];
    double ox, oy, px, py;
    p.src->getValueAtTime(time, ox, oy);
    const pp::Vec2 rn = toNormalised(rest);
    if (p.src->getNumKeys() > 0)
        p.src->setValueAtTime(time, rn.x, rn.y);
    else
        p.src->setValue(rn.x, rn.y);
    // Un-animated pins keep their offset, so they do not jump.
    if (p.pos->getNumKeys() == 0) {
        p.pos->getValue(px, py);
        p.pos->setValue(px + rn.x - ox, py + rn.y - oy);
    }
}

void PuppetPinPlugin::updateLocatorVisibility()
{
    for (LocatorParams& l : locs_) {
        const bool hide = !l.on->getValue();
        if (l.group->getIsSecret() == hide) continue;
        for (OFX::Param* q : std::initializer_list<OFX::Param*>{l.group, l.on, l.rest, l.pos, l.rot}) q->setIsSecret(hide);
    }
}

void PuppetPinPlugin::setLocatorRest(int slot, const pp::Vec2& rest, double time)
{
    touched();
    const pp::Vec2 n = toNormalised(rest);
    OFX::Double2DParam* p = locs_[slot].rest;
    if (p->getNumKeys() > 0)
        p->setValueAtTime(time, n.x, n.y);
    else
        p->setValue(n.x, n.y);
}

// ---- Pose tools ----------------------------------------------------------------------------

namespace {
// A copied pose (Copy Pose), shared by every Puppet Pin node: the pins' own values (normalised).
struct PoseClip {
    std::vector<int> slots;
    std::vector<pp::Vec2> pos;
    std::vector<double> rotation, scale;
};
PoseClip& poseClipboard()
{
    static PoseClip clip;
    return clip;
}
}  // namespace

// A pin's position / rotation / scale, keyframed like a drag would ("Keyframes When Dragging").
void PuppetPinPlugin::setPose(int slot, const pp::Vec2& n, double rotation, double scale, double time)
{
    touched();
    const double t = animTime(time);
    PinParams& p = pins_[slot];
    if (keyOnDrag(*p.pos))
        p.pos->setValueAtTime(t, n.x, n.y);
    else
        p.pos->setValue(n.x, n.y);
    if (std::fabs(p.rotation->getValueAtTime(t) - rotation) > 1e-9 || animated(*p.rotation)) setRotation(slot, rotation, time);
    if (std::fabs(p.scale->getValueAtTime(t) - scale) > 1e-9 || animated(*p.scale)) setScale(slot, scale, time);
}

// Key All Pins: a position keyframe on every pin at this frame (rotation / scale too where used).
void PuppetPinPlugin::keyAllPins(double time)
{
    touched();
    const double t = animTime(time);
    beginEditBlock("Key All Pins");
    for (int i = 0; i < kMaxPins; ++i) {
        PinParams& p = pins_[i];
        if (!p.on->getValueAtTime(t)) continue;
        double x, y;
        p.pos->getValueAtTime(t, x, y);
        p.pos->setValueAtTime(t, x, y);
        const double r = p.rotation->getValueAtTime(t), sc = p.scale->getValueAtTime(t);
        if (animated(*p.rotation) || std::fabs(r) > 1e-9) p.rotation->setValueAtTime(t, r);
        if (animated(*p.scale) || std::fabs(sc - 1.0) > 1e-9) p.scale->setValueAtTime(t, sc);
    }
    endEditBlock();
}

void PuppetPinPlugin::copyPose(double time)
{
    const double t = animTime(time);
    PoseClip clip;
    for (int i = 0; i < kMaxPins; ++i) {
        PinParams& p = pins_[i];
        if (!p.on->getValueAtTime(t)) continue;
        double x, y;
        p.pos->getValueAtTime(t, x, y);
        clip.slots.push_back(i);
        clip.pos.push_back({x, y});
        clip.rotation.push_back(p.rotation->getValueAtTime(t));
        clip.scale.push_back(p.scale->getValueAtTime(t));
    }
    poseClipboard() = clip;
    PPLOG("pose copied: %zu pins", clip.slots.size());
}

// Paste Pose: every copied pin, or only the selected ones when several are selected.
void PuppetPinPlugin::pastePose(double time)
{
    const PoseClip clip = poseClipboard();
    if (clip.slots.empty()) {
        sendMessage(OFX::Message::eMessageMessage, "", "Copy a pose first (Copy Pose).");
        return;
    }
    const std::vector<int> sel = selectedSlots();
    beginEditBlock("Paste Pose");
    for (size_t k = 0; k < clip.slots.size(); ++k) {
        const int slot = clip.slots[k];
        if (!pins_[slot].on->getValueAtTime(animTime(time))) continue;
        if (sel.size() > 1 && std::find(sel.begin(), sel.end(), slot) == sel.end()) continue;
        setPose(slot, clip.pos[k], clip.rotation[k], clip.scale[k], time);
    }
    endEditBlock();
}

// Mirror Pose: each selected pin's pose (its move away from rest, mirrored left / right) goes onto
// its partner on the other side of the body - the pin whose rest point mirrors its own. A pin in the
// middle mirrors itself. Nothing selected: the whole pose flips.
void PuppetPinPlugin::mirrorPose(double time)
{
    const double t = animTime(time);
    std::vector<int> slots;
    std::vector<pp::Vec2> rest, pos;
    std::vector<double> rot, sc;
    for (int i = 0; i < kMaxPins; ++i) {
        PinParams& p = pins_[i];
        if (!p.on->getValueAtTime(t)) continue;
        double x, y;
        p.src->getValueAtTime(t, x, y);
        rest.push_back(toCanonical(x, y));
        p.pos->getValueAtTime(t, x, y);
        pos.push_back(toCanonical(x, y));
        rot.push_back(p.rotation->getValueAtTime(t));
        sc.push_back(p.scale->getValueAtTime(t));
        slots.push_back(i);
    }
    if (slots.empty()) return;
    double x0 = 1e300, x1 = -1e300;
    for (const pp::Vec2& r : rest) {
        x0 = std::min(x0, r.x);
        x1 = std::max(x1, r.x);
    }
    const double axis = 0.5 * (x0 + x1);
    const double tol = std::max(1e-6, 0.12 * (x1 - x0));
    auto partnerOf = [&](size_t i) {
        const pp::Vec2 m(2.0 * axis - rest[i].x, rest[i].y);
        size_t best = i;
        double bestD = tol;
        for (size_t j = 0; j < rest.size(); ++j) {
            const double d = pp::length(rest[j] - m);
            if (d < bestD) {
                bestD = d;
                best = j;
            }
        }
        return best;
    };
    std::vector<int> sel = selectedSlots();
    if (sel.empty()) sel = slots;
    beginEditBlock("Mirror Pose");
    for (size_t i = 0; i < slots.size(); ++i) {
        if (std::find(sel.begin(), sel.end(), slots[i]) == sel.end()) continue;
        const size_t j = partnerOf(i);
        const pp::Vec2 move = pos[i] - rest[i];
        setPose(slots[j], toNormalised(rest[j] + pp::Vec2(-move.x, move.y)), -rot[i], sc[i], time);
    }
    endEditBlock();
}

// ---- Rig files ------------------------------------------------------------------------------

namespace {
// What a rig file holds: tag (b bool, i int, c choice, d double, p 2D point, s string) and name.
std::vector<std::pair<char, std::string>> rigPinParams(int i)
{
    const std::string n = pinParam(i, "");
    return {{'b', n + "on"},     {'p', n + "rest"},     {'p', n + "pos"},      {'d', n + "radius"},
            {'d', n + "rotation"}, {'d', n + "scale"},  {'d', n + "strength"}, {'d', n + "influence"},
            {'d', n + "falloff"}, {'i', n + "layer"}};
}
std::vector<std::pair<char, std::string>> rigDeformerParams(int k)
{
    const std::string n = "def" + std::to_string(k + 1) + "_";
    std::vector<std::pair<char, std::string>> out = {{'c', n + "type"}, {'s', n + "pins"}, {'b', n + "flip"}};
    for (const char* d : {"amount", "delay", "bend", "swing", "speed", "lag", "phase", "angle1", "angle2", "angle3",
                          "size", "spin", "direction", "strength", "gust", "seed", "firm", "damp"})
        out.push_back({'d', n + d});
    return out;
}
const std::vector<std::pair<char, std::string>>& rigGlobalParams()
{
    static const std::vector<std::pair<char, std::string>> g = {
        {'c', "meshSource"}, {'i', "triangles"}, {'d', "expansion"},  {'d', "threshold"},  {'d', "rigidity"},
        {'i', "iterations"}, {'c', "solverMode"}, {'d', "firmness"}, {'d', "damping"}, {'d', "jiggleReach"}};
    return g;
}
std::filesystem::path pathOf(const std::string& utf8) { return std::filesystem::u8path(utf8); }
}  // namespace

// The Rig File, or Documents\PuppetPin Rigs\Rig.pprig when it is empty.
std::string PuppetPinPlugin::rigPath()
{
    std::string path;
    rigFile_->getValue(path);
    while (!path.empty() && (path.back() == ' ' || path.back() == '"')) path.pop_back();
    while (!path.empty() && (path.front() == ' ' || path.front() == '"')) path.erase(path.begin());
    if (!path.empty()) return path;
    const wchar_t* home = _wgetenv(L"USERPROFILE");
    std::filesystem::path base = home ? std::filesystem::path(home) / L"Documents" : std::filesystem::path(L".");
    return (base / L"PuppetPin Rigs" / L"Rig.pprig").u8string();
}

// Save Rig: pins (rest point, current position and settings), deformers, mesh and solver settings.
void PuppetPinPlugin::saveRig(double time)
{
    const double t = animTime(time);
    std::ostringstream out;
    out.precision(10);
    out << "PuppetPinRig 1\n";
    auto put = [&](char tag, const std::string& name) {
        out << tag << ' ' << name << ' ';
        switch (tag) {
            case 'b': out << (fetchBooleanParam(name)->getValueAtTime(t) ? 1 : 0); break;
            case 'i': out << fetchIntParam(name)->getValueAtTime(t); break;
            case 'c': {
                int v = 0;
                fetchChoiceParam(name)->getValueAtTime(t, v);
                out << v;
                break;
            }
            case 'd': out << fetchDoubleParam(name)->getValueAtTime(t); break;
            case 'p': {
                double x, y;
                fetchDouble2DParam(name)->getValueAtTime(t, x, y);
                out << x << ' ' << y;
                break;
            }
            case 's': {
                std::string v;
                fetchStringParam(name)->getValueAtTime(t, v);
                for (char& ch : v)
                    if (ch == '\n' || ch == '\r') ch = ' ';
                out << v;
                break;
            }
        }
        out << '\n';
    };
    int pinsSaved = 0, defsSaved = 0;
    for (const auto& [tag, name] : rigGlobalParams()) put(tag, name);
    for (int i = 0; i < kMaxPins; ++i) {
        if (!pins_[i].on->getValueAtTime(t)) continue;
        for (const auto& [tag, name] : rigPinParams(i)) put(tag, name);
        ++pinsSaved;
    }
    for (int k = 0; k < kMaxDeformers; ++k) {
        if (choiceAt(defs_[k].type, t) == 0) continue;
        for (const auto& [tag, name] : rigDeformerParams(k)) put(tag, name);
        ++defsSaved;
    }
    const std::string path = rigPath();
    std::error_code ec;
    const std::filesystem::path fp = pathOf(path);
    if (fp.has_parent_path()) std::filesystem::create_directories(fp.parent_path(), ec);
    std::ofstream f(fp, std::ios::binary | std::ios::trunc);
    f << out.str();
    f.close();
    if (!f) {
        sendMessage(OFX::Message::eMessageError, "", "Could not write the rig file:\n" + path);
        return;
    }
    std::string shown;
    rigFile_->getValue(shown);
    if (shown.empty()) rigFile_->setValue(path);
    PPLOG("rig saved: %d pins, %d deformers -> %s", pinsSaved, defsSaved, path.c_str());
    sendMessage(OFX::Message::eMessageMessage, "",
                "Rig saved (" + std::to_string(pinsSaved) + " pins, " + std::to_string(defsSaved) + " deformers):\n" +
                    path);
}

// Load Rig: replaces every pin and deformer with the file's (no keyframes).
void PuppetPinPlugin::loadRig()
{
    const std::string path = rigPath();
    std::ifstream f(pathOf(path), std::ios::binary);
    std::string line;
    if (!f || !std::getline(f, line) || line.compare(0, 14, "PuppetPinRig 1") != 0) {
        sendMessage(OFX::Message::eMessageError, "", "Not a Puppet Pin rig file:\n" + path);
        return;
    }
    std::map<std::string, char> known;
    for (const auto& e : rigGlobalParams()) known[e.second] = e.first;
    for (int i = 0; i < kMaxPins; ++i)
        for (const auto& e : rigPinParams(i)) known[e.second] = e.first;
    for (int k = 0; k < kMaxDeformers; ++k)
        for (const auto& e : rigDeformerParams(k)) known[e.second] = e.first;

    beginEditBlock("Load Rig");
    for (int i = 0; i < kMaxPins; ++i) clearPin(i);
    for (int k = 0; k < kMaxDeformers; ++k) {
        defs_[k].type->deleteAllKeys();
        defs_[k].type->setValue(0);
        defs_[k].pins->setValue("");
    }
    int pinsLoaded = 0;
    while (std::getline(f, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.size() < 4 || line[1] != ' ') continue;
        const char tag = line[0];
        const size_t sp = line.find(' ', 2);
        const std::string name = line.substr(2, sp == std::string::npos ? std::string::npos : sp - 2);
        const std::string rest = sp == std::string::npos ? std::string() : line.substr(sp + 1);
        const auto it = known.find(name);
        if (it == known.end() || it->second != tag) continue;
        std::istringstream in(rest);
        try {
            switch (tag) {
                case 'b': {
                    int v = 0;
                    in >> v;
                    auto* q = fetchBooleanParam(name);
                    q->deleteAllKeys();
                    q->setValue(v != 0);
                    if (v && name.compare(name.size() - 3, 3, "_on") == 0) ++pinsLoaded;
                    break;
                }
                case 'i': {
                    int v = 0;
                    in >> v;
                    auto* q = fetchIntParam(name);
                    q->deleteAllKeys();
                    q->setValue(v);
                    break;
                }
                case 'c': {
                    int v = 0;
                    in >> v;
                    auto* q = fetchChoiceParam(name);
                    q->deleteAllKeys();
                    q->setValue(v);
                    break;
                }
                case 'd': {
                    double v = 0.0;
                    in >> v;
                    auto* q = fetchDoubleParam(name);
                    q->deleteAllKeys();
                    q->setValue(v);
                    break;
                }
                case 'p': {
                    double x = 0.5, y = 0.5;
                    in >> x >> y;
                    auto* q = fetchDouble2DParam(name);
                    q->deleteAllKeys();
                    q->setValue(x, y);
                    break;
                }
                case 's': fetchStringParam(name)->setValue(rest); break;
            }
        } catch (...) {
            PPLOG("rig load: %s failed", name.c_str());
        }
    }
    setSelectedSlot(-1);
    endEditBlock();
    touched();
    updatePinVisibility();
    updateDeformerVisibility();
    updateSolverVisibility();
    PPLOG("rig loaded: %d pins from %s", pinsLoaded, path.c_str());
}

// ---- Viewer helpers ---------------------------------------------------------------------------

// Onion skin: the posed mesh some frames before and after `time`, and the mesh outline.
void PuppetPinPlugin::onionGhosts(double time, bool stale, std::vector<Ghost>& ghosts,
                                  std::vector<std::pair<int, int>>& outline)
{
    ghosts.clear();
    outline.clear();
    std::shared_ptr<const MeshEntry> entry;
    Material mat;
    {
        std::lock_guard<std::mutex> lock(cacheMtx_);
        entry = lastEntry_;
        mat = lastMaterial_;
    }
    if (!entry) return;
    if (!mat.anchor.empty() && mat.anchor.size() != entry->mesh->verts.size()) mat = Material();
    if (!mat.stiffness.empty() && mat.stiffness.size() != entry->mesh->tris.size()) mat = Material();
    const int before = std::min(10, std::max(0, onionBefore_->getValue()));
    const int after = std::min(10, std::max(0, onionAfter_->getValue()));
    const int step = std::min(100, std::max(1, onionStep_->getValue()));
    const int frame = (int)std::floor(time + 0.5);
    std::lock_guard<std::mutex> lock(helperMtx_);
    GhostCache& c = ghostCache_;
    if (c.entry == entry.get() && c.frame == frame && c.before == before && c.after == after && c.step == step &&
        (stale || c.gen == paramGen_.load())) {
        ghosts = c.ghosts;
        outline = c.outline;
        return;
    }
    const pp::Mesh& M = *entry->mesh;
    std::vector<uint64_t> edges;
    edges.reserve(M.tris.size() * 3);
    for (const pp::Tri& tr : M.tris)
        for (int k = 0; k < 3; ++k) {
            const uint32_t a = tr[k], b = tr[(k + 1) % 3];
            edges.push_back(a < b ? ((uint64_t)a << 32 | b) : ((uint64_t)b << 32 | a));
        }
    std::sort(edges.begin(), edges.end());
    for (size_t i = 0; i < edges.size();) {
        size_t j = i + 1;
        while (j < edges.size() && edges[j] == edges[i]) ++j;
        if (j - i == 1) outline.push_back({(int)(edges[i] >> 32), (int)(uint32_t)edges[i]});
        i = j;
    }
    PinMemo memo;
    for (int k = -before; k <= after; ++k) {
        if (k == 0) continue;
        const double t2 = frame + (double)k * step;
        std::vector<pp::PinInput> pk;
        std::vector<int> sk;
        readPins(t2, pk, sk, true, nullptr, &memo);
        Ghost g;
        g.offset = k;
        if (pk.empty()) {
            g.pos = M.verts;
        } else {
            pp::Deformation d;
            pose(t2, *entry, mat, pk, d);
            g.pos = std::move(d.pos);
        }
        ghosts.push_back(std::move(g));
    }
    c.gen = paramGen_.load();
    c.entry = entry.get();
    c.frame = frame;
    c.before = before;
    c.after = after;
    c.step = step;
    c.ghosts = ghosts;
    c.outline = outline;
}

// Motion trails: where the given pins are on the frames around `time` (after deformers).
void PuppetPinPlugin::pinTrails(double time, const std::vector<int>& slots, bool stale, std::vector<Trail>& out)
{
    out.clear();
    const int frames = std::min(240, std::max(1, trailFrames_->getValue()));
    const int frame = (int)std::floor(time + 0.5);
    std::lock_guard<std::mutex> lock(helperMtx_);
    TrailCache& c = trailCache_;
    if (c.frame == frame && c.frames == frames && c.slots == slots && (stale || c.gen == paramGen_.load())) {
        out = c.trails;
        return;
    }
    const int first = frame - frames, n = 2 * frames + 1;
    for (int slot : slots) {
        Trail tr;
        tr.slot = slot;
        tr.first = first;
        tr.pts.assign(n, pp::Vec2());
        tr.on.assign(n, 0);
        tr.keyed.assign(n, 0);
        std::vector<double> keys;
        const int nk = (int)pins_[slot].pos->getNumKeys();
        for (int i = 0; i < nk; ++i) keys.push_back(pins_[slot].pos->getKeyTime(i));
        for (int i = 0; i < n; ++i) {
            const double at = animTime(first + i);
            for (double kt : keys)
                if (std::fabs(kt - at) < 1e-6) tr.keyed[i] = 1;
        }
        out.push_back(std::move(tr));
    }
    PinMemo memo;
    std::vector<pp::PinInput> pk;
    std::vector<int> sk;
    for (int i = 0; i < n; ++i) {
        readPins(first + i, pk, sk, true, nullptr, &memo);
        for (Trail& tr : out)
            for (size_t k = 0; k < sk.size(); ++k)
                if (sk[k] == tr.slot) {
                    tr.pts[i] = pk[k].dst;
                    tr.on[i] = 1;
                }
    }
    c.gen = paramGen_.load();
    c.frame = frame;
    c.frames = frames;
    c.slots = slots;
    c.trails = out;
}

void PuppetPinPlugin::updateHelperVisibility()
{
    auto show = [](OFX::Param* q, bool visible) {
        if (q->getIsSecret() == visible) q->setIsSecret(!visible);
    };
    const bool onion = onion_->getValue(), trails = trails_->getValue(), blur = mblur_->getValue();
    show(onionBefore_, onion);
    show(onionAfter_, onion);
    show(onionStep_, onion);
    show(trailFrames_, trails);
    show(shutter_, blur);
    show(mblurSamples_, blur);
}

// Writes every locator's position and rotation as keyframes, for each frame of the clip.
void PuppetPinPlugin::bakeLocators()
{
    std::shared_ptr<const MeshEntry> e;
    Material mat;
    {
        std::lock_guard<std::mutex> lock(cacheMtx_);
        e = lastEntry_;
        mat = lastMaterial_;
    }
    if (!e) {
        sendMessage(OFX::Message::eMessageWarning, "", "Show a frame of the clip first, so the mesh exists.");
        return;
    }
    if (!mat.anchor.empty() && mat.anchor.size() != e->mesh->verts.size()) mat = Material();
    if (!mat.stiffness.empty() && mat.stiffness.size() != e->mesh->tris.size()) mat = Material();
    std::vector<int> active;
    for (int k = 0; k < kMaxLocators; ++k)
        if (locs_[k].on->getValue()) active.push_back(k);
    if (active.empty()) {
        sendMessage(OFX::Message::eMessageWarning, "", "Add a locator first.");
        return;
    }
    const OfxRangeD range = srcClip_->getFrameRange();
    const int f0 = (int)std::floor(range.min + 0.5);
    const int f1 = std::min(f0 + 20000, (int)std::floor(range.max + 0.5));
    beginEditBlock("Bake Locators");
    for (int k : active) {
        locs_[k].pos->deleteAllKeys();
        locs_[k].rot->deleteAllKeys();
    }
    std::vector<pp::PinInput> pins;
    std::vector<int> slots;
    for (int f = f0; f <= f1; ++f) {
        readPins(f, pins, slots, true);
        pp::Deformation def;
        if (pins.empty())
            def.pos = e->mesh->verts;
        else
            pose(f, *e, mat, pins, def);
        for (int k : active) {
            double x, y;
            locs_[k].rest->getValueAtTime(f, x, y);
            pp::Vec2 at;
            double rot = 0.0;
            if (!trackPoint(*e->mesh, def.pos, toCanonical(x, y), at, rot)) at = toCanonical(x, y);
            const pp::Vec2 n = toNormalised(at);
            locs_[k].pos->setValueAtTime(f, n.x, n.y);
            locs_[k].rot->setValueAtTime(f, rot / kDegToRad);
        }
    }
    endEditBlock();
    PPLOG("baked %zu locators over frames %d..%d", active.size(), f0, f1);
}

void PuppetPinPlugin::updatePinVisibility()
{
    for (int i = 0; i < kMaxPins; ++i) {
        const bool hide = !pins_[i].on->getValue();
        PinParams& p = pins_[i];
        if (p.group->getIsSecret() == hide) continue;
        p.group->setIsSecret(hide);
        p.on->setIsSecret(hide);
        p.pos->setIsSecret(hide);
        p.src->setIsSecret(hide);
        p.radius->setIsSecret(hide);
        p.rotation->setIsSecret(hide);
        p.scale->setIsSecret(hide);
        p.strength->setIsSecret(hide);
        p.layer->setIsSecret(hide);
        p.influence->setIsSecret(hide);
        p.falloff->setIsSecret(hide);
    }
}

void PuppetPinPlugin::changedParam(const OFX::InstanceChangedArgs& args, const std::string& name)
{
    if (args.reason != OFX::eChangeTime) touched();
    // A pin whose settings change becomes the selected pin, so its handles show in the viewer. This
    // also covers animated pins, whose points Fusion drags with its own path handles: those clicks
    // never reach the overlay.
    if (args.reason != OFX::eChangeTime && name.size() > 5 && name.compare(0, 3, "pin") == 0 &&
        std::isdigit((unsigned char)name[3])) {
        const size_t us = name.find('_');
        const std::string what = us == std::string::npos ? std::string() : name.substr(us + 1);
        const int slot = std::atoi(name.c_str() + 3) - 1;
        if ((what == "pos" || what == "rest" || what == "radius" || what == "rotation" || what == "scale" ||
             what == "influence" || what == "falloff" || what == "strength") &&
            slot >= 0 && slot < kMaxPins && pins_[slot].on->getValue() && !isSelected(slot)) {
            PPLOG("select pin %d (%s changed, reason %d)", slot + 1, name.c_str(), (int)args.reason);
            setSelectedSlot(slot);
        }
        // Scale and Rotation act on the area inside the radius: a pin without one gets a starting
        // radius (about three mesh triangles), so the change shows right away.
        if (args.reason == OFX::eChangeUserEdit && (what == "scale" || what == "rotation") && slot >= 0 &&
            slot < kMaxPins && pins_[slot].on->getValue()) {
            PinParams& p = pins_[slot];
            const double t = animTime(args.time);
            const bool changed = std::fabs(p.scale->getValueAtTime(t) - 1.0) > 1e-9 ||
                                 std::fabs(p.rotation->getValueAtTime(t)) > 1e-9;
            if (changed) ensureRadius(slot, args.time);
        }
    }
    // Selected Pin / Selected Pins typed in the Inspector: keep the two in step.
    if ((name == "selectedPin" || name == "selectedPins") && args.reason != OFX::eChangeTime && !selecting_) {
        const int active = selectedSlot();
        std::string text;
        selection_->getValue(text);
        std::vector<int> listed;
        for (int v : pp::parsePinList(text, kMaxPins))
            if (pins_[v - 1].on->getValue()) listed.push_back(v - 1);
        if (std::find(listed.begin(), listed.end(), active) == listed.end()) {
            if (name == "selectedPin")
                setSelectedSlot(active);
            else
                setSelection(listed, listed.empty() ? -1 : listed[0]);
        }
        redrawOverlays();
    }
    if (name == "selectedPin" && DebugLog::get().on()) {
        for (int i = 0; i < kMaxPins; ++i)
            if (pins_[i].on->getValue())
                PPLOG("pin %d: position keys %u animating %d, rotation keys %u animating %d", i + 1,
                      pins_[i].pos->getNumKeys(), (int)pins_[i].pos->getIsAnimating(), pins_[i].rotation->getNumKeys(),
                      (int)pins_[i].rotation->getIsAnimating());
    }
    if (name == "addPin") {
        const int slot = firstFreeSlot();
        if (slot < 0) {
            sendMessage(OFX::Message::eMessageWarning, "", "All 48 pins are in use.");
            return;
        }
        const pp::Vec2 c = toCanonical(0.5, 0.5);
        beginEditBlock("Add Pin");
        createPin(slot, c, c, args.time);
        setSelectedSlot(slot);
        endEditBlock();
    } else if (name == "removePin") {
        if (!removeSelectedPin())
            sendMessage(OFX::Message::eMessageMessage, "", "Select a pin first (click it in the viewer).");
    } else if (name == "removeAll") {
        beginEditBlock("Remove All Pins");
        for (int i = 0; i < kMaxPins; ++i)
            if (pins_[i].on->getValue() || pins_[i].pos->getNumKeys() > 0) clearPin(i);
        setSelectedSlot(-1);
        endEditBlock();
        updatePinVisibility();
    } else if (name == "resetPin") {
        const std::vector<int> sel = selectedSlots();
        if (sel.empty()) return;
        beginEditBlock(sel.size() > 1 ? "Reset Pins" : "Reset Pin");
        for (int slot : sel) {
            PinParams& p = pins_[slot];
            double x, y;
            p.src->getValueAtTime(args.time, x, y);
            p.pos->deleteAllKeys();
            p.pos->setValue(x, y);
            p.rotation->deleteAllKeys();
            p.rotation->setValue(0.0);
            p.scale->deleteAllKeys();
            p.scale->setValue(1.0);
        }
        endEditBlock();
    } else if (name == "addLocator") {
        int slot = -1;
        for (int k = 0; k < kMaxLocators && slot < 0; ++k)
            if (!locs_[k].on->getValue()) slot = k;
        if (slot < 0) {
            sendMessage(OFX::Message::eMessageWarning, "", "All 8 locators are in use.");
            return;
        }
        // On the selected pin's point of the image, or in the middle.
        double x = 0.5, y = 0.5;
        const int sel = selectedSlot();
        if (sel >= 0 && sel < kMaxPins && pins_[sel].on->getValue()) pins_[sel].src->getValueAtTime(args.time, x, y);
        beginEditBlock("Add Locator");
        LocatorParams& l = locs_[slot];
        l.rest->deleteAllKeys();
        l.rest->setValue(x, y);
        l.pos->deleteAllKeys();
        l.pos->setValue(x, y);
        l.rot->deleteAllKeys();
        l.rot->setValue(0.0);
        l.on->setValue(true);
        endEditBlock();
        updateLocatorVisibility();
    } else if (name == "bakeLocators") {
        bakeLocators();
    } else if (name.size() > 3 && name.compare(name.size() - 3, 3, "_on") == 0) {
        updatePinVisibility();
        updateLocatorVisibility();
    } else if (name == "solverMode" || name == "simStartMode") {
        updateSolverVisibility();
    } else if (name == "keyAllPins") {
        keyAllPins(args.time);
    } else if (name == "copyPose") {
        copyPose(args.time);
    } else if (name == "pastePose") {
        pastePose(args.time);
    } else if (name == "mirrorPose") {
        mirrorPose(args.time);
    } else if (name == "saveRig") {
        saveRig(args.time);
    } else if (name == "loadRig") {
        loadRig();
    } else if (name == "onionSkin" || name == "motionTrails" || name == "motionBlur") {
        updateHelperVisibility();
    } else if (name == "onionBefore" || name == "onionAfter" || name == "onionStep" || name == "trailFrames") {
        // only the viewer changes
    } else if (name == "outputMode") {
        updateOutputVisibility();
    } else if (name == "loopMode") {
        updateLoopVisibility();
    } else if (name.size() > 5 && name.compare(0, 3, "def") == 0 &&
               (name.compare(name.size() - 5, 5, "_type") == 0 || name.compare(name.size() - 5, 5, "_pins") == 0)) {
        updateDeformerVisibility();
    } else {
        return;
    }
    redrawOverlays();
}

// =======================================================================================
// Viewer overlay: draws mesh and pins, adds and drags pins.

class PuppetInteract : public OFX::OverlayInteract {
public:
    PuppetInteract(OfxInteractHandle handle, OFX::ImageEffect* effect)
        : OFX::OverlayInteract(handle), fx_(dynamic_cast<PuppetPinPlugin*>(effect))
    {
    }

    bool draw(const OFX::DrawArgs& args) override;
    bool penDown(const OFX::PenArgs& args) override;
    bool penMotion(const OFX::PenArgs& args) override;
    bool penUp(const OFX::PenArgs& args) override;
    bool keyDown(const OFX::KeyArgs& args) override;
    bool keyUp(const OFX::KeyArgs& args) override;
    void loseFocus(const OFX::FocusArgs& args) override;
    ~PuppetInteract() override { closeBlock(); }

private:
    bool keyTaken_ = false;  // the last key press was ours (its release is ours too)
    // E / S: rotate / scale the selected pins around the selected pin by moving the mouse (or dragging).
    int keyTurn_ = -1;              // the pivot: the selected pin
    double keyTurnLast_ = 0.0;      // E: pen angle at the last motion
    bool keyTurnRef_ = false;       // keyTurnLast_ / keyRefDist_ is set
    bool keyTurnMoved_ = false;     // the pins changed since E / S
    pp::Vec2 keyTurnCentre_;        // where the pivot is drawn
    bool keyScaling_ = false;       // S (scale) instead of E (rotate)
    double keyRefDist_ = 1.0;       // S: pen distance from the pivot that means "unchanged"
    bool keyDrag_ = false;          // E / S followed by a drag: releasing finishes
    int keyRenders_ = -1;           // renders when the pins last changed (the preview shows until the next)
    struct XPin {
        int slot;
        pp::Vec2 pos;  // before E / S
        double rotation, scale;
        bool radius;   // its own area turns / scales too
    };
    std::vector<XPin> xform_;
    pp::Vec2 xformPivot_;
    double xformAmount_ = 0.0;      // degrees (E) or factor (S)
    void applyXform(double amount, double time);
    void restoreXform(double time);
    void endKeyTurn();
    // Every change made in the viewer is one undo step: an edit block from the first change of a drag
    // (or E / S) to its end.
    bool blockOpen_ = false;
    void openBlock(const char* name);
    void closeBlock();
    bool shift_ = false;            // Shift or Ctrl held: clicks add pins to the selection or take them out
    bool boxing_ = false, boxMoved_ = false;  // a box selection from an empty spot
    pp::Vec2 boxStart_, boxEnd_;
    void selectBox(double time);
    std::vector<std::pair<int, pp::Vec2>> dragGroup_;  // the other selected pins moving with drag_
    PuppetPinPlugin* fx_ = nullptr;
    int drag_ = -1;
    bool dragRest_ = false;
    int dragReach_ = -1;                    // pin whose influence circle is being resized
    pp::Vec2 reachCentre_;
    int dragTurn_ = -1;                     // pin being turned with its rotation knob
    int dragRadius_ = -1;                   // pin whose radius is being dragged
    double turnLast_ = 0.0, turnDegrees_ = 0.0, radiusScale_ = 1.0;
    int dragLoc_ = -1;                      // locator being placed
    bool dragging() const
    {
        return drag_ >= 0 || dragReach_ >= 0 || dragTurn_ >= 0 || dragRadius_ >= 0 || dragLoc_ >= 0 || boxing_ ||
               keyDrag_;
    }
    pp::Vec2 dragStartPen_, dragStartRaw_;  // drags move the pin's own value by the pen's motion
    int moves_ = 0;                         // pen motions in the current drag (debug log)
    // After a drag the preview stays up until the host has rendered the new frame (a short while).
    bool holdPreview_ = false;
    int holdUntilRender_ = 0;
    std::chrono::steady_clock::time_point releasedAt_;
    bool haveClick_ = false;                // last click on empty image, for double-click detection
    std::chrono::steady_clock::time_point clickTime_;
    pp::Vec2 clickPos_;
};

namespace {

struct Painter {
    OfxDrawSuiteV1* s;
    OfxDrawContextHandle ctx;
    void colour(float r, float g, float b, float a)
    {
        const OfxRGBAColourF c = {r, g, b, a};
        s->setColour(ctx, &c);
    }
    void width(float w) { s->setLineWidth(ctx, w); }
    void stipple(OfxDrawLineStipplePattern p) { s->setLineStipple(ctx, p); }
    void prim(OfxDrawPrimitive p, const std::vector<OfxPointD>& pts)
    {
        if (!pts.empty()) s->draw(ctx, p, pts.data(), (int)pts.size());
    }
    void text(const std::string& t, double x, double y, int align)
    {
        const OfxPointD p = {x, y};
        s->drawText(ctx, t.c_str(), &p, align);
    }
    std::vector<OfxPointD> circle(const pp::Vec2& c, double r, int n = 20)
    {
        std::vector<OfxPointD> pts(n);
        for (int i = 0; i < n; ++i) {
            const double a = 6.283185307179586 * i / n;
            pts[i] = {c.x + r * std::cos(a), c.y + r * std::sin(a)};
        }
        return pts;
    }
};

}  // namespace

namespace {
// Where the influence circle's resize handle sits (upper right of the circle).
pp::Vec2 reachHandle(const pp::Vec2& c, double r)
{
    return c + pp::Vec2(r, r) * 0.7071067811865476;
}
// The selected pin's radius circle: the rotation knob sits at the pin's rotation, the radius handle at
// the bottom; a pin without a radius gets a small radius handle just below it.
pp::Vec2 turnHandle(const pp::Vec2& c, double r, double rotation)
{
    return c + pp::Vec2(std::cos(rotation), std::sin(rotation)) * r;
}
pp::Vec2 radiusHandle(const pp::Vec2& c, double r, double ps)
{
    return c - pp::Vec2(0.0, r > 0.0 ? r : 18.0 * ps);
}
}  // namespace

bool PuppetInteract::draw(const OFX::DrawArgs& args)
{
    OfxDrawSuiteV1* suite = OFX::Private::gDrawSuite;
    if (!fx_ || !suite || !args.context) return false;
    Painter pt{suite, args.context};
    const double ps = std::max(1e-9, args.pixelScale.x);

    PuppetPinPlugin::OverlayState st;
    fx_->overlayState(args.time, st);
    const bool rest = st.mode == kModeRest;

    if (st.pastEdge) {
        const pp::Vec2 at = fx_->toCanonical(0.02, 0.10);
        const char* l1 = "Puppet Pin: part of the puppet is past the edge of its picture and gets cut off.";
        const char* l2 = "Right-click this node > Scripts > Puppet Pin - Add Room (one click, keeps the pose).";
        for (int k = 0; k < 2; ++k) {
            const double o = k == 0 ? 1.5 * ps : 0.0;
            if (k == 0)
                pt.colour(0.0f, 0.0f, 0.0f, 0.9f);
            else
                pt.colour(1.0f, 0.45f, 0.3f, 1.0f);
            pt.text(l1, at.x + o, at.y - o, 0);
            pt.text(l2, at.x + o, at.y - 18.0 * ps - o, 0);
        }
    }

    // Resolve shows no plugin status messages, so a Source without a see-through background (a
    // background merged in BEFORE Puppet Pin) is pointed out in the viewer. Never in the render.
    if (st.entry && st.entry->opaqueSource) {
        const pp::Vec2 at = fx_->toCanonical(0.02, 0.95);
        const char* l1 = "Puppet Pin: the Source has no transparent background, so the WHOLE frame is the puppet.";
        const char* l2 = "Put Puppet Pin on the cut-out character and Merge the background AFTER it.";
        for (int k = 0; k < 2; ++k) {
            const double o = k == 0 ? 1.5 * ps : 0.0;
            if (k == 0)
                pt.colour(0.0f, 0.0f, 0.0f, 0.9f);
            else
                pt.colour(1.0f, 0.82f, 0.15f, 1.0f);
            pt.text(l1, at.x + o, at.y - o, 0);
            pt.text(l2, at.x + o, at.y - 18.0 * ps - o, 0);
        }
    }

    // While a pin is dragged the host shows the frame from before the drag, so the overlay draws a
    // live preview over it: the image as flat-coloured triangles, posed by the current pins.
    // (After E / S without a drag the host may re-render by itself: then the preview steps aside.)
    bool preview = (drag_ >= 0 && !dragRest_) || dragReach_ >= 0 || dragTurn_ >= 0 || dragRadius_ >= 0 || keyDrag_ ||
                   (keyTurn_ >= 0 && keyTurnMoved_ && fx_->renderCount() == keyRenders_);
    if (!preview && holdPreview_) {
        holdPreview_ = fx_->renderCount() == holdUntilRender_ &&
                       std::chrono::steady_clock::now() - releasedAt_ < std::chrono::milliseconds(1500);
        preview = holdPreview_;
    }
    if (preview && !rest && st.entry && st.def.pos.size() == st.entry->mesh->verts.size()) {
        const pp::Mesh& M = *st.entry->mesh;
        std::vector<float> col;
        {
            std::lock_guard<std::mutex> lock(st.entry->previewMtx);
            col = st.entry->preview;
        }
        const auto& subs = subTriangles(previewLevel(M.tris.size()));
        if (col.size() == M.tris.size() * subs.size() * 4) {
            double x0 = 1e300, y0 = 1e300, x1 = -1e300, y1 = -1e300;
            for (const std::vector<pp::Vec2>* set : {&M.verts, (const std::vector<pp::Vec2>*)&st.def.pos})
                for (const pp::Vec2& v : *set) {
                    x0 = std::min(x0, v.x); x1 = std::max(x1, v.x);
                    y0 = std::min(y0, v.y); y1 = std::max(y1, v.y);
                }
            pt.colour(0.16f, 0.16f, 0.17f, 1.0f);  // hides the frame from before the drag
            pt.prim(kOfxDrawPrimitiveRectangle, {{x0 - 2.0 * ps, y0 - 2.0 * ps}, {x1 + 2.0 * ps, y1 + 2.0 * ps}});
            std::vector<int> order(M.tris.size());
            for (size_t t = 0; t < order.size(); ++t) order[t] = (int)t;
            if (st.def.triLayer.size() == M.tris.size())
                std::stable_sort(order.begin(), order.end(),
                                 [&](int a, int b) { return st.def.triLayer[a] < st.def.triLayer[b]; });
            for (int t : order) {
                const pp::Tri& tr = M.tris[t];
                const pp::Vec2 a = st.def.pos[tr[0]], b = st.def.pos[tr[1]], c = st.def.pos[tr[2]];
                for (size_t k = 0; k < subs.size(); ++k) {
                    const float* rgba = &col[(t * subs.size() + k) * 4];
                    if (rgba[3] < 0.02f) continue;
                    std::vector<OfxPointD> poly(3);
                    for (int j = 0; j < 3; ++j) {
                        const pp::Vec2 q = a * subs[k][j][0] + b * subs[k][j][1] + c * subs[k][j][2];
                        poly[j] = {q.x, q.y};
                    }
                    pt.colour(rgba[0], rgba[1], rgba[2], rgba[3]);
                    pt.prim(kOfxDrawPrimitivePolygon, poly);
                }
            }
        }
    }

    // Onion skin: outlines of the pose some frames before (orange) and after (blue).
    if (!rest && fx_->onionOn()) {
        std::vector<PuppetPinPlugin::Ghost> ghosts;
        std::vector<std::pair<int, int>> outline;
        fx_->onionGhosts(args.time, dragging() || keyTurn_ >= 0, ghosts, outline);
        int span = 1;
        for (const auto& g : ghosts) span = std::max(span, std::abs(g.offset));
        pt.stipple(kOfxDrawLineStipplePatternSolid);
        pt.width(1.5f);
        for (const auto& g : ghosts) {
            std::vector<OfxPointD> lines;
            lines.reserve(outline.size() * 2);
            for (const auto& [a, b] : outline) {
                if (a >= (int)g.pos.size() || b >= (int)g.pos.size()) continue;
                lines.push_back({g.pos[a].x, g.pos[a].y});
                lines.push_back({g.pos[b].x, g.pos[b].y});
            }
            const float alpha = 0.85f - 0.5f * (float)(std::abs(g.offset) - 1) / (float)span;
            if (g.offset < 0)
                pt.colour(1.0f, 0.55f, 0.25f, alpha);
            else
                pt.colour(0.35f, 0.75f, 1.0f, alpha);
            pt.prim(kOfxDrawPrimitiveLines, lines);
        }
    }

    if (fx_->showMesh() && st.entry && st.def.pos.size() == st.entry->mesh->verts.size()) {
        const pp::Mesh& M = *st.entry->mesh;
        std::vector<uint64_t> keys;
        keys.reserve(M.tris.size() * 3);
        for (const pp::Tri& t : M.tris)
            for (int k = 0; k < 3; ++k) {
                const uint32_t a = t[k], b = t[(k + 1) % 3];
                keys.push_back(a < b ? ((uint64_t)a << 32 | b) : ((uint64_t)b << 32 | a));
            }
        std::sort(keys.begin(), keys.end());
        keys.erase(std::unique(keys.begin(), keys.end()), keys.end());
        std::vector<OfxPointD> lines;
        lines.reserve(keys.size() * 2);
        for (uint64_t k : keys) {
            const pp::Vec2& a = st.def.pos[(uint32_t)(k >> 32)];
            const pp::Vec2& b = st.def.pos[(uint32_t)k];
            lines.push_back({a.x, a.y});
            lines.push_back({b.x, b.y});
        }
        pt.stipple(kOfxDrawLineStipplePatternSolid);
        pt.width(0.0f);
        if (rest)
            pt.colour(0.35f, 0.85f, 1.0f, 0.35f);
        else
            pt.colour(1.0f, 0.85f, 0.25f, 0.35f);
        pt.prim(kOfxDrawPrimitiveLines, lines);
    }

    if (!fx_->showPins()) return true;

    // Deformer rigs: the bones of their chains, dashed lines from a leader to its followers.
    if (!rest && !st.rigs.empty()) {
        std::vector<pp::Vec2> posOf(kMaxPins + 1), restOf(kMaxPins + 1);
        std::vector<char> has(kMaxPins + 1, 0);
        for (size_t k = 0; k < st.pins.size(); ++k) {
            posOf[st.slots[k] + 1] = st.pins[k].dst;
            restOf[st.slots[k] + 1] = st.pins[k].src;
            has[st.slots[k] + 1] = 1;
        }
        for (const pp::Deformer& d : st.rigs) {
            std::vector<OfxPointD> pts;
            std::vector<pp::Vec2> rests;
            for (int s : d.pins)
                if (has[s] == 1) {
                    has[s] = 2;  // each pin once
                    pts.push_back({posOf[s].x, posOf[s].y});
                    rests.push_back(restOf[s]);
                }
            for (int s : d.pins)
                if (has[s]) has[s] = 1;
            if (pts.size() < 2 || d.type == pp::DeformerType::Wiggle) continue;
            if (d.type == pp::DeformerType::Wave || d.type == pp::DeformerType::Wind ||
                d.type == pp::DeformerType::FK) {
                // A chain may branch: each pin hangs off the closest pin listed before it.
                const std::vector<int> parent = pp::chainParents(rests);
                std::vector<OfxPointD> bones;
                for (size_t i = 1; i < pts.size(); ++i) {
                    bones.push_back(pts[parent[i]]);
                    bones.push_back(pts[i]);
                }
                pt.stipple(kOfxDrawLineStipplePatternSolid);
                pt.width(3.0f);
                pt.colour(0.0f, 0.0f, 0.0f, 0.5f);
                pt.prim(kOfxDrawPrimitiveLines, bones);
                pt.width(1.5f);
                pt.colour(0.3f, 0.95f, 1.0f, 0.9f);
                pt.prim(kOfxDrawPrimitiveLines, bones);
            } else if (d.type == pp::DeformerType::Follower || d.type == pp::DeformerType::Spring) {
                std::vector<OfxPointD> lines;
                for (size_t i = 1; i < pts.size(); ++i) {
                    lines.push_back(pts[0]);
                    lines.push_back(pts[i]);
                }
                pt.stipple(kOfxDrawLineStipplePatternDash);
                pt.width(1.5f);
                pt.colour(1.0f, 0.45f, 0.9f, 0.85f);
                pt.prim(kOfxDrawPrimitiveLines, lines);
            } else {
                pt.stipple(kOfxDrawLineStipplePatternSolid);
                pt.width(3.0f);
                pt.colour(0.0f, 0.0f, 0.0f, 0.5f);
                pt.prim(kOfxDrawPrimitiveLineStrip, pts);
                pt.width(1.5f);
                pt.colour(0.3f, 0.95f, 1.0f, 0.9f);
                pt.prim(kOfxDrawPrimitiveLineStrip, pts);
            }
        }
        pt.stipple(kOfxDrawLineStipplePatternSolid);
    }

    const int sel = fx_->selectedSlot();
    const std::vector<int> selAll = fx_->selectedSlots();
    auto shownAt = [&](int slot, pp::Vec2& q) {
        for (size_t k = 0; k < st.pins.size(); ++k)
            if (st.slots[k] == slot) {
                q = rest ? st.pins[k].src : st.pins[k].dst;
                return true;
            }
        return false;
    };
    // Motion trails of the selected pins: a dot per frame, diamonds on keyframes, earlier frames
    // orange, later ones blue.
    if (!rest && fx_->trailsOn() && !selAll.empty()) {
        std::vector<PuppetPinPlugin::Trail> trails;
        fx_->pinTrails(args.time, selAll, dragging() || keyTurn_ >= 0, trails);
        const int now = (int)std::floor(args.time + 0.5);
        pt.stipple(kOfxDrawLineStipplePatternSolid);
        for (const auto& tr : trails) {
            std::vector<OfxPointD> past, future;
            for (size_t i = 0; i + 1 < tr.pts.size(); ++i) {
                if (!tr.on[i] || !tr.on[i + 1]) continue;
                auto& v = tr.first + (int)i + 1 <= now ? past : future;
                v.push_back({tr.pts[i].x, tr.pts[i].y});
                v.push_back({tr.pts[i + 1].x, tr.pts[i + 1].y});
            }
            pt.width(1.5f);
            pt.colour(1.0f, 0.6f, 0.3f, 0.8f);
            pt.prim(kOfxDrawPrimitiveLines, past);
            pt.colour(0.4f, 0.8f, 1.0f, 0.8f);
            pt.prim(kOfxDrawPrimitiveLines, future);
            for (size_t i = 0; i < tr.pts.size(); ++i) {
                const int f = tr.first + (int)i;
                if (!tr.on[i] || f == now) continue;
                const pp::Vec2 q = tr.pts[i];
                if (tr.keyed[i]) {
                    const double so = 4.5 * ps, si = 3.2 * ps;
                    pt.colour(0.0f, 0.0f, 0.0f, 0.85f);
                    pt.prim(kOfxDrawPrimitivePolygon, {{q.x, q.y - so}, {q.x + so, q.y}, {q.x, q.y + so}, {q.x - so, q.y}});
                    pt.colour(1.0f, 1.0f, 1.0f, 1.0f);
                    pt.prim(kOfxDrawPrimitivePolygon, {{q.x, q.y - si}, {q.x + si, q.y}, {q.x, q.y + si}, {q.x - si, q.y}});
                } else {
                    const double d = 1.6 * ps;
                    if (f < now)
                        pt.colour(1.0f, 0.6f, 0.3f, 1.0f);
                    else
                        pt.colour(0.4f, 0.8f, 1.0f, 1.0f);
                    pt.prim(kOfxDrawPrimitiveRectangle, {{q.x - d, q.y - d}, {q.x + d, q.y + d}});
                }
            }
        }
    }
    if (keyTurn_ >= 0 && !rest) {
        const pp::Vec2 c = keyTurnCentre_;
        pt.stipple(kOfxDrawLineStipplePatternSolid);
        pt.width(2.0f);
        if (keyScaling_)
            pt.colour(1.0f, 0.65f, 0.25f, 0.9f);
        else
            pt.colour(0.35f, 0.8f, 1.0f, 0.9f);
        pt.prim(kOfxDrawPrimitiveLineLoop, pt.circle(c, 16.0 * ps, 32));
        std::vector<OfxPointD> spokes;  // from the pivot to the pins that turn around it
        for (const XPin& x : xform_) {
            pp::Vec2 q;
            if (x.slot == keyTurn_ || !shownAt(x.slot, q)) continue;
            spokes.push_back({c.x, c.y});
            spokes.push_back({q.x, q.y});
        }
        pt.width(1.0f);
        pt.stipple(kOfxDrawLineStipplePatternDot);
        pt.prim(kOfxDrawPrimitiveLines, spokes);
        pt.stipple(kOfxDrawLineStipplePatternSolid);
        const std::string who = xform_.size() > 1 ? std::to_string(xform_.size()) + " pins around pin " +
                                                        std::to_string(keyTurn_ + 1)
                                                  : "pin " + std::to_string(keyTurn_ + 1);
        const pp::Vec2 t = fx_->toCanonical(0.5, 0.97);
        pt.text(keyScaling_ ? "Scale " + who + ": move the mouse away from / towards it (or drag). Click or S = keep, Esc = cancel"
                            : "Rotate " + who + ": move the mouse around it (or drag). Click or E = keep, Esc = cancel",
                t.x, t.y, kOfxDrawTextAlignmentCenterH | kOfxDrawTextAlignmentTop);
    } else if (selAll.size() > 1) {
        const pp::Vec2 t = fx_->toCanonical(0.5, 0.97);
        pt.colour(1.0f, 1.0f, 1.0f, 0.9f);
        pt.text(std::to_string(selAll.size()) + " pins selected: drag one to move them all, E rotates / S scales them "
                    "around pin " + std::to_string(sel + 1) + " (red)",
                t.x, t.y, kOfxDrawTextAlignmentCenterH | kOfxDrawTextAlignmentTop);
    }
    if (boxing_ && boxMoved_) {
        pt.stipple(kOfxDrawLineStipplePatternDash);
        pt.width(1.0f);
        pt.colour(1.0f, 1.0f, 1.0f, 0.9f);
        pt.prim(kOfxDrawPrimitiveLineLoop,
                {{boxStart_.x, boxStart_.y}, {boxEnd_.x, boxStart_.y}, {boxEnd_.x, boxEnd_.y}, {boxStart_.x, boxEnd_.y}});
        pt.stipple(kOfxDrawLineStipplePatternSolid);
    }
    if (st.pins.empty()) {
        const pp::Vec2 c = fx_->toCanonical(0.5, 0.97);
        const int am = fx_->addMode();
        const char* hint = am == kAddDoubleClick   ? "Puppet Pin: double-click on the image to add pins"
                           : am == kAddSingleClick ? "Puppet Pin: click on the image to add pins"
                                                   : "Puppet Pin: use Add Pin to add pins";
        pt.colour(1.0f, 1.0f, 1.0f, 0.9f);
        pt.text(hint, c.x, c.y, kOfxDrawTextAlignmentCenterH | kOfxDrawTextAlignmentTop);
    }
    // Locators (under the pins): pink diamonds where their point of the image is now.
    for (const auto& [slot, restPt] : st.locators) {
        pp::Vec2 q = restPt;
        double rot = 0.0;
        if (!rest && st.entry) trackPoint(*st.entry->mesh, st.def.pos, restPt, q, rot);
        const double so = 6.5 * ps, si = 5.0 * ps;
        pt.stipple(kOfxDrawLineStipplePatternSolid);
        pt.colour(0.0f, 0.0f, 0.0f, 0.85f);
        pt.prim(kOfxDrawPrimitivePolygon, {{q.x, q.y - so}, {q.x + so, q.y}, {q.x, q.y + so}, {q.x - so, q.y}});
        pt.colour(1.0f, 0.35f, 0.85f, 1.0f);
        pt.prim(kOfxDrawPrimitivePolygon, {{q.x, q.y - si}, {q.x + si, q.y}, {q.x, q.y + si}, {q.x - si, q.y}});
        pt.colour(1.0f, 0.6f, 0.92f, 0.95f);
        pt.text("L" + std::to_string(slot + 1), q.x + 8.0 * ps, q.y - 14.0 * ps, kOfxDrawTextAlignmentLeft);
    }
    for (size_t k = 0; k < st.pins.size(); ++k) {
        const pp::PinInput& pin = st.pins[k];
        const pp::Vec2 p = rest ? pin.src : pin.dst;
        const bool selected = st.slots[k] == sel;
        const bool alsoSelected = !selected && std::find(selAll.begin(), selAll.end(), st.slots[k]) != selAll.end();
        const bool offMesh = st.entry && k < st.def.pinTri.size() && st.def.pinTri[k] < 0 && pin.radius <= 0.0;
        const bool driven = !rest && k < st.driven.size() && st.driven[k];

        if (pin.influence > 0.0) {
            // How far the pin pulls: dotted circle, the falloff ring's inner edge fainter, and on the
            // selected pin a handle to resize it.
            pt.stipple(kOfxDrawLineStipplePatternDot);
            pt.width(selected ? 1.5f : 1.0f);
            pt.colour(0.45f, 1.0f, 0.55f, selected ? 0.9f : 0.45f);
            pt.prim(kOfxDrawPrimitiveLineLoop, pt.circle(p, pin.influence, 64));
            if (selected) {
                const double inner = pin.influence * (1.0 - std::min(1.0, std::max(0.0, pin.falloff)));
                if (inner > 2.0 * ps) {
                    pt.colour(0.45f, 1.0f, 0.55f, 0.35f);
                    pt.prim(kOfxDrawPrimitiveLineLoop, pt.circle(p, inner, 64));
                }
                const pp::Vec2 hp = reachHandle(p, pin.influence);
                const double s = 4.5 * ps;
                pt.stipple(kOfxDrawLineStipplePatternSolid);
                pt.colour(0.0f, 0.0f, 0.0f, 0.85f);
                pt.prim(kOfxDrawPrimitiveRectangle, {{hp.x - s - ps, hp.y - s - ps}, {hp.x + s + ps, hp.y + s + ps}});
                pt.colour(0.45f, 1.0f, 0.55f, 1.0f);
                pt.prim(kOfxDrawPrimitiveRectangle, {{hp.x - s, hp.y - s}, {hp.x + s, hp.y + s}});
            }
        }
        if (pin.radius > 0.0) {
            pt.stipple(kOfxDrawLineStipplePatternDash);
            pt.width(1.0f);
            pt.colour(1.0f, 1.0f, 1.0f, selected || alsoSelected ? 0.8f : 0.45f);
            const double r = pin.radius * (rest ? 1.0 : std::max(1e-3, pin.scale));
            pt.prim(kOfxDrawPrimitiveLineLoop, pt.circle(p, r, 48));
            if (!rest) {
                // rotation tick
                const double a = pin.rotation;
                pt.prim(kOfxDrawPrimitiveLines, {{p.x, p.y}, {p.x + r * std::cos(a), p.y + r * std::sin(a)}});
            }
        }
        pt.stipple(kOfxDrawLineStipplePatternSolid);
        if (selected && !driven) {
            const double r = pin.radius * (rest ? 1.0 : std::max(1e-3, pin.scale));
            pt.width(1.5f);
            if (!rest && pin.radius > 0.0) {
                const pp::Vec2 h = turnHandle(p, r, pin.rotation);  // drag to rotate
                pt.colour(0.0f, 0.0f, 0.0f, 0.85f);
                pt.prim(kOfxDrawPrimitivePolygon, pt.circle(h, 5.5 * ps, 16));
                pt.colour(0.35f, 0.8f, 1.0f, 1.0f);
                pt.prim(kOfxDrawPrimitivePolygon, pt.circle(h, 4.5 * ps, 16));
            }
            const pp::Vec2 h = radiusHandle(p, r, ps);  // drag to set the radius
            const double sz = 5.0 * ps;
            const std::vector<OfxPointD> dia = {{h.x, h.y - sz}, {h.x + sz, h.y}, {h.x, h.y + sz}, {h.x - sz, h.y}};
            if (pin.radius > 0.0) {
                const double so = sz + 1.2 * ps;
                pt.colour(0.0f, 0.0f, 0.0f, 0.85f);
                pt.prim(kOfxDrawPrimitivePolygon, {{h.x, h.y - so}, {h.x + so, h.y}, {h.x, h.y + so}, {h.x - so, h.y}});
                pt.colour(1.0f, 1.0f, 1.0f, 1.0f);
                pt.prim(kOfxDrawPrimitivePolygon, dia);
            } else {
                pt.colour(1.0f, 1.0f, 1.0f, 0.9f);
                pt.prim(kOfxDrawPrimitiveLineLoop, dia);
            }
        }
        const double r = (selected || alsoSelected ? 6.5 : 5.0) * ps;
        if (offMesh)
            pt.colour(0.55f, 0.55f, 0.55f, 0.9f);
        else if (driven)
            pt.colour(0.3f, 0.95f, 1.0f, 0.55f);
        else if (selected)
            pt.colour(1.0f, 0.3f, 0.15f, 1.0f);
        else if (alsoSelected)
            pt.colour(1.0f, 0.62f, 0.45f, 1.0f);
        else if (rest)
            pt.colour(0.3f, 0.85f, 1.0f, 1.0f);
        else
            pt.colour(1.0f, 0.85f, 0.1f, 1.0f);
        pt.prim(kOfxDrawPrimitivePolygon, pt.circle(p, r, 16));
        pt.width(1.5f);
        pt.colour(0.0f, 0.0f, 0.0f, 0.85f);
        pt.prim(kOfxDrawPrimitiveLineLoop, pt.circle(p, r + 0.5 * ps, 16));
        pt.colour(1.0f, 1.0f, 1.0f, 0.95f);
        pt.text(std::to_string(st.slots[k] + 1), p.x + 8.0 * ps, p.y + 8.0 * ps, kOfxDrawTextAlignmentLeft);
    }
    return true;
}

void PuppetInteract::openBlock(const char* name)
{
    if (blockOpen_ || !fx_) return;
    fx_->beginEditBlock(name);
    blockOpen_ = true;
}

void PuppetInteract::closeBlock()
{
    if (!blockOpen_) return;
    blockOpen_ = false;
    if (fx_) fx_->endEditBlock();
}

void PuppetInteract::applyXform(double amount, double time)
{
    const double a = amount * kDegToRad, c = std::cos(a), sn = std::sin(a);
    for (const XPin& x : xform_) {
        if (x.slot != keyTurn_) {
            const pp::Vec2 off = x.pos - xformPivot_;
            fx_->movePin(x.slot, xformPivot_ + (keyScaling_ ? off * amount : pp::rotate(off, c, sn)), time);
        }
        if (!x.radius) continue;
        if (keyScaling_)
            fx_->setScale(x.slot, x.scale * amount, time);
        else
            fx_->setRotation(x.slot, x.rotation + amount, time);
    }
}

void PuppetInteract::restoreXform(double time)
{
    for (const XPin& x : xform_) {
        if (x.slot != keyTurn_) fx_->movePin(x.slot, x.pos, time);
        if (!x.radius) continue;
        if (keyScaling_)
            fx_->setScale(x.slot, x.scale, time);
        else
            fx_->setRotation(x.slot, x.rotation, time);
    }
}

void PuppetInteract::endKeyTurn()
{
    if (keyTurnMoved_ && fx_) {  // the preview stays up until the host has rendered the result
        holdPreview_ = true;
        holdUntilRender_ = fx_->renderCount();
        releasedAt_ = std::chrono::steady_clock::now();
    }
    keyTurn_ = -1;
    keyTurnRef_ = keyTurnMoved_ = keyScaling_ = keyDrag_ = false;
    xform_.clear();
    closeBlock();
}

namespace {
// The pivot of a group of pins: the one where it hangs on the rest (closest to a pin outside the
// group, on the original image), e.g. the shoulder of an arm; with every pin selected, the middle one.
int groupPivot(const PuppetPinPlugin::OverlayState& st, const std::vector<int>& group)
{
    std::vector<pp::Vec2> in, out;
    std::vector<int> inSlot;
    for (size_t k = 0; k < st.pins.size(); ++k) {
        if (std::find(group.begin(), group.end(), st.slots[k]) != group.end()) {
            in.push_back(st.pins[k].src);
            inSlot.push_back(st.slots[k]);
        } else {
            out.push_back(st.pins[k].src);
        }
    }
    if (in.empty()) return group.empty() ? -1 : group[0];
    pp::Vec2 mid;
    for (const pp::Vec2& v : in) mid += v / (double)in.size();
    int best = 0;
    double bestD = 1e300;
    for (size_t i = 0; i < in.size(); ++i) {
        double d = out.empty() ? pp::length2(in[i] - mid) : 1e300;
        for (const pp::Vec2& o : out) d = std::min(d, pp::length2(in[i] - o));
        if (d < bestD) {
            bestD = d;
            best = (int)i;
        }
    }
    return inSlot[best];
}
}  // namespace

void PuppetInteract::selectBox(double time)
{
    PuppetPinPlugin::OverlayState st;
    fx_->overlayState(time, st);
    const bool rest = st.mode == kModeRest;
    const double x0 = std::min(boxStart_.x, boxEnd_.x), x1 = std::max(boxStart_.x, boxEnd_.x);
    const double y0 = std::min(boxStart_.y, boxEnd_.y), y1 = std::max(boxStart_.y, boxEnd_.y);
    std::vector<int> sel = shift_ ? fx_->selectedSlots() : std::vector<int>{};
    for (size_t k = 0; k < st.pins.size(); ++k) {
        const pp::Vec2 q = rest ? st.pins[k].src : st.pins[k].dst;
        if (q.x < x0 || q.x > x1 || q.y < y0 || q.y > y1) continue;
        if (std::find(sel.begin(), sel.end(), st.slots[k]) == sel.end()) sel.push_back(st.slots[k]);
    }
    int active = fx_->selectedSlot();
    if (!shift_ || std::find(sel.begin(), sel.end(), active) == sel.end()) active = groupPivot(st, sel);
    fx_->setSelection(sel, sel.empty() ? -1 : active);
    PPLOG("box selected %zu pin(s), pivot %d", sel.size(), active + 1);
}

bool PuppetInteract::penDown(const OFX::PenArgs& args)
{
    if (!fx_) return false;
    if (keyTurn_ < 0) closeBlock();  // a drag whose release never arrived
    PuppetPinPlugin::OverlayState st;
    fx_->overlayState(args.time, st);
    const bool rest = st.mode == kModeRest;
    const pp::Vec2 p(args.penPosition.x, args.penPosition.y);
    const double ps = std::max(1e-9, args.pixelScale.x);
    PPLOG("penDown %.2f %.2f pixelScale %.4f t=%.2f shift %d", p.x, p.y, ps, args.time, (int)shift_);
    moves_ = 0;
    holdPreview_ = false;
    boxing_ = false;
    if (keyTurn_ >= 0) {
        if (keyTurnMoved_) {  // the pins already followed the mouse: this click keeps it
            endKeyTurn();
            requestRedraw();
            return true;
        }
        // Otherwise drag anywhere to turn / scale; releasing finishes.
        if (keyScaling_)
            keyRefDist_ = std::max(pp::length(p - keyTurnCentre_), 10.0 * ps);
        else
            keyTurnLast_ = std::atan2(p.y - keyTurnCentre_.y, p.x - keyTurnCentre_.x);
        keyTurnRef_ = true;
        keyDrag_ = true;
        haveClick_ = false;
        return true;
    }

    // The selected pin's influence handle.
    const int sel = fx_->selectedSlot();
    for (size_t k = 0; k < st.pins.size(); ++k) {
        if (st.slots[k] != sel || st.pins[k].influence <= 0.0) continue;
        const pp::Vec2 c = rest ? st.pins[k].src : st.pins[k].dst;
        if (pp::length(reachHandle(c, st.pins[k].influence) - p) > 8.0 * ps) continue;
        dragReach_ = sel;
        reachCentre_ = c;
        haveClick_ = false;
        return true;
    }
    // ...its rotation knob and radius handle.
    for (size_t k = 0; k < st.pins.size(); ++k) {
        if (st.slots[k] != sel || (k < st.driven.size() && st.driven[k] && !rest)) continue;
        const pp::PinInput& pin = st.pins[k];
        const pp::Vec2 c = rest ? pin.src : pin.dst;
        const double scale = rest ? 1.0 : std::max(1e-3, pin.scale);
        const double r = pin.radius * scale;
        if (!rest && pin.radius > 0.0 && pp::length(turnHandle(c, r, pin.rotation) - p) <= 8.0 * ps) {
            dragTurn_ = sel;
            reachCentre_ = c;
            turnLast_ = std::atan2(p.y - c.y, p.x - c.x);
            turnDegrees_ = fx_->rawRotation(sel, args.time);
            haveClick_ = false;
            return true;
        }
        if (pp::length(radiusHandle(c, r, ps) - p) <= 8.0 * ps) {
            dragRadius_ = sel;
            reachCentre_ = c;
            radiusScale_ = scale;
            haveClick_ = false;
            return true;
        }
    }

    int hit = -1;
    double best = 10.0 * ps;
    pp::Vec2 hitPos;
    for (size_t k = 0; k < st.pins.size(); ++k) {
        const pp::Vec2 q = rest ? st.pins[k].src : st.pins[k].dst;
        const double d = pp::length(q - p);
        if (d <= best) {
            best = d;
            hit = st.slots[k];
            hitPos = q;
        }
    }
    if (hit < 0) {
        // Locators: drag one to the point of the image it should follow.
        for (const auto& [slot, restPt] : st.locators) {
            pp::Vec2 q = restPt;
            double rot = 0.0;
            if (!rest && st.entry) trackPoint(*st.entry->mesh, st.def.pos, restPt, q, rot);
            if (pp::length(q - p) > 8.0 * ps) continue;
            dragLoc_ = slot;
            dragStartPen_ = p;
            dragStartRaw_ = q;
            haveClick_ = false;
            return true;
        }
    }
    if (hit >= 0) {
        haveClick_ = false;
        std::vector<int> sel = fx_->selectedSlots();
        const bool inSel = std::find(sel.begin(), sel.end(), hit) != sel.end();
        if (shift_) {
            // Shift / Ctrl + click: adds the pin to the selection (as the pivot), or takes it out.
            if (inSel) {
                sel.erase(std::find(sel.begin(), sel.end(), hit));
                const int active = fx_->selectedSlot();
                fx_->setSelection(sel, sel.empty() ? -1 : (active == hit ? sel[0] : active));
            } else {
                sel.push_back(hit);
                fx_->setSelection(sel, hit);
            }
            requestRedraw();
            return true;
        }
        // A pin of a multiple selection becomes the pivot and drags them all; any other pin is
        // selected on its own.
        dragGroup_.clear();
        if (inSel && sel.size() > 1) {
            fx_->setSelection(sel, hit);
            for (int s : sel)
                if (s != hit) dragGroup_.push_back({s, fx_->rawPin(s, args.time, rest)});
        } else {
            fx_->setSelectedSlot(hit);
        }
        drag_ = hit;
        dragRest_ = rest;
        dragStartPen_ = p;
        dragStartRaw_ = fx_->rawPin(hit, args.time, rest);
        requestRedraw();
        return true;
    }

    // An empty spot. On the image a double-click (or a click, see "Add Pins By") adds a pin; otherwise
    // a drag selects the pins inside a box and a click deselects.
    bool onMesh = true;
    pp::Vec2 restPt = p;
    if (st.entry) {
        if (rest) {
            double b[3];
            onMesh = st.entry->mesh->locator.locate(p, b) >= 0 ||
                     st.entry->mesh->locator.nearest(p, 0.5 * st.entry->mesh->edgeLength, b) >= 0;
        } else {
            onMesh = fx_->restPointAt(st, p, restPt);
        }
    }
    const int am = fx_->addMode();
    const auto now = std::chrono::steady_clock::now();
    bool add = false;
    if (onMesh && !shift_) {
        if (am == kAddSingleClick)
            add = true;
        else if (am == kAddDoubleClick)
            add = haveClick_ && now - clickTime_ < std::chrono::milliseconds(500) && pp::length(p - clickPos_) < 8.0 * ps;
    }
    if (!add) {
        haveClick_ = onMesh && am == kAddDoubleClick && !shift_;
        clickTime_ = now;
        clickPos_ = p;
        boxing_ = true;
        boxMoved_ = false;
        boxStart_ = boxEnd_ = p;
        return true;
    }
    haveClick_ = false;
    const int slot = fx_->firstFreeSlot();
    if (slot < 0) {
        fx_->sendMessage(OFX::Message::eMessageWarning, "", "All 48 pins are in use.");
        return true;
    }
    openBlock("Add Pin");  // one undo step together with the drag that follows
    fx_->createPin(slot, restPt, p, args.time);
    fx_->setSelectedSlot(slot);
    dragGroup_.clear();
    drag_ = slot;
    dragRest_ = rest;
    dragStartPen_ = p;
    dragStartRaw_ = p;
    requestRedraw();
    return true;
}

bool PuppetInteract::penMotion(const OFX::PenArgs& args)
{
    if (dragging()) PPLOG("penMotion %d %.2f %.2f", ++moves_, args.penPosition.x, args.penPosition.y);
    const pp::Vec2 pen(args.penPosition.x, args.penPosition.y);
    if (!fx_) return false;
    if (boxing_) {
        boxEnd_ = pen;
        if (pp::length(boxEnd_ - boxStart_) > 4.0 * std::max(1e-9, args.pixelScale.x)) boxMoved_ = true;
        requestRedraw();
        return true;
    }
    if (keyTurn_ >= 0) {
        // After E / S the pins follow the mouse around / away from the pivot.
        bool changed = false;
        if (keyScaling_) {
            const double d = pp::length(pen - keyTurnCentre_);
            if (!keyTurnRef_) {
                keyRefDist_ = std::max(d, 1e-6);
                keyTurnRef_ = true;
            } else {
                xformAmount_ = std::min(100.0, std::max(0.01, d / keyRefDist_));
                changed = true;
            }
        } else {
            const double a = std::atan2(pen.y - keyTurnCentre_.y, pen.x - keyTurnCentre_.x);
            if (keyTurnRef_) {
                double d = a - keyTurnLast_;
                while (d > 3.14159265358979323846) d -= 2.0 * 3.14159265358979323846;
                while (d < -3.14159265358979323846) d += 2.0 * 3.14159265358979323846;
                if (d != 0.0) {
                    xformAmount_ += d / kDegToRad;
                    changed = true;
                }
            }
            keyTurnLast_ = a;
            keyTurnRef_ = true;
        }
        if (changed) {
            openBlock(keyScaling_ ? (xform_.size() > 1 ? "Scale Pins" : "Scale Pin")
                                  : (xform_.size() > 1 ? "Rotate Pins" : "Rotate Pin"));
            applyXform(xformAmount_, args.time);
            keyTurnMoved_ = true;
            keyRenders_ = fx_->renderCount();
        }
        requestRedraw();
        return true;
    }
    if (dragTurn_ >= 0) {
        // Follow the pen's angle around the pin, counting whole turns.
        const double a = std::atan2(pen.y - reachCentre_.y, pen.x - reachCentre_.x);
        double d = a - turnLast_;
        while (d > 3.14159265358979323846) d -= 2.0 * 3.14159265358979323846;
        while (d < -3.14159265358979323846) d += 2.0 * 3.14159265358979323846;
        turnLast_ = a;
        turnDegrees_ += d / kDegToRad;
        openBlock("Rotate Pin");
        fx_->setRotation(dragTurn_, turnDegrees_, args.time);
        requestRedraw();
        return true;
    }
    if (dragLoc_ >= 0) {
        PuppetPinPlugin::OverlayState st;
        fx_->overlayState(args.time, st);
        const pp::Vec2 at = dragStartRaw_ + (pen - dragStartPen_);
        pp::Vec2 restPt = at;
        if (st.mode == kModeRest || fx_->restPointAt(st, at, restPt)) {
            openBlock("Move Locator");
            fx_->setLocatorRest(dragLoc_, restPt, args.time);
        }
        requestRedraw();
        return true;
    }
    if (dragRadius_ >= 0) {
        // Dragged back onto the pin, the radius goes away.
        const double ps = std::max(1e-9, args.pixelScale.x);
        const double r = pp::length(pen - reachCentre_);
        openBlock("Pin Radius");
        fx_->setRadius(dragRadius_, r < 4.0 * ps ? 0.0 : r / radiusScale_, args.time);
        requestRedraw();
        return true;
    }
    if (dragReach_ >= 0) {
        const double r = pp::length(pen - reachCentre_);
        openBlock("Pin Influence");
        fx_->setInfluence(dragReach_, std::max(1.0, r), args.time);  // the handle sits on the circle
        requestRedraw();
        return true;
    }
    if (drag_ < 0) return false;
    const pp::Vec2 delta = pen - dragStartPen_;
    openBlock(dragRest_ ? "Move Rest Position" : (dragGroup_.empty() ? "Move Pin" : "Move Pins"));
    if (dragRest_) {
        fx_->moveRest(drag_, dragStartRaw_ + delta, args.time);
        for (const auto& [slot, from] : dragGroup_) fx_->moveRest(slot, from + delta, args.time);
    } else {
        fx_->movePin(drag_, dragStartRaw_ + delta, args.time);
        for (const auto& [slot, from] : dragGroup_) fx_->movePin(slot, from + delta, args.time);
    }
    requestRedraw();
    return true;
}

bool PuppetInteract::penUp(const OFX::PenArgs& args)
{
    if (!fx_) return false;
    if (boxing_) {
        boxing_ = false;
        if (boxMoved_)
            selectBox(args.time);
        else if (!shift_)
            fx_->setSelectedSlot(-1);  // a click on an empty spot deselects
        requestRedraw();
        return true;
    }
    if (keyDrag_) {
        endKeyTurn();
        requestRedraw();
        return true;
    }
    if (!dragging()) return false;
    const bool posed = (drag_ >= 0 && !dragRest_) || dragReach_ >= 0 || dragTurn_ >= 0 || dragRadius_ >= 0;
    dragLoc_ = -1;
    drag_ = -1;
    dragReach_ = -1;
    dragTurn_ = -1;
    dragRadius_ = -1;
    dragGroup_.clear();
    closeBlock();
    PPLOG("penUp after %d moves", moves_);
    holdPreview_ = moves_ > 0 && posed;
    holdUntilRender_ = fx_->renderCount();
    releasedAt_ = std::chrono::steady_clock::now();
    requestRedraw();
    return true;
}

namespace {
bool isDeleteKey(int sym)
{
    return sym == kOfxKey_Delete || sym == kOfxKey_BackSpace || sym == kOfxKey_KP_Delete;
}
bool isModifier(int sym)
{
    return sym == kOfxKey_Shift_L || sym == kOfxKey_Shift_R || sym == kOfxKey_Control_L || sym == kOfxKey_Control_R;
}
}  // namespace

// Delete / Backspace in the viewer removes the selected pins; E / S rotate / scale them. Without a
// selected pin the keys are left to the host (e.g. deleting nodes).
bool PuppetInteract::keyDown(const OFX::KeyArgs& args)
{
    const int sym = args.keySymbol;
    PPLOG("keyDown sym 0x%x", sym);
    keyTaken_ = false;
    if (isModifier(sym)) {
        shift_ = true;
        return false;  // the host may want it too
    }
    if (!fx_ || dragging()) return false;
    if (keyTurn_ >= 0) {
        // While turning / scaling: Esc puts the pins back, E, S or Enter keeps it. Any other key keeps
        // it too and goes to the host (e.g. Ctrl+Z, which then undoes the turn).
        if (sym == kOfxKey_Escape) {
            if (keyTurnMoved_) restoreXform(args.time);
            keyTurnMoved_ = false;
            endKeyTurn();
        } else if (sym == kOfxKey_e || sym == kOfxKey_E || sym == kOfxKey_s || sym == kOfxKey_S || sym == kOfxKey_Return ||
                   sym == kOfxKey_KP_Enter) {
            endKeyTurn();
        } else {
            endKeyTurn();
            requestRedraw();
            return false;
        }
        keyTaken_ = true;
        requestRedraw();
        return true;
    }
    if (isDeleteKey(sym)) {
        if (!fx_->removeSelectedPin()) return false;
        keyTaken_ = true;
        requestRedraw();
        return true;
    }
    const bool scaleKey = sym == kOfxKey_s || sym == kOfxKey_S;
    if (sym == kOfxKey_e || sym == kOfxKey_E || scaleKey) {
        // E / S with a selected pin: turn / scale the selection around it by moving the mouse (no pin:
        // the key is the host's). R and W are Resolve's Loader / Saver shortcuts in the Fusion page, so
        // rotating is on E.
        PuppetPinPlugin::OverlayState st;
        fx_->overlayState(args.time, st);
        if (st.mode != kModeAnimate) return false;
        const int sel = fx_->selectedSlot();
        auto indexOf = [&](int slot) {
            for (size_t k = 0; k < st.slots.size(); ++k)
                if (st.slots[k] == slot) return (int)k;
            return -1;
        };
        auto drivenAt = [&](int k) { return k >= 0 && k < (int)st.driven.size() && st.driven[k]; };
        const int k = indexOf(sel);
        if (k < 0) return false;
        const std::vector<int> group = fx_->selectedSlots();
        const bool single = group.size() <= 1;
        if (single && drivenAt(k)) return false;
        const double t = args.time;
        xform_.clear();
        for (int slot : group) {
            const int j = indexOf(slot);
            if (j < 0 || (slot != sel && drivenAt(j))) continue;  // deformers move those
            xform_.push_back({slot, fx_->rawPin(slot, t, false), fx_->rawRotation(slot, t), fx_->rawScale(slot, t),
                              fx_->hasRadius(slot, t)});
        }
        if (single) {
            // A lone pin turns / scales its own area, which needs a radius.
            if (!xform_[0].radius) {
                openBlock(scaleKey ? "Scale Pin" : "Rotate Pin");
                fx_->ensureRadius(sel, t);
            }
            xform_[0].radius = true;
        }
        keyTurn_ = sel;
        keyTurnCentre_ = st.pins[k].dst;
        xformPivot_ = fx_->rawPin(sel, t, false);
        keyScaling_ = scaleKey;
        xformAmount_ = scaleKey ? 1.0 : 0.0;
        keyTurnRef_ = keyTurnMoved_ = keyDrag_ = false;
        keyTaken_ = true;
        requestRedraw();
        return true;
    }
    return false;
}

bool PuppetInteract::keyUp(const OFX::KeyArgs& args)
{
    if (isModifier(args.keySymbol)) {
        shift_ = false;
        return false;
    }
    if (!keyTaken_) return false;
    keyTaken_ = false;
    return true;
}

void PuppetInteract::loseFocus(const OFX::FocusArgs&)
{
    shift_ = false;
    if (keyTurn_ >= 0) {  // keep what was done, so the undo step closes
        endKeyTurn();
        requestRedraw();
    }
}

class PuppetOverlayDescriptor : public OFX::DefaultEffectOverlayDescriptor<PuppetOverlayDescriptor, PuppetInteract> {
};

// =======================================================================================

class PuppetPinFactory : public OFX::PluginFactoryHelper<PuppetPinFactory> {
public:
    PuppetPinFactory() : OFX::PluginFactoryHelper<PuppetPinFactory>(kPluginIdentifier, kPluginVersionMajor, kPluginVersionMinor) {}
    void load() override {}
    void unload() override {}
    void describe(OFX::ImageEffectDescriptor& desc) override;
    void describeInContext(OFX::ImageEffectDescriptor& desc, OFX::ContextEnum context) override;
    OFX::ImageEffect* createInstance(OfxImageEffectHandle handle, OFX::ContextEnum context) override;
};

void PuppetPinFactory::describe(OFX::ImageEffectDescriptor& desc)
{
    desc.setLabels(kPluginName, kPluginName, kPluginName);
    desc.setPluginGrouping(kPluginGrouping);
    desc.setPluginDescription(kPluginDescription);
    desc.addSupportedContext(OFX::eContextFilter);
    desc.addSupportedContext(OFX::eContextGeneral);
    desc.addSupportedBitDepth(OFX::eBitDepthFloat);
    desc.setSingleInstance(false);
    desc.setHostFrameThreading(false);
    desc.setSupportsMultiResolution(false);
    desc.setSupportsTiles(false);
    desc.setTemporalClipAccess(false);
    desc.setRenderTwiceAlways(false);
    desc.setSupportsMultipleClipPARs(false);
    desc.setRenderThreadSafety(OFX::eRenderInstanceSafe);
    desc.setOverlayInteractDescriptor(new PuppetOverlayDescriptor());
}

namespace {

template <class D>
D* finish(D* p, const std::string& label, const std::string& hint, OFX::GroupParamDescriptor* group,
          OFX::PageParamDescriptor* page)
{
    p->setLabels(label, label, label);
    p->setHint(hint);
    if (group) p->setParent(*group);
    if (page) page->addChild(*p);
    return p;
}

}  // namespace

void PuppetPinFactory::describeInContext(OFX::ImageEffectDescriptor& desc, OFX::ContextEnum /*context*/)
{
    OFX::ClipDescriptor* srcClip = desc.defineClip(kOfxImageEffectSimpleSourceClipName);
    srcClip->addSupportedComponent(OFX::ePixelComponentRGBA);
    srcClip->setTemporalClipAccess(false);
    srcClip->setSupportsTiles(false);
    srcClip->setIsMask(false);

    // Optional second input (Fusion / Color page): a mask that anchors or stiffens areas.
    OFX::ClipDescriptor* weightClip = desc.defineClip("WeightMap");
    weightClip->addSupportedComponent(OFX::ePixelComponentRGBA);
    weightClip->addSupportedComponent(OFX::ePixelComponentAlpha);
    weightClip->setOptional(true);
    weightClip->setTemporalClipAccess(false);
    weightClip->setSupportsTiles(false);
    weightClip->setIsMask(false);

    // Optional image layers (Fusion page): separate parts (arm, body, head) rigged together, each
    // meshed from its own alpha and drawn over the Source in input order.
    for (int i = 1; i < kMaxLayers; ++i) {
        OFX::ClipDescriptor* c = desc.defineClip(layerClipName(i));
        c->setLabels("Layer " + std::to_string(i + 1), "Layer " + std::to_string(i + 1),
                     "Layer " + std::to_string(i + 1));
        c->addSupportedComponent(OFX::ePixelComponentRGBA);
        c->setOptional(true);
        c->setTemporalClipAccess(false);
        c->setSupportsTiles(false);
        c->setIsMask(false);
    }

    OFX::ClipDescriptor* dstClip = desc.defineClip(kOfxImageEffectOutputClipName);
    dstClip->addSupportedComponent(OFX::ePixelComponentRGBA);
    dstClip->setSupportsTiles(false);

    // One page: Resolve shows plugin parameters on a single tab, so the parts are collapsible sections.
    OFX::PageParamDescriptor* page = desc.definePageParam("Controls");

    {
        auto* p = finish(desc.defineChoiceParam("mode"), "Pin Mode",
                         "Animate: drag pins to pose (keyframes with Auto Keyframe). "
                         "Edit Rest Pose: shows the original image; drag pins to change where they grab it.",
                         nullptr, page);
        p->appendOption("Animate");
        p->appendOption("Edit Rest Pose");
        p->setDefault(kModeAnimate);
        p->setAnimates(false);
    }
    {
        auto* p = finish(desc.defineChoiceParam("addMode"), "Add Pins By",
                         "How pins are added in the viewer. A single click on the image always selects/deselects.",
                         nullptr, page);
        p->appendOption("Double-Click on Image");
        p->appendOption("Single Click on Image");
        p->appendOption("Add Pin Button Only");
        p->setDefault(kAddDoubleClick);
        p->setAnimates(false);
        p->setEvaluateOnChange(false);
    }
    for (const char* id : {"pinFrameOrigin", "pinFrameSize"}) {
        // set by "Puppet Pin - Add Room"; hidden
        auto* p = desc.defineDouble2DParam(id);
        p->setLabel(id);
        p->setDefault(0.0, 0.0);
        p->setDoubleType(OFX::eDoubleTypePlain);
        p->setRange(-1e7, -1e7, 1e7, 1e7);
        p->setAnimates(false);
        p->setIsSecret(true);
        if (page) page->addChild(*p);
    }
    {
        auto* p = finish(desc.defineBooleanParam("showMesh"), "Show Mesh", "Draw the mesh in the viewer overlay.",
                         nullptr, page);
        p->setDefault(true);
        p->setAnimates(false);
        p->setEvaluateOnChange(false);
    }
    {
        auto* p = finish(desc.defineBooleanParam("showPins"), "Show Pins", "Draw the pins in the viewer overlay.",
                         nullptr, page);
        p->setDefault(true);
        p->setAnimates(false);
        p->setEvaluateOnChange(false);
    }
    {
        auto* p = finish(desc.defineIntParam("selectedPin"), "Selected Pin",
                         "The selected pin (click a pin in the viewer). 0 = none.", nullptr, page);
        p->setDefault(0);
        p->setRange(0, kMaxPins);
        p->setDisplayRange(0, kMaxPins);
        p->setAnimates(false);
        p->setEvaluateOnChange(false);
    }
    {
        auto* p = finish(desc.defineStringParam("selectedPins"), "Selected Pins",
                         "Every selected pin, e.g. 8, 9, 10. In the viewer, drag a box around pins (from an empty "
                         "spot) or Shift+click pins to select several; drag one of them to move them all. E rotates "
                         "and S scales them around the Selected Pin (drawn red; click another selected pin to make "
                         "it the pivot).",
                         nullptr, page);
        p->setDefault("");
        p->setStringType(OFX::eStringTypeSingleLine);
        p->setAnimates(false);
        p->setEvaluateOnChange(false);
    }
    finish(desc.definePushButtonParam("addPin"), "Add Pin", "Adds a pin at the centre of the frame.", nullptr, page);
    finish(desc.definePushButtonParam("removePin"), "Remove Selected Pins", "Deletes the selected pins.", nullptr,
           page);
    finish(desc.definePushButtonParam("resetPin"), "Reset Selected Pins",
           "Moves the selected pins back to their rest positions and deletes their keyframes.", nullptr, page);
    finish(desc.definePushButtonParam("removeAll"), "Remove All Pins", "Deletes every pin.", nullptr, page);

    // Pins
    auto* pinsGroup = finish(desc.defineGroupParam("pinsGroup"), "Pins",
                             "Settings of each pin (only pins in use are listed).", nullptr, page);
    pinsGroup->setOpen(true);
    for (int i = 0; i < kMaxPins; ++i) {
        const std::string n = std::to_string(i + 1);
        auto* g = finish(desc.defineGroupParam(pinParam(i, "group")), "Pin " + n, "Pin " + n, pinsGroup, page);
        g->setOpen(false);
        {
            auto* p = finish(desc.defineBooleanParam(pinParam(i, "on")), "Enabled", "Pin " + n + " is active.", g, page);
            p->setDefault(false);
            p->setAnimates(false);
        }
        {
            auto* p = finish(desc.defineDouble2DParam(pinParam(i, "pos")), "Position",
                             "Where the pin is (fraction of the frame, 0..1, origin bottom left).", g, page);
            p->setDefault(0.5, 0.5);
            p->setDoubleType(OFX::eDoubleTypePlain);
            p->setIncrement(0.001);
            p->setDigits(4);
            p->setDisplayRange(0.0, 0.0, 1.0, 1.0);
            p->setRange(-100.0, -100.0, 100.0, 100.0);
        }
        {
            auto* p = finish(desc.defineDouble2DParam(pinParam(i, "rest")), "Rest Position",
                             "The point of the original image the pin grabs.", g, page);
            p->setDefault(0.5, 0.5);
            p->setDoubleType(OFX::eDoubleTypePlain);
            p->setIncrement(0.001);
            p->setDigits(4);
            p->setDisplayRange(0.0, 0.0, 1.0, 1.0);
            p->setRange(-100.0, -100.0, 100.0, 100.0);
        }
        {
            auto* p = finish(desc.defineDoubleParam(pinParam(i, "radius")), "Radius",
                             "Pixels around the pin that move rigidly with it. Needed for Rotation and Scale.", g,
                             page);
            p->setDefault(0.0);
            p->setRange(0.0, 100000.0);
            p->setDisplayRange(0.0, 300.0);
        }
        {
            auto* p = finish(desc.defineDoubleParam(pinParam(i, "influence")), "Influence",
                             "How far the pin pulls the image, in pixels measured along the shape. Beyond it the "
                             "image stays as it would be without this pin. 0 = no limit. In the viewer, drag the "
                             "green handle on the circle.",
                             g, page);
            p->setDefault(0.0);
            p->setRange(0.0, 100000.0);
            p->setDisplayRange(0.0, 600.0);
        }
        {
            auto* p = finish(desc.defineDoubleParam(pinParam(i, "falloff")), "Influence Falloff",
                             "How soft the edge of the influence is: the share of it over which the pull fades out. "
                             "0 = hard edge.",
                             g, page);
            p->setDefault(0.5);
            p->setRange(0.0, 1.0);
            p->setDisplayRange(0.0, 1.0);
        }
        {
            auto* p = finish(desc.defineDoubleParam(pinParam(i, "rotation")), "Rotation",
                             "Rotates the area inside the radius (degrees). A pin without a radius gets one.", g, page);
            p->setDefault(0.0);
            p->setDoubleType(OFX::eDoubleTypeAngle);
            p->setRange(-36000.0, 36000.0);
            p->setDisplayRange(-180.0, 180.0);
        }
        {
            auto* p = finish(desc.defineDoubleParam(pinParam(i, "scale")), "Scale",
                             "Scales the area inside the radius. A pin without a radius gets one.", g, page);
            p->setDefault(1.0);
            p->setRange(0.01, 100.0);
            p->setDisplayRange(0.1, 3.0);
        }
        {
            auto* p = finish(desc.defineDoubleParam(pinParam(i, "strength")), "Strength",
                             "How hard the pin pulls. Lower values give a softer, lagging pull.", g, page);
            p->setDefault(1.0);
            p->setRange(0.0, 1.0);
            p->setDisplayRange(0.0, 1.0);
        }
        {
            auto* p = finish(desc.defineIntParam(pinParam(i, "layer")), "Layer",
                             "Where parts overlap, the part around the pin with the higher layer is drawn on top. "
                             "With equal layers, parts further out from the middle of the body (hands, feet, head) "
                             "go over the parts closer to it. Can be keyframed: an arm swings behind the body, then "
                             "in front.",
                             g, page);
            p->setDefault(0);
            p->setRange(-100, 100);
            p->setDisplayRange(-10, 10);
            p->setAnimates(true);
        }
    }
    // Mesh
    auto* meshGroup = finish(desc.defineGroupParam("meshGroup"), "Mesh", "How the mesh is built from the image.",
                             nullptr, page);
    meshGroup->setOpen(false);
    {
        auto* p = finish(desc.defineChoiceParam("meshSource"), "Mesh From",
                         "Alpha: the mesh covers the non-transparent pixels (separate parts move separately). "
                         "Full Frame: one mesh over the whole frame.",
                         meshGroup, page);
        p->appendOption("Alpha");
        p->appendOption("Full Frame");
        p->setDefault(0);
        p->setAnimates(false);
    }
    {
        auto* p = finish(desc.defineIntParam("triangles"), "Triangles",
                         "Approximate number of mesh triangles. More follow the outline closer and bend smoother.",
                         meshGroup, page);
        p->setDefault(800);
        p->setRange(20, 20000);
        p->setDisplayRange(100, 5000);
        p->setAnimates(false);
    }
    {
        auto* p = finish(desc.defineDoubleParam("expansion"), "Expansion",
                         "How far (pixels) the mesh reaches past the edge of the alpha.", meshGroup, page);
        p->setDefault(3.0);
        p->setRange(0.0, 200.0);
        p->setDisplayRange(0.0, 30.0);
        p->setAnimates(false);
    }
    {
        auto* p = finish(desc.defineDoubleParam("threshold"), "Alpha Threshold",
                         "Pixels with more alpha than this are part of the mesh.", meshGroup, page);
        p->setDefault(0.05);
        p->setRange(0.0, 0.999);
        p->setDisplayRange(0.0, 1.0);
        p->setAnimates(false);
    }

    // Solver
    auto* solverGroup = finish(desc.defineGroupParam("solverGroup"), "Solver", "Deformation settings.", nullptr, page);
    solverGroup->setOpen(false);
    solverGroup->setOpen(false);
    {
        auto* p = finish(desc.defineDoubleParam("rigidity"), "Rigidity",
                         "1 keeps every part's shape and size (as rigid as possible); lower values let parts "
                         "stretch and squash.",
                         solverGroup, page);
        p->setDefault(1.0);
        p->setRange(0.0, 1.0);
        p->setDisplayRange(0.0, 1.0);
    }
    {
        auto* p = finish(desc.defineIntParam("iterations"), "Iterations",
                         "Solver iterations. More is stiffer and more accurate, but slower.", solverGroup, page);
        p->setDefault(12);
        p->setRange(0, 200);
        p->setDisplayRange(1, 50);
        p->setAnimates(false);
    }

    {
        auto* p = finish(desc.defineChoiceParam("solverMode"), "Solver",
                         "Static: every frame is solved on its own. Smooth Follow: the image lags behind the pins "
                         "and catches up. Elastic: it lags, overshoots and wobbles before it settles (jiggle).",
                         solverGroup, page);
        p->appendOption("Static");
        p->appendOption("Smooth Follow");
        p->appendOption("Elastic (Jiggle)");
        p->setDefault(kSolveStatic);
        p->setAnimates(false);
        desc.addClipPreferencesSlaveParam(*p);
    }
    {
        auto* p = finish(desc.defineDoubleParam("firmness"), "Firmness",
                         "How quickly the material catches up, in wobbles per second. Low = soft and slow.", solverGroup,
                         page);
        p->setDefault(2.0);
        p->setRange(0.05, 60.0);
        p->setDisplayRange(0.3, 8.0);
        p->setAnimates(false);
    }
    {
        auto* p = finish(desc.defineDoubleParam("damping"), "Damping",
                         "How quickly the wobble dies down. 0 = wobbles for a long time, 1 = no overshoot.",
                         solverGroup, page);
        p->setDefault(0.15);
        p->setRange(0.0, 1.0);
        p->setDisplayRange(0.0, 1.0);
        p->setAnimates(false);
    }
    {
        auto* p = finish(desc.defineDoubleParam("jiggleReach"), "Jiggle Reach",
                         "How far from the pins the lag/jiggle reaches full strength (share of the mesh size). "
                         "Areas at the pins always follow them exactly.",
                         solverGroup, page);
        p->setDefault(0.25);
        p->setRange(0.01, 4.0);
        p->setDisplayRange(0.05, 1.0);
        p->setAnimates(false);
    }
    {
        auto* p = finish(desc.defineChoiceParam("simStartMode"), "Simulation Start",
                         "The frame where the simulation starts at rest.", solverGroup, page);
        p->appendOption("First Frame of the Clip");
        p->appendOption("Custom Frame");
        p->setDefault(0);
        p->setAnimates(false);
    }
    {
        auto* p = finish(desc.defineIntParam("simStart"), "Start Frame", "Frame where the simulation starts.",
                         solverGroup, page);
        p->setDefault(0);
        p->setRange(-1000000, 1000000);
        p->setDisplayRange(0, 500);
        p->setAnimates(false);
    }

    // Deformers
    auto* deformersGroup = finish(desc.defineGroupParam("deformersGroup"), "Deformers",
                                  "Rigs that move pins automatically: Follower, Wave, IK, FK, Wiggle, Wind, Spring.",
                                  nullptr, page);
    deformersGroup->setOpen(false);
    for (int k = 0; k < kMaxDeformers; ++k) {
        const std::string n = "def" + std::to_string(k + 1) + "_";
        auto* g = finish(desc.defineGroupParam(n + "group"), "Deformer " + std::to_string(k + 1),
                         "Moves pins procedurally before the mesh is solved.", deformersGroup, page);
        g->setOpen(k == 0);
        {
            auto* p = finish(desc.defineChoiceParam(n + "type"), "Type",
                             "Follower: the first pin leads, the others copy its motion (optionally late). "
                             "Wave: a chain swings around its first pin (tails, hair). "
                             "IK: root, joint, end - move the end pin and the joint bends to keep bone lengths. "
                             "FK: a chain posed with joint angles. "
                             "Wiggle: the pins wander around randomly. "
                             "Wind: a chain leans into the wind, with gusts. "
                             "Spring: the followers trail the leader on a spring (lag, overshoot, settle).",
                             g, page);
            desc.addClipPreferencesSlaveParam(*p);
            p->appendOption("Off");
            p->appendOption("Follower");
            p->appendOption("Wave");
            p->appendOption("IK (Root, Joint, End)");
            p->appendOption("FK Chain");
            p->appendOption("Wiggle");
            p->appendOption("Wind");
            p->appendOption("Spring");
            p->setDefault(0);
            p->setAnimates(false);
        }
        {
            auto* p = finish(desc.defineStringParam(n + "pins"), "Pins",
                             "Pin numbers separated by commas. Follower / Spring: leader first, then followers "
                             "(e.g. 4, 3). Wave / Wind / FK: base first, then the chain outwards. IK: root, joint, end "
                             "(hip, knee, foot). Wiggle: any pins.",
                             g, page);
            p->setDefault("");
            p->setStringType(OFX::eStringTypeSingleLine);
            p->setAnimates(false);
        }
        auto num = [&](const char* id, const char* label, const char* hint, double def, double lo, double hi,
                       double dlo, double dhi, bool angle) {
            auto* p = finish(desc.defineDoubleParam(n + id), label, hint, g, page);
            p->setDefault(def);
            p->setRange(lo, hi);
            p->setDisplayRange(dlo, dhi);
            if (angle) p->setDoubleType(OFX::eDoubleTypeAngle);
        };
        num("amount", "Follow Amount", "How much of the leader's motion the followers copy.", 0.6, -10, 10, 0, 1.5,
            false);
        num("delay", "Delay (frames)", "How late the followers copy the leader.", 3, -1000, 1000, 0, 30, false);
        num("bend", "Bend", "Static rotation of the chain around its first pin (degrees).", 0, -3600, 3600, -90, 90,
            true);
        num("swing", "Swing", "Oscillation size at every joint (degrees).", 15, -360, 360, 0, 60, true);
        num("speed", "Speed", "How fast: swings (Wave), wiggles (Wiggle) or gusts (Wind) per second.", 1, 0, 100, 0,
            5, false);
        num("lag", "Lag per Joint (frames)", "How much later each joint swings than the one before it.", 3, -1000,
            1000, 0, 20, false);
        num("phase", "Phase", "Offset of the oscillation (degrees).", 0, -36000, 36000, -180, 180, true);
        {
            auto* p = finish(desc.defineBooleanParam(n + "flip"), "Flip Bend", "Bend the joint the other way.", g, page);
            p->setDefault(false);
        }
        num("angle1", "Root Angle", "Rotation at the first pin of the chain (degrees).", 0, -36000, 36000, -180, 180,
            true);
        num("angle2", "Joint 2 Angle", "Rotation at the second pin (degrees).", 0, -36000, 36000, -180, 180, true);
        num("angle3", "Joint 3+ Angle", "Rotation at the third and further pins (degrees).", 0, -36000, 36000, -180,
            180, true);
        num("size", "Size", "How far the pins wander (pixels).", 20, 0, 100000, 0, 200, false);
        num("spin", "Rotation", "How much pins with a radius turn back and forth (degrees).", 0, 0, 3600, 0, 90, true);
        num("direction", "Direction", "Where the wind blows to (0 = right, 90 = up, 180 = left).", 0, -36000, 36000,
            -180, 180, true);
        num("strength", "Strength", "How far each joint bends in a full gust (degrees).", 20, -3600, 3600, 0, 90, true);
        num("gust", "Gustiness", "0 = steady wind, 1 = strong gusts.", 0.5, 0, 1, 0, 1, false);
        num("seed", "Seed", "Changes the random pattern.", 0, -100000, 100000, 0, 100, false);
        num("firm", "Firmness", "How quickly the followers catch up: wobbles per second.", 2, 0.05, 60, 0.3, 8, false);
        num("damp", "Damping", "How quickly the wobble dies down. 0 = wobbles long, 1 = no overshoot.", 0.2, 0, 1, 0,
            1, false);
    }

    // Animation
    auto* animGroup = finish(desc.defineGroupParam("animGroup"), "Animation",
                             "Auto Keyframe and looping of the pin animation.", nullptr, page);
    animGroup->setOpen(false);
    {
        auto* p = finish(desc.defineChoiceParam("keyMode"), "Keyframes When Dragging",
                         "Only Keyframed Pins: dragging, turning or scaling a pin sets a keyframe only if that "
                         "setting already has keyframes (set the first one yourself with its keyframe button); "
                         "otherwise the pin just moves. Every Pin: every drag sets a keyframe.",
                         animGroup, page);
        p->appendOption("Only Keyframed Pins");
        p->appendOption("Every Pin");
        p->setDefault(0);
        p->setAnimates(false);
        p->setEvaluateOnChange(false);
    }
    {
        auto* p = finish(desc.defineChoiceParam("loopMode"), "Loop Pin Animation",
                         "After Loop End, repeat the pin keyframes from Loop Start to Loop End (Loop), or play "
                         "them back and forth (Ping-Pong).",
                         animGroup, page);
        p->appendOption("Off");
        p->appendOption("Loop");
        p->appendOption("Ping-Pong");
        p->setDefault(0);
        p->setAnimates(false);
        desc.addClipPreferencesSlaveParam(*p);
    }
    {
        auto* p = finish(desc.defineIntParam("loopStart"), "Loop Start", "First frame of the loop.", animGroup, page);
        p->setDefault(0);
        p->setRange(-1000000, 1000000);
        p->setDisplayRange(0, 300);
        p->setAnimates(false);
    }
    {
        auto* p = finish(desc.defineIntParam("loopEnd"), "Loop End",
                         "Last frame of the loop; after it the animation repeats.", animGroup, page);
        p->setDefault(24);
        p->setRange(-1000000, 1000000);
        p->setDisplayRange(0, 300);
        p->setAnimates(false);
    }
    finish(desc.definePushButtonParam("keyAllPins"), "Key All Pins",
           "Sets a position keyframe on every pin at this frame (and rotation / scale where a pin uses them).",
           animGroup, page);
    finish(desc.definePushButtonParam("copyPose"), "Copy Pose",
           "Remembers every pin's position, rotation and scale at this frame. It can be pasted on another frame or "
           "into another Puppet Pin node.",
           animGroup, page);
    finish(desc.definePushButtonParam("pastePose"), "Paste Pose",
           "Puts the copied pose on the pins at this frame (only on the selected pins when several are selected). "
           "Keyframes are set like when dragging (see Keyframes When Dragging).",
           animGroup, page);
    finish(desc.definePushButtonParam("mirrorPose"), "Mirror Pose",
           "Mirrors the selected pins' pose onto their partners on the other side of the body (the left arm's pose "
           "goes onto the right arm). With nothing selected the whole pose flips left / right.",
           animGroup, page);
    {
        auto* p = finish(desc.defineBooleanParam("onionSkin"), "Onion Skin",
                         "Shows the outline of the pose a few frames before (orange) and after (blue) in the viewer.",
                         animGroup, page);
        p->setDefault(false);
        p->setAnimates(false);
        p->setEvaluateOnChange(false);
    }
    {
        auto* p = finish(desc.defineIntParam("onionBefore"), "Frames Before", "How many outlines before this frame.",
                         animGroup, page);
        p->setDefault(2);
        p->setRange(0, 10);
        p->setDisplayRange(0, 10);
        p->setAnimates(false);
        p->setEvaluateOnChange(false);
    }
    {
        auto* p = finish(desc.defineIntParam("onionAfter"), "Frames After", "How many outlines after this frame.",
                         animGroup, page);
        p->setDefault(2);
        p->setRange(0, 10);
        p->setDisplayRange(0, 10);
        p->setAnimates(false);
        p->setEvaluateOnChange(false);
    }
    {
        auto* p = finish(desc.defineIntParam("onionStep"), "Frame Step", "Frames between the outlines.", animGroup,
                         page);
        p->setDefault(2);
        p->setRange(1, 100);
        p->setDisplayRange(1, 12);
        p->setAnimates(false);
        p->setEvaluateOnChange(false);
    }
    {
        auto* p = finish(desc.defineBooleanParam("motionTrails"), "Motion Trails",
                         "Shows the path of the selected pins over the frames around this one: a dot per frame, "
                         "diamonds on keyframes, earlier frames orange, later ones blue.",
                         animGroup, page);
        p->setDefault(false);
        p->setAnimates(false);
        p->setEvaluateOnChange(false);
    }
    {
        auto* p = finish(desc.defineIntParam("trailFrames"), "Trail Length",
                         "How many frames before and after this one the trails show.", animGroup, page);
        p->setDefault(12);
        p->setRange(1, 240);
        p->setDisplayRange(1, 48);
        p->setAnimates(false);
        p->setEvaluateOnChange(false);
    }
    // Locators
    OFX::GroupParamDescriptor* locatorsGroup = nullptr;
    {
        auto* g = finish(desc.defineGroupParam("locatorGroup"), "Locators",
                         "Track points of the image (a hat on a head): Add Locator, drag its pink diamond onto the "
                         "point, then Bake Locators. Other nodes can follow the baked Position / Rotation.",
                         nullptr, page);
        g->setOpen(false);
        locatorsGroup = g;
        finish(desc.definePushButtonParam("addLocator"), "Add Locator",
               "Adds a locator at the selected pin (or the middle). Drag its pink diamond in the viewer to the point "
               "it should follow.",
               g, page);
        finish(desc.definePushButtonParam("bakeLocators"), "Bake Locators",
               "Writes every locator's Position and Rotation as keyframes for each frame of the clip. Bake again "
               "after changing the animation.",
               g, page);
    }
    for (int k = 0; k < kMaxLocators; ++k) {
        const std::string n = std::to_string(k + 1);
        auto* g = finish(desc.defineGroupParam(locParam(k, "group")), "Locator " + n, "Locator " + n, locatorsGroup,
                         page);
        g->setOpen(false);
        {
            auto* p = finish(desc.defineBooleanParam(locParam(k, "on")), "Enabled", "Locator " + n + " is active.", g,
                             page);
            p->setDefault(false);
            p->setAnimates(false);
        }
        {
            auto* p = finish(desc.defineDouble2DParam(locParam(k, "rest")), "Point on Image",
                             "The point of the original image this locator follows (0..1 of the frame).", g, page);
            p->setDefault(0.5, 0.5);
            p->setDoubleType(OFX::eDoubleTypePlain);
            p->setDigits(4);
            p->setIncrement(0.001);
            p->setDisplayRange(0.0, 0.0, 1.0, 1.0);
            p->setRange(-100.0, -100.0, 100.0, 100.0);
        }
        {
            auto* p = finish(desc.defineDouble2DParam(locParam(k, "pos")), "Position",
                             "Baked: where the point is on each frame (0..1 of the frame). In Fusion, other nodes can "
                             "follow it, e.g. an expression on a Transform's Center: PuppetPin1.loc" + n + "_pos",
                             g, page);
            p->setDefault(0.5, 0.5);
            p->setDoubleType(OFX::eDoubleTypePlain);
            p->setDigits(4);
            p->setDisplayRange(0.0, 0.0, 1.0, 1.0);
            p->setRange(-100.0, -100.0, 100.0, 100.0);
        }
        {
            auto* p = finish(desc.defineDoubleParam(locParam(k, "rot")), "Rotation",
                             "Baked: how much the image has turned at the point (degrees).", g, page);
            p->setDefault(0.0);
            p->setDoubleType(OFX::eDoubleTypeAngle);
            p->setRange(-36000.0, 36000.0);
            p->setDisplayRange(-180.0, 180.0);
        }
    }

    // Weight map
    auto* weightGroup = finish(desc.defineGroupParam("weightGroup"), "Weight Map",
                               "Connect a mask or matte to the node's Weight Map input (Fusion page).", nullptr, page);
    weightGroup->setOpen(false);
    {
        auto* p = finish(desc.defineChoiceParam("weightMode"), "Use Weight Map To",
                         "Anchor: white areas stay where they are (grey holds back part of the motion). "
                         "Stiffen: white areas keep their shape but can still move.",
                         weightGroup, page);
        p->appendOption("Anchor (white holds still)");
        p->appendOption("Stiffen (white keeps its shape)");
        p->setDefault(0);
        p->setAnimates(false);
    }
    {
        auto* p = finish(desc.defineChoiceParam("weightChannel"), "Read From", "Which channel of the weight map is used.",
                         weightGroup, page);
        p->appendOption("Luminance");
        p->appendOption("Alpha");
        p->setDefault(0);
        p->setAnimates(false);
    }
    {
        auto* p = finish(desc.defineBooleanParam("weightInvert"), "Invert", "Use black instead of white.", weightGroup,
                         page);
        p->setDefault(false);
        p->setAnimates(false);
    }

    // Rig files
    {
        auto* g = finish(desc.defineGroupParam("rigGroup"), "Save / Load Rig",
                         "Saves the pins, deformers, mesh and solver settings to a file, to use on another clip.",
                         nullptr, page);
        g->setOpen(false);
        {
            auto* p = finish(desc.defineStringParam("rigFile"), "Rig File",
                             "Where Save Rig writes and Load Rig reads. Empty: Documents\\PuppetPin Rigs\\Rig.pprig.",
                             g, page);
            p->setDefault("");
            p->setStringType(OFX::eStringTypeFilePath);
            p->setFilePathExists(false);
            p->setAnimates(false);
            p->setEvaluateOnChange(false);
        }
        finish(desc.definePushButtonParam("saveRig"), "Save Rig",
               "Writes the pins (rest points, positions and settings, without keyframes), the deformers and the "
               "mesh / solver settings to the Rig File.",
               g, page);
        finish(desc.definePushButtonParam("loadRig"), "Load Rig",
               "Replaces every pin and deformer with the ones in the Rig File. Pin points are stored relative to the "
               "frame, so a similar picture fits; fix single pins in Edit Rest Pose.",
               g, page);
    }

    // Render
    auto* renderGroup = finish(desc.defineGroupParam("renderGroup"), "Render", "Output settings.", nullptr, page);
    renderGroup->setOpen(false);
    {
        auto* p = finish(desc.defineChoiceParam("outputMode"), "Output",
                         "Composite: every image layer, posed. Single Layer: only one layer (to grade or "
                         "composite it separately). Motion Vectors: how far each pixel moved since the previous "
                         "frame (red = x, green = y, in pixels) for Vector Motion Blur.",
                         renderGroup, page);
        p->appendOption("Composite (All Layers)");
        p->appendOption("Single Layer");
        p->appendOption("Motion Vectors");
        p->setDefault(kOutComposite);
        p->setAnimates(false);
    }
    {
        auto* p = finish(desc.defineIntParam("outputLayer"), "Layer",
                         "Which layer Single Layer shows: 1 = the Source input, 2..16 = the Layer inputs.", renderGroup,
                         page);
        p->setDefault(1);
        p->setRange(1, kMaxLayers);
        p->setDisplayRange(1, kMaxLayers);
        p->setAnimates(false);
    }
    {
        auto* p = finish(desc.defineChoiceParam("antialias"), "Antialiasing",
                         "Supersampling of the warped image. Mostly matters for opaque edges.", renderGroup, page);
        p->appendOption("Off");
        p->appendOption("2x2");
        p->appendOption("3x3");
        p->appendOption("4x4");
        p->setDefault(1);
        p->setAnimates(false);
    }
    {
        auto* p = finish(desc.defineBooleanParam("keepUnmeshed"), "Keep Unmeshed Pixels",
                         "Pixels outside the mesh (below the alpha threshold) stay where they are instead of "
                         "being removed.",
                         renderGroup, page);
        p->setDefault(true);
        p->setAnimates(false);
    }
    {
        auto* p = finish(desc.defineBooleanParam("burnIn"), "Draw Overlay Into Image",
                         "Draws the mesh and pins into the output (for hosts or pages without overlay support).",
                         renderGroup, page);
        p->setDefault(false);
        p->setAnimates(false);
    }
    {
        auto* p = finish(desc.defineBooleanParam("motionBlur"), "Motion Blur",
                         "Blurs moving parts along their motion, like a camera shutter.", renderGroup, page);
        p->setDefault(false);
        p->setAnimates(false);
    }
    {
        auto* p = finish(desc.defineDoubleParam("shutter"), "Shutter",
                         "How long the shutter is open, in frames: 0.5 = a 180 degree shutter, 1 = a full frame of "
                         "motion.",
                         renderGroup, page);
        p->setDefault(0.5);
        p->setRange(0.0, 2.0);
        p->setDisplayRange(0.0, 1.0);
    }
    {
        auto* p = finish(desc.defineIntParam("blurSamples"), "Blur Quality",
                         "Images averaged per frame. More is smoother and slower.", renderGroup, page);
        p->setDefault(8);
        p->setRange(2, 64);
        p->setDisplayRange(2, 32);
        p->setAnimates(false);
    }

}

OFX::ImageEffect* PuppetPinFactory::createInstance(OfxImageEffectHandle handle, OFX::ContextEnum /*context*/)
{
    return new PuppetPinPlugin(handle);
}

void OFX::Plugin::getPluginIDs(OFX::PluginFactoryArray& ids)
{
    static PuppetPinFactory factory;
    ids.push_back(&factory);
}
