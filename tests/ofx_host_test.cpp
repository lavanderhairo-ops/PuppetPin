// Minimal OFX host used to test PuppetPin.ofx end to end without Resolve: loads the
// binary, describes it, creates an instance, drives the viewer overlay with simulated
// mouse events, renders frames, and draws the overlay into PNGs.
#include "test_util.h"

#include "ofxCore.h"
#include "ofxDrawSuite.h"
#include "ofxImageEffect.h"
#include "ofxInteract.h"
#include "ofxKeySyms.h"
#include "ofxMemory.h"
#include "ofxMessage.h"
#include "ofxMultiThread.h"
#include "ofxParam.h"
#include "ofxProperty.h"

#include <windows.h>

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {

int gFailures = 0;
#define CHECK(cond, ...)                         \
    do {                                         \
        if (!(cond)) {                           \
            std::printf("  FAIL: " __VA_ARGS__); \
            std::printf("\n");                   \
            ++gFailures;                         \
        }                                        \
    } while (0)

// ------------------------------------------------------------------------------------
// Properties (lenient: unknown properties read as zero / empty).

struct Prop {
    std::vector<int> i;
    std::vector<double> d;
    std::vector<std::string> s;
    std::vector<void*> p;
};
struct PropSet {
    std::map<std::string, Prop> m;
    Prop& operator[](const std::string& k) { return m[k]; }
    void setS(const char* k, const std::string& v) { m[k].s = {v}; }
    void setI(const char* k, std::vector<int> v) { m[k].i = v; }
    void setD(const char* k, std::vector<double> v) { m[k].d = v; }
    void setP(const char* k, void* v) { m[k].p = {v}; }
    std::string getS(const char* k, int i = 0)
    {
        auto it = m.find(k);
        return it != m.end() && (int)it->second.s.size() > i ? it->second.s[i] : std::string();
    }
    void* getP(const char* k)
    {
        auto it = m.find(k);
        return it != m.end() && !it->second.p.empty() ? it->second.p[0] : nullptr;
    }
};
PropSet* PS(OfxPropertySetHandle h) { return reinterpret_cast<PropSet*>(h); }
OfxPropertySetHandle H(PropSet* p) { return reinterpret_cast<OfxPropertySetHandle>(p); }

template <class T> void putAt(std::vector<T>& v, int idx, const T& x)
{
    if ((int)v.size() <= idx) v.resize(idx + 1);
    v[idx] = x;
}

OfxStatus propSetPointer(OfxPropertySetHandle h, const char* k, int i, void* v) { putAt((*PS(h))[k].p, i, v); return kOfxStatOK; }
OfxStatus propSetString(OfxPropertySetHandle h, const char* k, int i, const char* v) { putAt((*PS(h))[k].s, i, std::string(v ? v : "")); return kOfxStatOK; }
OfxStatus propSetDouble(OfxPropertySetHandle h, const char* k, int i, double v) { putAt((*PS(h))[k].d, i, v); return kOfxStatOK; }
OfxStatus propSetInt(OfxPropertySetHandle h, const char* k, int i, int v) { putAt((*PS(h))[k].i, i, v); return kOfxStatOK; }
OfxStatus propSetPointerN(OfxPropertySetHandle h, const char* k, int n, void* const* v) { (*PS(h))[k].p.assign(v, v + n); return kOfxStatOK; }
OfxStatus propSetStringN(OfxPropertySetHandle h, const char* k, int n, const char* const* v)
{
    auto& s = (*PS(h))[k].s;
    s.clear();
    for (int i = 0; i < n; ++i) s.push_back(v[i]);
    return kOfxStatOK;
}
OfxStatus propSetDoubleN(OfxPropertySetHandle h, const char* k, int n, const double* v) { (*PS(h))[k].d.assign(v, v + n); return kOfxStatOK; }
OfxStatus propSetIntN(OfxPropertySetHandle h, const char* k, int n, const int* v) { (*PS(h))[k].i.assign(v, v + n); return kOfxStatOK; }
OfxStatus propGetPointer(OfxPropertySetHandle h, const char* k, int i, void** v)
{
    auto& p = (*PS(h))[k].p;
    *v = (int)p.size() > i ? p[i] : nullptr;
    return kOfxStatOK;
}
OfxStatus propGetString(OfxPropertySetHandle h, const char* k, int i, char** v)
{
    auto& s = (*PS(h))[k].s;
    static char empty[1] = {0};
    *v = (int)s.size() > i ? const_cast<char*>(s[i].c_str()) : empty;
    return kOfxStatOK;
}
OfxStatus propGetDouble(OfxPropertySetHandle h, const char* k, int i, double* v)
{
    auto& p = (*PS(h))[k];
    *v = (int)p.d.size() > i ? p.d[i] : ((int)p.i.size() > i ? p.i[i] : 0.0);
    return kOfxStatOK;
}
OfxStatus propGetInt(OfxPropertySetHandle h, const char* k, int i, int* v)
{
    auto& p = (*PS(h))[k];
    *v = (int)p.i.size() > i ? p.i[i] : ((int)p.d.size() > i ? (int)p.d[i] : 0);
    return kOfxStatOK;
}
OfxStatus propGetPointerN(OfxPropertySetHandle h, const char* k, int n, void** v) { for (int i = 0; i < n; ++i) propGetPointer(h, k, i, &v[i]); return kOfxStatOK; }
OfxStatus propGetStringN(OfxPropertySetHandle h, const char* k, int n, char** v) { for (int i = 0; i < n; ++i) propGetString(h, k, i, &v[i]); return kOfxStatOK; }
OfxStatus propGetDoubleN(OfxPropertySetHandle h, const char* k, int n, double* v) { for (int i = 0; i < n; ++i) propGetDouble(h, k, i, &v[i]); return kOfxStatOK; }
OfxStatus propGetIntN(OfxPropertySetHandle h, const char* k, int n, int* v) { for (int i = 0; i < n; ++i) propGetInt(h, k, i, &v[i]); return kOfxStatOK; }
OfxStatus propReset(OfxPropertySetHandle h, const char* k) { PS(h)->m.erase(k); return kOfxStatOK; }
OfxStatus propGetDimension(OfxPropertySetHandle h, const char* k, int* n)
{
    auto& p = (*PS(h))[k];
    *n = (int)std::max({p.i.size(), p.d.size(), p.s.size(), p.p.size()});
    return kOfxStatOK;
}
OfxPropertySuiteV1 gPropSuite = {propSetPointer, propSetString, propSetDouble, propSetInt, propSetPointerN,
                                 propSetStringN, propSetDoubleN, propSetIntN, propGetPointer, propGetString,
                                 propGetDouble, propGetInt, propGetPointerN, propGetStringN, propGetDoubleN,
                                 propGetIntN, propReset, propGetDimension};

// ------------------------------------------------------------------------------------
// Parameters.

double gTime = 0.0;  // the host's current time

struct Param {
    std::string name, type;
    PropSet props;
    std::vector<double> value;
    std::string sval;
    std::map<double, std::vector<double>> keys;
    int dims() const
    {
        if (type == kOfxParamTypeDouble2D || type == kOfxParamTypeInteger2D) return 2;
        if (type == kOfxParamTypeDouble3D || type == kOfxParamTypeInteger3D || type == kOfxParamTypeRGB) return 3;
        if (type == kOfxParamTypeRGBA) return 4;
        return 1;
    }
    bool isInt() const
    {
        return type == kOfxParamTypeInteger || type == kOfxParamTypeBoolean || type == kOfxParamTypeChoice ||
               type == kOfxParamTypeInteger2D || type == kOfxParamTypeInteger3D;
    }
    bool hasValue() const
    {
        return type != kOfxParamTypeGroup && type != kOfxParamTypePage && type != kOfxParamTypePushButton &&
               type != kOfxParamTypeString && type != kOfxParamTypeCustom;
    }
    std::vector<double> at(double t) const
    {
        if (keys.empty()) return value;
        auto hi = keys.lower_bound(t);
        if (hi == keys.begin()) return hi->second;
        if (hi == keys.end()) return std::prev(hi)->second;
        if (hi->first == t) return hi->second;
        auto lo = std::prev(hi);
        if (isInt()) return lo->second;
        const double f = (t - lo->first) / (hi->first - lo->first);
        std::vector<double> r(lo->second.size());
        for (size_t i = 0; i < r.size(); ++i) r[i] = lo->second[i] + (hi->second[i] - lo->second[i]) * f;
        return r;
    }
};

struct ParamSet {
    PropSet props;
    std::vector<std::unique_ptr<Param>> params;
    Param* find(const std::string& n)
    {
        for (auto& p : params)
            if (p->name == n) return p.get();
        return nullptr;
    }
};
ParamSet* PSet(OfxParamSetHandle h) { return reinterpret_cast<ParamSet*>(h); }
Param* PR(OfxParamHandle h) { return reinterpret_cast<Param*>(h); }

int gEditBlocks = 0;
// Like the host's undo: every outermost edit block is one undo step that restores the parameters as
// they were when it began. Changes outside a block are not recorded.
struct ParamState {
    Param* p;
    std::vector<double> value;
    std::string sval;
    std::map<double, std::vector<double>> keys;
};
std::vector<ParamState> gBlockStart;
std::vector<std::vector<ParamState>> gUndo;
bool gInOverlay = false;              // inside a viewer event (pen / key)
std::vector<std::string> gUnblocked;  // pin settings the viewer changed outside an edit block
void noteChange(const Param* p)
{
    if (gInOverlay && gEditBlocks == 0 && p->name.compare(0, 8, "selected") != 0) gUnblocked.push_back(p->name);
}
bool undo()
{
    if (gUndo.empty()) return false;
    for (const ParamState& st : gUndo.back()) {
        st.p->value = st.value;
        st.p->sval = st.sval;
        st.p->keys = st.keys;
    }
    gUndo.pop_back();
    return true;
}

OfxStatus paramDefine(OfxParamSetHandle ps, const char* type, const char* name, OfxPropertySetHandle* props)
{
    auto p = std::make_unique<Param>();
    p->name = name;
    p->type = type;
    p->props.setS(kOfxPropName, name);
    p->props.setS(kOfxParamPropType, type);
    if (props) *props = H(&p->props);
    PSet(ps)->params.push_back(std::move(p));
    return kOfxStatOK;
}
OfxStatus paramGetHandle(OfxParamSetHandle ps, const char* name, OfxParamHandle* param, OfxPropertySetHandle* props)
{
    Param* p = PSet(ps)->find(name);
    if (!p) {
        std::printf("  host: unknown param '%s'\n", name);
        return kOfxStatErrUnknown;
    }
    *param = reinterpret_cast<OfxParamHandle>(p);
    if (props) *props = H(&p->props);
    return kOfxStatOK;
}
OfxStatus paramSetGetPropertySet(OfxParamSetHandle ps, OfxPropertySetHandle* props) { *props = H(&PSet(ps)->props); return kOfxStatOK; }
OfxStatus paramGetPropertySet(OfxParamHandle p, OfxPropertySetHandle* props) { *props = H(&PR(p)->props); return kOfxStatOK; }

void readOut(Param* p, const std::vector<double>& v, va_list ap)
{
    if (p->type == kOfxParamTypeString) {
        *va_arg(ap, const char**) = p->sval.c_str();
        return;
    }
    for (int i = 0; i < p->dims(); ++i) {
        const double x = i < (int)v.size() ? v[i] : 0.0;
        if (p->isInt())
            *va_arg(ap, int*) = (int)std::lround(x);
        else
            *va_arg(ap, double*) = x;
    }
}
std::vector<double> readIn(Param* p, va_list ap)
{
    std::vector<double> v(p->dims());
    for (int i = 0; i < p->dims(); ++i) v[i] = p->isInt() ? (double)va_arg(ap, int) : va_arg(ap, double);
    return v;
}
OfxStatus paramGetValue(OfxParamHandle h, ...)
{
    va_list ap;
    va_start(ap, h);
    readOut(PR(h), PR(h)->at(gTime), ap);
    va_end(ap);
    return kOfxStatOK;
}
OfxStatus paramGetValueAtTime(OfxParamHandle h, OfxTime t, ...)
{
    va_list ap;
    va_start(ap, t);
    readOut(PR(h), PR(h)->at(t), ap);
    va_end(ap);
    return kOfxStatOK;
}
OfxStatus paramGetDerivative(OfxParamHandle h, OfxTime t, ...)
{
    va_list ap;
    va_start(ap, t);
    readOut(PR(h), std::vector<double>(4, 0.0), ap);
    va_end(ap);
    return kOfxStatOK;
}
OfxStatus paramGetIntegral(OfxParamHandle h, OfxTime, OfxTime, ...) { return kOfxStatErrUnsupported; }
OfxStatus paramSetValue(OfxParamHandle h, ...)
{
    va_list ap;
    va_start(ap, h);
    Param* p = PR(h);
    noteChange(p);
    if (p->type == kOfxParamTypeString) {
        p->sval = va_arg(ap, const char*);
    } else {
        const auto v = readIn(p, ap);
        if (!p->keys.empty()) p->keys[gTime] = v;
        else p->value = v;
    }
    va_end(ap);
    return kOfxStatOK;
}
OfxStatus paramSetValueAtTime(OfxParamHandle h, OfxTime t, ...)
{
    va_list ap;
    va_start(ap, t);
    Param* p = PR(h);
    noteChange(p);
    if (p->type != kOfxParamTypeString) p->keys[t] = readIn(p, ap);
    va_end(ap);
    return kOfxStatOK;
}
OfxStatus paramGetNumKeys(OfxParamHandle h, unsigned int* n) { *n = (unsigned)PR(h)->keys.size(); return kOfxStatOK; }
OfxStatus paramGetKeyTime(OfxParamHandle h, unsigned int nth, OfxTime* t)
{
    auto it = PR(h)->keys.begin();
    if (nth >= PR(h)->keys.size()) return kOfxStatErrBadIndex;
    std::advance(it, nth);
    *t = it->first;
    return kOfxStatOK;
}
OfxStatus paramGetKeyIndex(OfxParamHandle h, OfxTime t, int dir, int* idx)
{
    int i = 0;
    for (auto& k : PR(h)->keys) {
        if ((dir == 0 && k.first == t) || (dir > 0 && k.first > t)) { *idx = i; return kOfxStatOK; }
        ++i;
    }
    return kOfxStatFailed;
}
OfxStatus paramDeleteKey(OfxParamHandle h, OfxTime t) { PR(h)->keys.erase(t); return kOfxStatOK; }
OfxStatus paramDeleteAllKeys(OfxParamHandle h)
{
    Param* p = PR(h);
    if (!p->keys.empty()) p->value = p->at(gTime);
    p->keys.clear();
    return kOfxStatOK;
}
OfxStatus paramCopy(OfxParamHandle, OfxParamHandle, OfxTime, const OfxRangeD*) { return kOfxStatOK; }
OfxStatus paramEditBegin(OfxParamSetHandle ps, const char*)
{
    if (gEditBlocks++ == 0) {
        gBlockStart.clear();
        for (auto& q : PSet(ps)->params) gBlockStart.push_back({q.get(), q->value, q->sval, q->keys});
    }
    return kOfxStatOK;
}
OfxStatus paramEditEnd(OfxParamSetHandle)
{
    if (--gEditBlocks == 0) gUndo.push_back(std::move(gBlockStart));
    return kOfxStatOK;
}
OfxParameterSuiteV1 gParamSuite = {paramDefine, paramGetHandle, paramSetGetPropertySet, paramGetPropertySet,
                                   paramGetValue, paramGetValueAtTime, paramGetDerivative, paramGetIntegral,
                                   paramSetValue, paramSetValueAtTime, paramGetNumKeys, paramGetKeyTime,
                                   paramGetKeyIndex, paramDeleteKey, paramDeleteAllKeys, paramCopy,
                                   paramEditBegin, paramEditEnd};

// ------------------------------------------------------------------------------------
// Image effect.

struct Clip {
    std::string name;
    PropSet props;
    ppt::Image* image = nullptr;  // pixels served for this clip
    int ox = 0, oy = 0;           // where its lower-left pixel is
};

struct Effect {
    PropSet props;
    ParamSet params;
    std::vector<std::unique_ptr<Clip>> clips;
    Clip* clip(const std::string& n)
    {
        for (auto& c : clips)
            if (c->name == n) return c.get();
        return nullptr;
    }
};
Effect* EF(OfxImageEffectHandle h) { return reinterpret_cast<Effect*>(h); }

struct ImageRecord {
    PropSet props;
};

OfxStatus getPropertySet(OfxImageEffectHandle e, OfxPropertySetHandle* p) { *p = H(&EF(e)->props); return kOfxStatOK; }
OfxStatus getParamSet(OfxImageEffectHandle e, OfxParamSetHandle* p) { *p = reinterpret_cast<OfxParamSetHandle>(&EF(e)->params); return kOfxStatOK; }
OfxStatus clipDefine(OfxImageEffectHandle e, const char* name, OfxPropertySetHandle* p)
{
    auto c = std::make_unique<Clip>();
    c->name = name;
    c->props.setS(kOfxPropName, name);
    *p = H(&c->props);
    EF(e)->clips.push_back(std::move(c));
    return kOfxStatOK;
}
OfxStatus clipGetHandle(OfxImageEffectHandle e, const char* name, OfxImageClipHandle* clip, OfxPropertySetHandle* p)
{
    Clip* c = EF(e)->clip(name);
    if (!c) return kOfxStatErrBadIndex;
    *clip = reinterpret_cast<OfxImageClipHandle>(c);
    if (p) *p = H(&c->props);
    return kOfxStatOK;
}
OfxStatus clipGetPropertySet(OfxImageClipHandle c, OfxPropertySetHandle* p) { *p = H(&reinterpret_cast<Clip*>(c)->props); return kOfxStatOK; }
OfxStatus clipGetImage(OfxImageClipHandle ch, OfxTime, const OfxRectD*, OfxPropertySetHandle* out)
{
    Clip* c = reinterpret_cast<Clip*>(ch);
    if (!c->image) return kOfxStatFailed;
    auto* rec = new ImageRecord;
    ppt::Image& img = *c->image;
    rec->props.setP(kOfxImagePropData, img.px.data());
    rec->props.setI(kOfxImagePropBounds, {c->ox, c->oy, c->ox + img.w, c->oy + img.h});
    rec->props.setI(kOfxImagePropRegionOfDefinition, {c->ox, c->oy, c->ox + img.w, c->oy + img.h});
    rec->props.setI(kOfxImagePropRowBytes, {img.w * 4 * (int)sizeof(float)});
    rec->props.setS(kOfxImageEffectPropPixelDepth, kOfxBitDepthFloat);
    rec->props.setS(kOfxImageEffectPropComponents, kOfxImageComponentRGBA);
    rec->props.setS(kOfxImageEffectPropPreMultiplication, kOfxImagePreMultiplied);
    rec->props.setD(kOfxImageEffectPropRenderScale, {1.0, 1.0});
    rec->props.setD(kOfxImagePropPixelAspectRatio, {1.0});
    rec->props.setS(kOfxImagePropField, kOfxImageFieldNone);
    rec->props.setS(kOfxImagePropUniqueIdentifier, c->name);
    *out = H(&rec->props);
    return kOfxStatOK;
}
OfxStatus clipReleaseImage(OfxPropertySetHandle h) { delete reinterpret_cast<ImageRecord*>(h); return kOfxStatOK; }
int gW = 960, gH = 540;
OfxStatus clipGetRegionOfDefinition(OfxImageClipHandle ch, OfxTime, OfxRectD* r)
{
    const Clip* c = reinterpret_cast<Clip*>(ch);
    if (c->image && c->name != kOfxImageEffectOutputClipName)
        *r = {(double)c->ox, (double)c->oy, (double)(c->ox + c->image->w), (double)(c->oy + c->image->h)};
    else
        *r = {0, 0, (double)gW, (double)gH};
    return kOfxStatOK;
}
int effectAbort(OfxImageEffectHandle) { return 0; }
OfxStatus imageMemoryAlloc(OfxImageEffectHandle, size_t n, OfxImageMemoryHandle* m) { *m = reinterpret_cast<OfxImageMemoryHandle>(std::malloc(n)); return kOfxStatOK; }
OfxStatus imageMemoryFree(OfxImageMemoryHandle m) { std::free(m); return kOfxStatOK; }
OfxStatus imageMemoryLock(OfxImageMemoryHandle m, void** p) { *p = m; return kOfxStatOK; }
OfxStatus imageMemoryUnlock(OfxImageMemoryHandle) { return kOfxStatOK; }
OfxImageEffectSuiteV1 gEffectSuite = {getPropertySet, getParamSet, clipDefine, clipGetHandle, clipGetPropertySet,
                                      clipGetImage, clipReleaseImage, clipGetRegionOfDefinition, effectAbort,
                                      imageMemoryAlloc, imageMemoryFree, imageMemoryLock, imageMemoryUnlock};

// ------------------------------------------------------------------------------------
// Memory, threads, messages, interacts.

OfxStatus memoryAlloc(void*, size_t n, void** p) { *p = std::malloc(n); return kOfxStatOK; }
OfxStatus memoryFree(void* p) { std::free(p); return kOfxStatOK; }
OfxMemorySuiteV1 gMemorySuite = {memoryAlloc, memoryFree};

OfxStatus multiThread(OfxThreadFunctionV1 f, unsigned int n, void* arg)
{
    if (n <= 1) { f(0, 1, arg); return kOfxStatOK; }
    std::vector<std::thread> t;
    for (unsigned i = 0; i < n; ++i) t.emplace_back([=] { f(i, n, arg); });
    for (auto& x : t) x.join();
    return kOfxStatOK;
}
OfxStatus multiThreadNumCPUs(unsigned int* n) { *n = std::max(1u, std::thread::hardware_concurrency()); return kOfxStatOK; }
OfxStatus multiThreadIndex(unsigned int* i) { *i = 0; return kOfxStatOK; }
int multiThreadIsSpawnedThread() { return 0; }
OfxStatus mutexCreate(OfxMutexHandle* m, int) { *m = reinterpret_cast<OfxMutexHandle>(new std::recursive_mutex); return kOfxStatOK; }
OfxStatus mutexDestroy(const OfxMutexHandle m) { delete reinterpret_cast<std::recursive_mutex*>(m); return kOfxStatOK; }
OfxStatus mutexLock(const OfxMutexHandle m) { reinterpret_cast<std::recursive_mutex*>(m)->lock(); return kOfxStatOK; }
OfxStatus mutexUnLock(const OfxMutexHandle m) { reinterpret_cast<std::recursive_mutex*>(m)->unlock(); return kOfxStatOK; }
OfxStatus mutexTryLock(const OfxMutexHandle m) { return reinterpret_cast<std::recursive_mutex*>(m)->try_lock() ? kOfxStatOK : kOfxStatFailed; }
OfxMultiThreadSuiteV1 gThreadSuite = {multiThread, multiThreadNumCPUs, multiThreadIndex, multiThreadIsSpawnedThread,
                                      mutexCreate, mutexDestroy, mutexLock, mutexUnLock, mutexTryLock};

std::vector<std::string> gMessages;
OfxStatus message(void*, const char* type, const char*, const char* fmt, ...)
{
    char buf[1024];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    gMessages.push_back(buf);
    std::printf("  plugin message (%s): %s\n", type, buf);
    return kOfxStatOK;
}
OfxMessageSuiteV1 gMessageSuite = {message};
std::string gPersistent;
OfxStatus setPersistent(void*, const char*, const char*, const char* fmt, ...)
{
    char buf[1024];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    gPersistent = buf;
    return kOfxStatOK;
}
OfxStatus clearPersistent(void*)
{
    gPersistent.clear();
    return kOfxStatOK;
}
OfxMessageSuiteV2 gMessageSuite2 = {message, setPersistent, clearPersistent};

struct InteractInst {
    PropSet props;
};
int gRedraws = 0;
OfxStatus interactSwapBuffers(OfxInteractHandle) { return kOfxStatOK; }
OfxStatus interactRedraw(OfxInteractHandle) { ++gRedraws; return kOfxStatOK; }
OfxStatus interactGetPropertySet(OfxInteractHandle h, OfxPropertySetHandle* p) { *p = H(&reinterpret_cast<InteractInst*>(h)->props); return kOfxStatOK; }
OfxInteractSuiteV1 gInteractSuite = {interactSwapBuffers, interactRedraw, interactGetPropertySet};

// Draw suite: rasterises the overlay into an image so it can be inspected.
struct Canvas {
    ppt::Image* img = nullptr;
    float rgba[4] = {1, 1, 1, 1};
    int lines = 0, polys = 0, texts = 0, loops = 0, rects = 0;
    void plot(int x, int y)
    {
        if (!img || x < 0 || y < 0 || x >= img->w || y >= img->h) return;
        float* o = img->at(x, y);
        const float a = rgba[3];
        for (int c = 0; c < 3; ++c) o[c] = rgba[c] * a + o[c] * (1 - a);
        o[3] = a + o[3] * (1 - a);
    }
    void line(OfxPointD a, OfxPointD b)
    {
        const double n = std::max(1.0, std::ceil(std::max(std::fabs(b.x - a.x), std::fabs(b.y - a.y))));
        if (n > 5000) return;
        for (int i = 0; i <= (int)n; ++i)
            plot((int)std::floor(a.x + (b.x - a.x) * i / n), (int)std::floor(a.y + (b.y - a.y) * i / n));
    }
};
Canvas gCanvas;
OfxStatus drawGetColour(OfxDrawContextHandle, OfxStandardColour, OfxRGBAColourF* c) { *c = {1, 1, 1, 1}; return kOfxStatOK; }
OfxStatus drawSetColour(OfxDrawContextHandle, const OfxRGBAColourF* c)
{
    gCanvas.rgba[0] = c->r; gCanvas.rgba[1] = c->g; gCanvas.rgba[2] = c->b; gCanvas.rgba[3] = c->a;
    return kOfxStatOK;
}
OfxStatus drawSetLineWidth(OfxDrawContextHandle, float) { return kOfxStatOK; }
OfxStatus drawSetLineStipple(OfxDrawContextHandle, OfxDrawLineStipplePattern) { return kOfxStatOK; }
OfxStatus drawDraw(OfxDrawContextHandle, OfxDrawPrimitive prim, const OfxPointD* p, int n)
{
    switch (prim) {
        case kOfxDrawPrimitiveLines:
            gCanvas.lines += n / 2;
            for (int i = 0; i + 1 < n; i += 2) gCanvas.line(p[i], p[i + 1]);
            break;
        case kOfxDrawPrimitiveLineStrip:
            for (int i = 0; i + 1 < n; ++i) gCanvas.line(p[i], p[i + 1]);
            break;
        case kOfxDrawPrimitiveRectangle:
            ++gCanvas.rects;
            if (n >= 2)
                for (int y = (int)std::floor(std::min(p[0].y, p[1].y)); y <= (int)std::ceil(std::max(p[0].y, p[1].y)); ++y)
                    for (int x = (int)std::floor(std::min(p[0].x, p[1].x)); x <= (int)std::ceil(std::max(p[0].x, p[1].x)); ++x)
                        gCanvas.plot(x, y);
            break;
        case kOfxDrawPrimitiveLineLoop:
            ++gCanvas.loops;
            for (int i = 0; i < n; ++i) gCanvas.line(p[i], p[(i + 1) % n]);
            break;
        case kOfxDrawPrimitivePolygon: {
            ++gCanvas.polys;
            double x0 = 1e9, y0 = 1e9, x1 = -1e9, y1 = -1e9;
            for (int i = 0; i < n; ++i) { x0 = std::min(x0, p[i].x); x1 = std::max(x1, p[i].x); y0 = std::min(y0, p[i].y); y1 = std::max(y1, p[i].y); }
            for (int y = (int)std::floor(y0); y <= (int)std::ceil(y1); ++y)
                for (int x = (int)std::floor(x0); x <= (int)std::ceil(x1); ++x) {
                    bool in = false;
                    for (int i = 0, j = n - 1; i < n; j = i++)
                        if ((p[i].y > y + 0.5) != (p[j].y > y + 0.5) &&
                            x + 0.5 < (p[j].x - p[i].x) * (y + 0.5 - p[i].y) / (p[j].y - p[i].y) + p[i].x)
                            in = !in;
                    if (in) gCanvas.plot(x, y);
                }
            break;
        }
        default: break;
    }
    return kOfxStatOK;
}
OfxStatus drawText(OfxDrawContextHandle, const char*, const OfxPointD* p, int)
{
    ++gCanvas.texts;
    for (int y = 0; y < 6; ++y)
        for (int x = 0; x < 4; ++x) gCanvas.plot((int)p->x + x, (int)p->y + y);
    return kOfxStatOK;
}
OfxDrawSuiteV1 gDrawSuite = {drawGetColour, drawSetColour, drawSetLineWidth, drawSetLineStipple, drawDraw, drawText};

// ------------------------------------------------------------------------------------
// Host.

PropSet gHostProps;
const void* fetchSuite(OfxPropertySetHandle, const char* name, int version)
{
    const std::string n(name);
    if (n == kOfxPropertySuite && version == 1) return &gPropSuite;
    if (n == kOfxImageEffectSuite && version == 1) return &gEffectSuite;
    if (n == kOfxParameterSuite && version == 1) return &gParamSuite;
    if (n == kOfxMemorySuite && version == 1) return &gMemorySuite;
    if (n == kOfxMultiThreadSuite && version == 1) return &gThreadSuite;
    if (n == kOfxMessageSuite && version == 1) return &gMessageSuite;
    if (n == kOfxMessageSuite && version == 2) return &gMessageSuite2;
    if (n == kOfxInteractSuite && version == 1) return &gInteractSuite;
    if (n == kOfxDrawSuite && version == 1) return &gDrawSuite;
    return nullptr;
}
OfxHost gHost = {nullptr, fetchSuite};

OfxPlugin* gPlugin = nullptr;
OfxStatus call(const char* action, const void* handle, PropSet* in = nullptr, PropSet* out = nullptr)
{
    return gPlugin->mainEntry(action, handle, in ? H(in) : nullptr, out ? H(out) : nullptr);
}

// ------------------------------------------------------------------------------------

Effect* gInstance = nullptr;
OfxPluginEntryPoint* gOverlayMain = nullptr;
InteractInst gInteract;

Param* param(const std::string& n)
{
    Param* p = gInstance->params.find(n);
    if (!p) {
        std::printf("  FAIL: missing param %s\n", n.c_str());
        ++gFailures;
        static Param dummy;
        dummy.value.assign(4, 0.0);
        return &dummy;
    }
    return p;
}

void fillPenArgs(PropSet& in, double x, double y)
{
    in.setP(kOfxPropEffectInstance, gInstance);
    in.setD(kOfxPropTime, {gTime});
    in.setD(kOfxImageEffectPropRenderScale, {1.0, 1.0});
    in.setD(kOfxInteractPropPixelScale, {1.0, 1.0});
    in.setD(kOfxInteractPropBackgroundColour, {0, 0, 0});
    in.setD(kOfxInteractPropPenPosition, {x, y});
    in.setI(kOfxInteractPropPenViewportPosition, {(int)x, (int)y});
    in.setD(kOfxInteractPropPenPressure, {1.0});
}

OfxStatus pen(const char* action, double x, double y)
{
    PropSet in;
    fillPenArgs(in, x, y);
    gInOverlay = true;
    const OfxStatus st = gOverlayMain(action, &gInteract, H(&in), nullptr);
    gInOverlay = false;
    return st;
}

void drag(double x0, double y0, double x1, double y1)
{
    CHECK(pen(kOfxInteractActionPenDown, x0, y0) == kOfxStatOK, "penDown not handled at %.0f %.0f", x0, y0);
    for (int i = 1; i <= 5; ++i) pen(kOfxInteractActionPenMotion, x0 + (x1 - x0) * i / 5, y0 + (y1 - y0) * i / 5);
    pen(kOfxInteractActionPenUp, x1, y1);
}

// A double-click on the image adds a pin (the default "Add Pins By"), then it is dragged.
void addPinAt(double x0, double y0, double x1, double y1)
{
    pen(kOfxInteractActionPenDown, x0, y0);
    pen(kOfxInteractActionPenUp, x0, y0);
    drag(x0, y0, x1, y1);
}
void addPinAt(double x, double y) { addPinAt(x, y, x, y); }

bool renderFrame(ppt::Image& out, bool* identity = nullptr)
{
    PropSet in, outArgs;
    in.setD(kOfxPropTime, {gTime});
    in.setS(kOfxImageEffectPropFieldToRender, kOfxImageFieldNone);
    in.setI(kOfxImageEffectPropRenderWindow, {0, 0, gW, gH});
    in.setD(kOfxImageEffectPropRenderScale, {1.0, 1.0});
    if (identity) {
        const OfxStatus st = call(kOfxImageEffectActionIsIdentity, gInstance, &in, &outArgs);
        *identity = st == kOfxStatOK;
    }
    out = ppt::Image(gW, gH);
    gInstance->clip(kOfxImageEffectOutputClipName)->image = &out;
    const OfxStatus st = call(kOfxImageEffectActionRender, gInstance, &in, nullptr);
    gInstance->clip(kOfxImageEffectOutputClipName)->image = nullptr;
    return st == kOfxStatOK;
}

// Renders the window x1,y1 .. x2,y2 (which may reach past the frame) into `out`.
bool renderWindow(ppt::Image& out, int x1, int y1, int x2, int y2)
{
    PropSet in;
    in.setD(kOfxPropTime, {gTime});
    in.setS(kOfxImageEffectPropFieldToRender, kOfxImageFieldNone);
    in.setI(kOfxImageEffectPropRenderWindow, {x1, y1, x2, y2});
    in.setD(kOfxImageEffectPropRenderScale, {1.0, 1.0});
    out = ppt::Image(x2 - x1, y2 - y1);
    Clip* o = gInstance->clip(kOfxImageEffectOutputClipName);
    o->image = &out;
    o->ox = x1;
    o->oy = y1;
    const OfxStatus st = call(kOfxImageEffectActionRender, gInstance, &in, nullptr);
    o->image = nullptr;
    o->ox = o->oy = 0;
    return st == kOfxStatOK;
}

// The plugin's region of definition (false: it left it to the host).
bool regionOfDefinition(OfxRectD& r)
{
    PropSet in, out;
    in.setD(kOfxPropTime, {gTime});
    in.setD(kOfxImageEffectPropRenderScale, {1.0, 1.0});
    out.setD(kOfxImageEffectPropRegionOfDefinition, {0, 0, 0, 0});
    if (call(kOfxImageEffectActionGetRegionOfDefinition, gInstance, &in, &out) != kOfxStatOK) return false;
    const auto& v = out[kOfxImageEffectPropRegionOfDefinition].d;
    r = {v[0], v[1], v[2], v[3]};
    return true;
}

void drawOverlay(ppt::Image& canvas)
{
    PropSet in;
    in.setP(kOfxPropEffectInstance, gInstance);
    in.setP(kOfxInteractPropDrawContext, (void*)0x1);
    in.setD(kOfxInteractPropPixelScale, {1.0, 1.0});
    in.setD(kOfxInteractPropBackgroundColour, {0, 0, 0});
    in.setD(kOfxPropTime, {gTime});
    in.setD(kOfxImageEffectPropRenderScale, {1.0, 1.0});
    gCanvas = Canvas();
    gCanvas.img = &canvas;
    gOverlayMain(kOfxInteractActionDraw, &gInteract, H(&in), nullptr);
}

void pressButton(const char* name)
{
    PropSet in;
    in.setS(kOfxPropType, kOfxTypeParameter);
    in.setS(kOfxPropName, name);
    in.setS(kOfxPropChangeReason, kOfxChangeUserEdited);
    in.setD(kOfxPropTime, {gTime});
    in.setD(kOfxImageEffectPropRenderScale, {1.0, 1.0});
    call(kOfxActionInstanceChanged, gInstance, &in, nullptr);
}

// The host telling the plugin a parameter changed (e.g. Fusion's own path handle moved a pin).
void notifyChanged(const char* name, const char* reason)
{
    PropSet in;
    in.setS(kOfxPropType, kOfxTypeParameter);
    in.setS(kOfxPropName, name);
    in.setS(kOfxPropChangeReason, reason);
    in.setD(kOfxPropTime, {gTime});
    in.setD(kOfxImageEffectPropRenderScale, {1.0, 1.0});
    call(kOfxActionInstanceChanged, gInstance, &in, nullptr);
}

// A key press / release in the viewer.
OfxStatus keyEvent(const char* action, int sym)
{
    PropSet in;
    in.setP(kOfxPropEffectInstance, gInstance);
    in.setD(kOfxPropTime, {gTime});
    in.setD(kOfxImageEffectPropRenderScale, {1.0, 1.0});
    in.setI(kOfxPropKeySym, {sym});
    in.setS(kOfxPropKeyString, "");
    gInOverlay = true;
    const OfxStatus st = gOverlayMain(action, &gInteract, H(&in), nullptr);
    gInOverlay = false;
    return st;
}
OfxStatus keyDown(int sym) { return keyEvent(kOfxInteractActionKeyDown, sym); }
OfxStatus keyUp(int sym) { return keyEvent(kOfxInteractActionKeyUp, sym); }

// A pin's position in pixels.
std::vector<double> pinAt(int n, double t = 0.0)
{
    const auto v = param("pin" + std::to_string(n) + "_pos")->at(t);
    return {v[0] * gW, v[1] * gH};
}

std::vector<double> val(const std::string& n, double t) { return param(n)->at(t); }

}  // namespace

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::printf("usage: ofx_host_test <PuppetPin.ofx> [outdir]\n");
        return 2;
    }
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    const std::string outDir = argc > 2 ? argv[2] : ".";
    gHost.host = H(&gHostProps);
    gHostProps.setS(kOfxPropName, "PuppetPinTestHost");
    gHostProps.setI(kOfxImageEffectPropSupportsOverlays, {1});
    gHostProps.setI(kOfxImageEffectPropSupportsMultipleClipDepths, {0});
    gHostProps.setS(kOfxImageEffectPropSupportedComponents, kOfxImageComponentRGBA);
    gHostProps.setI(kOfxParamHostPropMaxParameters, {-1});
    gHostProps.setI(kOfxParamHostPropMaxPages, {0});
    gHostProps["OfxImageEffectPropSupportedContexts"].s = {kOfxImageEffectContextFilter, kOfxImageEffectContextGeneral};

    HMODULE dll = LoadLibraryA(argv[1]);
    if (!dll) {
        std::printf("FAIL: cannot load %s (error %lu)\n", argv[1], GetLastError());
        return 1;
    }
    auto getNum = reinterpret_cast<int (*)()>(GetProcAddress(dll, "OfxGetNumberOfPlugins"));
    auto getPlugin = reinterpret_cast<OfxPlugin* (*)(int)>(GetProcAddress(dll, "OfxGetPlugin"));
    if (!getNum || !getPlugin) {
        std::printf("FAIL: missing OFX entry points\n");
        return 1;
    }
    std::printf("[load] %d plugin(s)\n", getNum());
    gPlugin = getPlugin(0);
    std::printf("  id %s v%u.%u api %s %d\n", gPlugin->pluginIdentifier, gPlugin->pluginVersionMajor,
                gPlugin->pluginVersionMinor, gPlugin->pluginApi, gPlugin->apiVersion);
    gPlugin->setHost(&gHost);
    CHECK(call(kOfxActionLoad, nullptr) == kOfxStatOK, "load action failed");

    // Describe.
    Effect desc;
    CHECK(call(kOfxActionDescribe, &desc) == kOfxStatOK, "describe failed");
    std::printf("[describe] label '%s' grouping '%s'\n", desc.props.getS(kOfxPropLabel).c_str(),
                desc.props.getS(kOfxImageEffectPluginPropGrouping).c_str());
    gOverlayMain = reinterpret_cast<OfxPluginEntryPoint*>(desc.props.getP(kOfxImageEffectPluginPropOverlayInteractV2));
    if (!gOverlayMain) gOverlayMain = reinterpret_cast<OfxPluginEntryPoint*>(desc.props.getP(kOfxImageEffectPluginPropOverlayInteractV1));
    CHECK(gOverlayMain != nullptr, "no overlay interact");

    Effect ctxDesc;
    ctxDesc.props = desc.props;
    PropSet ctxArgs;
    ctxArgs.setS(kOfxImageEffectPropContext, kOfxImageEffectContextFilter);
    CHECK(call(kOfxImageEffectActionDescribeInContext, &ctxDesc, &ctxArgs) == kOfxStatOK, "describeInContext failed");
    std::printf("[describeInContext] %zu params, %zu clips\n", ctxDesc.params.params.size(), ctxDesc.clips.size());
    {
        // The host must re-query the clip preferences (frame varying) when these change.
        const auto& slaves = ctxDesc.props[kOfxImageEffectPropClipPreferencesSlaveParam].s;
        auto has = [&](const char* n) { return std::find(slaves.begin(), slaves.end(), n) != slaves.end(); };
        CHECK(has("solverMode") && has("def1_type") && has("def8_type"), "clip preference slave params missing");
    }

    // Instance.
    ppt::Image src = ppt::makeCharacter(gW, gH);
    Effect inst;
    gInstance = &inst;
    inst.props = ctxDesc.props;
    inst.props.setS(kOfxImageEffectPropContext, kOfxImageEffectContextFilter);
    inst.props.setS(kOfxPropType, kOfxTypeImageEffectInstance);
    inst.props.setD(kOfxImageEffectPropProjectSize, {(double)gW, (double)gH});
    inst.props.setD(kOfxImageEffectPropProjectExtent, {(double)gW, (double)gH});
    inst.props.setD(kOfxImageEffectPropProjectOffset, {0.0, 0.0});
    inst.props.setD(kOfxImageEffectPropProjectPixelAspectRatio, {1.0});
    inst.props.setD(kOfxImageEffectInstancePropEffectDuration, {100.0});
    inst.props.setD(kOfxImageEffectPropFrameRate, {24.0});
    inst.props.setI(kOfxPropIsInteractive, {1});
    for (auto& p : ctxDesc.params.params) {
        auto q = std::make_unique<Param>(*p);
        const Prop& def = q->props[kOfxParamPropDefault];
        q->value.assign(q->dims(), 0.0);
        if (q->type == kOfxParamTypeString && !def.s.empty()) q->sval = def.s[0];
        for (int i = 0; i < q->dims(); ++i) {
            if (i < (int)def.d.size()) q->value[i] = def.d[i];
            else if (i < (int)def.i.size()) q->value[i] = def.i[i];
        }
        inst.params.params.push_back(std::move(q));
    }
    for (auto& c : ctxDesc.clips) {
        auto k = std::make_unique<Clip>(*c);
        // Only the Source and the Output are connected; optional inputs (weight map, layers) start empty.
        k->props.setI(kOfxImageClipPropConnected, {k->name == kOfxImageEffectSimpleSourceClipName || k->name == kOfxImageEffectOutputClipName ? 1 : 0});
        k->props.setS(kOfxImageEffectPropPixelDepth, kOfxBitDepthFloat);
        k->props.setS(kOfxImageEffectPropComponents, kOfxImageComponentRGBA);
        k->props.setS(kOfxImageClipPropUnmappedPixelDepth, kOfxBitDepthFloat);
        k->props.setS(kOfxImageClipPropUnmappedComponents, kOfxImageComponentRGBA);
        k->props.setS(kOfxImageEffectPropPreMultiplication, kOfxImagePreMultiplied);
        k->props.setD(kOfxImagePropPixelAspectRatio, {1.0});
        k->props.setD(kOfxImageEffectPropFrameRate, {24.0});
        k->props.setD(kOfxImageEffectPropFrameRange, {0.0, 100.0});
        k->props.setS(kOfxImageClipPropFieldOrder, kOfxImageFieldNone);
        if (k->name == kOfxImageEffectSimpleSourceClipName) k->image = &src;
        inst.clips.push_back(std::move(k));
    }
    CHECK(call(kOfxActionCreateInstance, &inst) == kOfxStatOK, "createInstance failed");
    std::printf("[instance] created\n");
    CHECK(param("pin1_group")->props["OfxParamPropSecret"].i.size() && param("pin1_group")->props["OfxParamPropSecret"].i[0] == 1,
          "unused pin groups should be hidden");

    // Overlay instance.
    PropSet idesc;
    {
        const OfxStatus st = gOverlayMain(kOfxActionDescribe, reinterpret_cast<OfxInteractHandle>(&idesc), nullptr, nullptr);
        CHECK(st == kOfxStatOK || st == kOfxStatReplyDefault, "overlay describe failed (%d)", st);
    }
    gInteract.props.setP(kOfxPropEffectInstance, &inst);
    gInteract.props.setD(kOfxInteractPropPixelScale, {1.0, 1.0});
    gInteract.props.setD(kOfxInteractPropBackgroundColour, {0, 0, 0});
    CHECK(gOverlayMain(kOfxActionCreateInstance, &gInteract, nullptr, nullptr) == kOfxStatOK, "overlay create failed");

    // Frame 0 without pins: an identity, unless the mesh is shown (the viewer's mesh comes from
    // rendering, so Triangles / Expansion would not update before the first pin otherwise).
    ppt::Image out;
    bool ident = true;
    CHECK(renderFrame(out, &ident), "render without pins failed");
    CHECK(!ident, "with Show Mesh on, no pins must still render (to rebuild the mesh)");
    {
        drawOverlay(out);
        const int linesBefore = gCanvas.lines;
        param("triangles")->value = {2500};
        param("expansion")->value = {12.0};
        renderFrame(out, &ident);
        drawOverlay(out);
        std::printf("[no pins] mesh lines %d -> %d after raising Triangles\n", linesBefore, gCanvas.lines);
        CHECK(gCanvas.lines > linesBefore * 2, "changing Triangles did not change the viewer mesh");
        param("triangles")->value = {800};
        param("expansion")->value = {3.0};
    }
    param("showMesh")->value = {0};
    renderFrame(out, &ident);
    std::printf("[no pins] identity with Show Mesh off=%d\n", ident);
    CHECK(ident, "no pins should be an identity");
    param("showMesh")->value = {1};
    renderFrame(out);
    double maxDiff = 0;
    for (size_t i = 0; i < out.px.size(); ++i) maxDiff = std::max(maxDiff, (double)std::fabs(out.px[i] - src.px[i]));
    CHECK(maxDiff == 0.0, "render without pins changed the image (%g)", maxDiff);

    {
        // Keyframes When Dragging, default "Only Keyframed Pins": a pin without keyframes just moves;
        // once it has a keyframe, every drag on another frame adds one.
        std::printf("[keyframes]\n");
        gTime = 5;
        addPinAt(576, 222, 600, 240);
        const size_t keysUnanimated = param("pin1_pos")->keys.size();
        const auto moved = param("pin1_pos")->value;
        param("pin1_pos")->keys[5.0] = moved;  // the user sets a first keyframe (keyframe button)
        gTime = 10;
        drag(600, 240, 620, 260);
        const size_t keysAnimated = param("pin1_pos")->keys.size();
        std::printf("  keys after dragging an unkeyed pin: %zu, after keying it and dragging on frame 10: %zu\n",
                    keysUnanimated, keysAnimated);
        CHECK(keysUnanimated == 0 && std::fabs(moved[0] * gW - 600) < 1e-6,
              "dragging a pin without keyframes must not set one");
        CHECK(keysAnimated == 2, "dragging a keyframed pin should add a keyframe");
        gTime = 0;
        pressButton("removeAll");
    }
    // The rest of the test keys pins by dragging them, like with "Every Pin".
    param("keyMode")->value = {1};

    // Click pins onto the character and pose the arm, like a user would.
    std::printf("[overlay] adding pins\n");
    addPinAt(440, 52);                  // left foot
    addPinAt(520, 52);                  // right foot
    addPinAt(480, 200);                 // hips
    addPinAt(480, 335, 470, 335);       // neck, nudged
    addPinAt(576, 222, 650, 430);       // hand, raised
    CHECK(gEditBlocks == 0, "unbalanced edit blocks (%d)", gEditBlocks);
    for (int i = 1; i <= 5; ++i) CHECK(val("pin" + std::to_string(i) + "_on", 0)[0] == 1, "pin %d not enabled", i);
    CHECK(val("pin6_on", 0)[0] == 0, "pin 6 should be free");
    const auto pos5 = val("pin5_pos", 0), rest5 = val("pin5_rest", 0);
    std::printf("  pin5 rest %.4f %.4f pos %.4f %.4f, keys %zu, selected %g\n", rest5[0], rest5[1], pos5[0], pos5[1],
                param("pin5_pos")->keys.size(), val("selectedPin", 0)[0]);
    CHECK(std::fabs(pos5[0] - 650.0 / gW) < 1e-6 && std::fabs(pos5[1] - 430.0 / gH) < 1e-6, "pin5 position wrong");
    // The neck pin already bent the body a little, so the click maps back to a nearby rest point.
    CHECK(std::hypot(rest5[0] * gW - 576.0, rest5[1] * gH - 222.0) < 15.0, "pin5 rest wrong");
    CHECK(param("pin5_pos")->keys.size() == 1, "auto keyframe should have set one key");
    CHECK(val("selectedPin", 0)[0] == 5, "pin 5 should be selected");
    CHECK(param("pin5_group")->props["OfxParamPropSecret"].i[0] == 0, "used pin group should be visible");

    // A single click on the image only deselects.
    pen(kOfxInteractActionPenDown, 480, 150);
    pen(kOfxInteractActionPenUp, 480, 150);
    CHECK(val("pin6_on", 0)[0] == 0, "a single click added a pin");
    CHECK(val("selectedPin", 0)[0] == 0, "a single click on the image should deselect");
    // Two clicks far apart are not a double-click.
    pen(kOfxInteractActionPenDown, 460, 140);
    pen(kOfxInteractActionPenUp, 460, 140);
    pen(kOfxInteractActionPenDown, 480, 260);
    pen(kOfxInteractActionPenUp, 480, 260);
    CHECK(val("pin6_on", 0)[0] == 0, "two separate clicks added a pin");

    // Clicking off the image must not add a pin.
    pen(kOfxInteractActionPenDown, 50, 500);
    pen(kOfxInteractActionPenUp, 50, 500);
    CHECK(val("pin6_on", 0)[0] == 0, "click outside the mesh added a pin");

    ident = true;
    CHECK(renderFrame(out, &ident), "render with pins failed");
    CHECK(!ident, "posed frame reported as identity");
    ppt::savePNG(outDir + "/host_1_posed.png", out);
    ppt::Image ov = out;
    drawOverlay(ov);
    std::printf("  overlay: %d mesh lines, %d discs, %d outlines, %d labels\n", gCanvas.lines, gCanvas.polys,
                gCanvas.loops, gCanvas.texts);
    CHECK(gCanvas.lines > 100 && gCanvas.polys == 5, "overlay did not draw mesh and pins");
    ppt::savePNG(outDir + "/host_2_overlay.png", ov);

    // Animate: at frame 24 drag the hand down; frame 12 must be in between.
    gTime = 24;
    drag(650, 430, 620, 150);
    CHECK(param("pin5_pos")->keys.size() == 2, "expected 2 keys on pin 5, got %zu", param("pin5_pos")->keys.size());
    const auto mid = val("pin5_pos", 12);
    CHECK(std::fabs(mid[0] - (650.0 + 620.0) / 2 / gW) < 1e-6, "keyframe interpolation wrong");
    gTime = 12;
    CHECK(renderFrame(out), "render frame 12 failed");
    ppt::savePNG(outDir + "/host_3_frame12.png", out);

    // Dragging an existing pin from its edge keeps the grab offset.
    gTime = 0;
    drag(442, 54, 452, 64);
    const auto foot = val("pin1_pos", 0);
    CHECK(std::fabs(foot[0] * gW - 450) < 1e-6 && std::fabs(foot[1] * gH - 62) < 1e-6, "drag offset not kept (%.2f %.2f)",
          foot[0] * gW, foot[1] * gH);

    // Rest pose mode: moving a pin's rest position.
    param("mode")->value = {1};
    ident = false;
    param("showMesh")->value = {0};
    renderFrame(out, &ident);
    CHECK(ident, "rest pose mode should show the original image");
    param("showMesh")->value = {1};
    {
        const auto r = val("pin3_rest", 0);
        std::printf("  pin3 rest before %.3f %.3f\n", r[0] * gW, r[1] * gH);
    }
    drag(480, 200, 485, 210);
    const auto r3 = val("pin3_rest", 0);
    std::printf("  pin3 rest after %.3f %.3f\n", r3[0] * gW, r3[1] * gH);
    CHECK(std::fabs(r3[0] * gW - 485) < 1e-6 && std::fabs(r3[1] * gH - 210) < 1e-6, "rest drag failed");
    ppt::Image restOv = src;
    drawOverlay(restOv);
    ppt::savePNG(outDir + "/host_4_restpose.png", restOv);
    param("mode")->value = {0};

    // Buttons.
    param("selectedPin")->value = {2};
    pressButton("removePin");
    CHECK(val("pin2_on", 0)[0] == 0 && val("selectedPin", 0)[0] == 0, "Remove Selected Pin failed");
    CHECK(param("pin2_group")->props["OfxParamPropSecret"].i[0] == 1, "removed pin group should hide");
    pressButton("addPin");
    CHECK(val("pin2_on", 0)[0] == 1, "Add Pin should reuse the free slot");
    param("selectedPin")->value = {5};
    pressButton("resetPin");
    CHECK(param("pin5_pos")->keys.empty(), "Reset Selected Pin should clear keys");
    pressButton("removeAll");
    int on = 0;
    for (int i = 1; i <= 48; ++i) on += (int)val("pin" + std::to_string(i) + "_on", 0)[0];
    CHECK(on == 0, "Remove All Pins left %d pins", on);

    // Deformers: an IK leg and a waving arm.
    gTime = 0;
    addPinAt(480, 200);  // 1 hips
    addPinAt(438, 118);  // 2 left knee
    addPinAt(440, 52);   // 3 left foot
    addPinAt(520, 52);   // 4 right foot
    addPinAt(480, 325);  // 5 shoulder
    addPinAt(420, 276);  // 6 left elbow
    addPinAt(384, 222);  // 7 left hand
    param("def1_type")->value = {3};
    param("def1_pins")->sval = "1, 2, 3";
    drag(440, 52, 360, 120);  // kick: move the foot, the knee follows by IK
    const auto knee = val("pin2_pos", 0);
    CHECK(std::fabs(knee[0] * gW - 438) < 1e-6 && std::fabs(knee[1] * gH - 118) < 1e-6,
          "IK must not write the joint pin's own value");
    CHECK(renderFrame(out), "IK render failed");
    {
        ppt::Image ik = out;
        drawOverlay(ik);
        std::printf("[ik] overlay: %d lines, %d discs\n", gCanvas.lines, gCanvas.polys);
        ppt::savePNG(outDir + "/host_6_ik.png", ik);
    }
    param("def2_type")->value = {2};
    param("def2_pins")->sval = "5, 6, 7";
    param("def2_swing")->value = {35};
    param("def2_speed")->value = {1};
    ppt::Image w0, w6;
    gTime = 0;
    renderFrame(w0);
    gTime = 6;
    renderFrame(w6);
    {
        ppt::Image a = w0, b = w6;
        gTime = 0;
        drawOverlay(a);
        gTime = 6;
        drawOverlay(b);
        ppt::savePNG(outDir + "/host_7_wave_f0.png", a);
        ppt::savePNG(outDir + "/host_8_wave_f6.png", b);
    }
    double waveDiff = 0;
    for (size_t i = 0; i < w0.px.size(); ++i) waveDiff += std::fabs(w0.px[i] - w6.px[i]);
    CHECK(waveDiff > 100.0, "the wave deformer does not animate (diff %.1f)", waveDiff);
    {
        // Wiggle and Wind animate on their own; Spring trails its leader, then settles where a
        // Follower would be.
        std::printf("[deformers]\n");
        param("def2_type")->value = {0};
        auto diff = [](const ppt::Image& a, const ppt::Image& b) {
            double d = 0;
            for (size_t i = 0; i < a.px.size(); ++i) d += std::fabs(a.px[i] - b.px[i]);
            return d;
        };
        auto frames = [&](double t0, double t1, ppt::Image& a, ppt::Image& b) {
            gTime = t0;
            renderFrame(a);
            gTime = t1;
            renderFrame(b);
        };
        ppt::Image a, b;
        param("def3_type")->value = {5};
        param("def3_pins")->sval = "7";
        param("def3_size")->value = {25};
        frames(0, 10, a, b);
        const double wig = diff(a, b);
        param("def3_type")->value = {6};
        param("def3_pins")->sval = "5, 6, 7";
        frames(0, 10, a, b);
        const double wnd = diff(a, b);
        // Spring: the shoulder (5) jumps right at frame 2, the hand (7) follows it.
        gTime = 2;
        drag(480, 325, 540, 325);
        param("def3_type")->value = {7};
        param("def3_pins")->sval = "5, 7";
        param("def3_amount")->value = {1};
        ppt::Image s4, s60, f4, f60;
        frames(4, 60, s4, s60);
        param("def3_type")->value = {1};
        param("def3_delay")->value = {0};
        frames(4, 60, f4, f60);
        std::printf("  wiggle %.1f, wind %.1f, spring vs follower: frame 4 %.1f, frame 60 %.1f\n", wig, wnd,
                    diff(s4, f4), diff(s60, f60));
        CHECK(wig > 100.0, "wiggle does not animate (%.1f)", wig);
        CHECK(wnd > 100.0, "wind does not animate (%.1f)", wnd);
        CHECK(diff(s4, f4) > 100.0, "spring should trail its leader right after it moves");
        CHECK(diff(s60, f60) < 0.05 * diff(s4, f4), "spring should settle where a follower is");
        param("def3_type")->value = {0};
    }
    param("def1_type")->value = {0};
    param("def2_type")->value = {0};
    gTime = 0;
    pressButton("removeAll");

    // Solvers: pose the arm with keyframes at frames 0 and 4, compare Static / Elastic / Smooth Follow.
    gTime = 0;
    addPinAt(440, 52);   // 1 left foot
    addPinAt(520, 52);   // 2 right foot
    addPinAt(480, 200);  // 3 hips
    addPinAt(480, 335);  // 4 neck
    addPinAt(576, 222);  // 5 hand, keyed at frame 0
    gTime = 4;
    drag(576, 222, 650, 430);  // raised at frame 4, then held
    auto diffImages = [](const ppt::Image& a, const ppt::Image& b) {
        double d = 0;
        for (size_t i = 0; i < a.px.size(); ++i) d += std::fabs(a.px[i] - b.px[i]);
        return d;
    };
    auto renderAt = [&](double t, ppt::Image& img) {
        gTime = t;
        return renderFrame(img);
    };
    std::vector<ppt::Image> stat(61), elas(61), smooth(61);
    // Hosts (Fusion) reuse the last rendered frame when nothing animates, unless the output is frame varying.
    auto frameVarying = [&]() {
        PropSet prefs;
        call(kOfxImageEffectActionGetClipPreferences, gInstance, nullptr, &prefs);
        const auto& v = prefs[kOfxImageEffectFrameVarying].i;
        return !v.empty() && v[0] != 0;
    };
    param("solverMode")->value = {0};
    CHECK(!frameVarying(), "static solver should not be frame varying");
    param("def3_type")->value = {2};
    CHECK(frameVarying(), "a wave deformer must make the output frame varying");
    param("def3_type")->value = {0};
    param("solverMode")->value = {2};
    CHECK(frameVarying(), "the elastic solver must make the output frame varying");
    param("solverMode")->value = {0};
    for (int f : {5, 6, 8, 10, 14, 20, 30, 60}) renderAt(f, stat[f]);
    param("solverMode")->value = {2};
    param("firmness")->value = {1.5};
    for (int f = 0; f <= 60; ++f) CHECK(renderAt(f, elas[f]), "elastic render failed at %d", f);
    param("solverMode")->value = {1};
    for (int f : {5, 6, 8, 10, 14, 20, 30, 60}) renderAt(f, smooth[f]);
    std::printf("[solver] difference to the static pose:\n  frame   elastic   smooth\n");
    for (int f : {5, 6, 8, 10, 14, 20, 30, 60})
        std::printf("  %5d  %8.1f  %8.1f\n", f, diffImages(elas[f], stat[f]), diffImages(smooth[f], stat[f]));
    const double e6 = diffImages(elas[6], stat[6]), e60 = diffImages(elas[60], stat[60]);
    CHECK(e6 > 200.0, "elastic should differ from the static pose right after the motion (%.1f)", e6);
    CHECK(e60 < 0.05 * e6 && e60 < diffImages(elas[30], stat[30]), "elastic should settle (%.1f vs %.1f)", e60, e6);
    const double s6 = diffImages(smooth[6], stat[6]), s20 = diffImages(smooth[20], stat[20]),
                 s60 = diffImages(smooth[60], stat[60]);
    CHECK(s6 > 200.0 && s20 < s6 && s60 < 0.02 * s6, "smooth follow should lag and arrive (%.1f %.1f %.1f)", s6, s20,
          s60);
    // A frame rendered again (after rendering later frames) must be identical: the cache is consistent.
    param("solverMode")->value = {2};
    ppt::Image again;
    renderAt(60, again);
    renderAt(10, again);
    CHECK(diffImages(again, elas[10]) == 0.0, "re-rendering frame 10 gave a different image");
    // Editing a keyframe re-simulates.
    gTime = 4;
    drag(650, 430, 700, 380);
    ppt::Image edited;
    renderAt(10, edited);
    CHECK(diffImages(edited, elas[10]) > 100.0, "changing a keyframe did not re-simulate");
    {
        ppt::Image a = elas[6], b = elas[10];
        ppt::savePNG(outDir + "/host_9_elastic_f6.png", a);
        ppt::savePNG(outDir + "/host_10_elastic_f10.png", b);
    }
    param("solverMode")->value = {0};
    gTime = 0;
    pressButton("removeAll");

    // Weight map, anchor: everything left of x = 440 is white and must not move.
    ppt::Image weight(gW, gH);
    for (int y = 0; y < gH; ++y)
        for (int x = 0; x < 440; ++x) {
            float* w = weight.at(x, y);
            w[0] = w[1] = w[2] = w[3] = 1.0f;
        }
    Clip* wclip = inst.clip("WeightMap");
    CHECK(wclip != nullptr, "no WeightMap input");
    addPinAt(480, 200);                 // hips
    addPinAt(480, 335, 560, 335);       // neck pulled right
    addPinAt(576, 222);                 // hand
    ppt::Image freeImg, anchored;
    renderAt(0, freeImg);
    if (wclip) {
        wclip->image = &weight;
        wclip->props.setI(kOfxImageClipPropConnected, {1});
    }
    renderAt(0, anchored);
    double leftFree = 0, leftAnchored = 0;
    for (int y = 0; y < gH; ++y)
        for (int x = 0; x < 410; ++x)
            for (int c = 0; c < 4; ++c) {
                leftFree += std::fabs(freeImg.at(x, y)[c] - src.at(x, y)[c]);
                leftAnchored += std::fabs(anchored.at(x, y)[c] - src.at(x, y)[c]);
            }
    std::printf("[weight map] change left of the anchor edge: %.1f free, %.4f anchored\n", leftFree, leftAnchored);
    CHECK(leftFree > 100.0, "test setup: the left side should move without a weight map");
    CHECK(leftAnchored < 1e-3, "anchored area moved (%.4f)", leftAnchored);
    ppt::savePNG(outDir + "/host_11_weight_anchor.png", anchored);
    // Stiffen mode keeps the shape but lets it move.
    param("weightMode")->value = {1};
    ppt::Image stiff;
    renderAt(0, stiff);
    CHECK(diffImages(stiff, anchored) > 100.0, "stiffen mode should differ from anchor mode");
    if (wclip) {
        wclip->image = nullptr;
        wclip->props.setI(kOfxImageClipPropConnected, {0});
    }
    param("weightMode")->value = {0};
    pressButton("removeAll");

    {
        // Pin influence: the hand pin only pulls the area within 140 px (along the shape).
        std::printf("[influence]\n");
        addPinAt(576, 222, 640, 300);
        ppt::Image unlimited, limited;
        renderAt(0, unlimited);
        param("pin1_influence")->value = {140.0};
        renderAt(0, limited);
        auto legsChange = [&](const ppt::Image& img) {
            double sum = 0.0;
            for (int y = 0; y < 150; ++y)
                for (int x = 0; x < gW; ++x)
                    for (int c = 0; c < 4; ++c) sum += std::fabs(img.at(x, y)[c] - src.at(x, y)[c]);
            return sum;
        };
        const double du = legsChange(unlimited), dl = legsChange(limited);
        std::printf("  change of the legs: no limit %.1f, influence 140: %.4f\n", du, dl);
        CHECK(du > 100.0, "test setup: a single pin should move the legs too");
        CHECK(dl == 0.0, "the legs moved although the hand pin's influence does not reach them (%.4f)", dl);
        CHECK(diffImages(limited, unlimited) > 100.0, "the influence changed nothing");
        ppt::savePNG(outDir + "/host_12_influence.png", limited);
        // Viewer: a dotted circle with a handle; dragging the handle sets the influence.
        ppt::Image ov(gW, gH);
        drawOverlay(ov);
        CHECK(gCanvas.rects >= 2, "no influence handle drawn (%d rectangles)", gCanvas.rects);
        ppt::savePNG(outDir + "/host_13_influence_overlay.png", ov);
        const double k = 0.7071067811865476;
        drag(640 + 140 * k, 300 + 140 * k, 640 + 200 * k, 300 + 200 * k);
        const double after = param("pin1_influence")->value[0];
        std::printf("  dragging the handle out: influence %.1f\n", after);
        CHECK(std::fabs(after - 200.0) < 1.0, "dragging the influence handle gave %.1f instead of 200", after);

        // While a pin is dragged, the overlay draws the posed image itself (hosts re-render after).
        // The preview stays up after a drag until the host has rendered again, as hosts do now.
        ppt::Image rerendered;
        renderAt(0, rerendered);
        drawOverlay(ov);
        const int idlePolys = gCanvas.polys;
        pen(kOfxInteractActionPenDown, 640, 300);
        pen(kOfxInteractActionPenMotion, 600, 330);
        ppt::Image during(gW, gH);
        drawOverlay(during);
        const int dragPolys = gCanvas.polys;
        pen(kOfxInteractActionPenUp, 600, 330);
        std::printf("  drag preview: %d filled polygons while dragging, %d idle\n", dragPolys, idlePolys);
        CHECK(dragPolys > idlePolys + 1000, "no drag preview drawn (%d polygons)", dragPolys);
        ppt::savePNG(outDir + "/host_14_drag_preview.png", during);
        pressButton("removeAll");
    }

    {
        // On-image handles of the selected pin: drag the diamond below it to give it a radius, then
        // the round knob on the radius circle to turn it.
        std::printf("[handles]\n");
        param("keyMode")->value = {0};
        addPinAt(480, 200);  // hips
        addPinAt(576, 222);  // hand (selected)
        // A pin moved by the host's own handle (an animated pin's path point in Fusion) or edited in
        // the Inspector becomes the selected pin; a plain time change does not select anything.
        notifyChanged("pin1_pos", kOfxChangeUserEdited);
        const int afterEdit = (int)param("selectedPin")->value[0];
        notifyChanged("pin2_rotation", kOfxChangeTime);
        const int afterTime = (int)param("selectedPin")->value[0];
        notifyChanged("pin2_radius", kOfxChangePluginEdited);
        const int afterPlugin = (int)param("selectedPin")->value[0];
        std::printf("  selected pin after host edit %d, time change %d, plugin edit %d\n", afterEdit, afterTime,
                    afterPlugin);
        CHECK(afterEdit == 1 && afterTime == 1 && afterPlugin == 2, "pin selection does not follow pin edits");
        {
            // Delete in the viewer removes the selected pin; without a selection the key is the host's.
            addPinAt(520, 52);  // pin 3, selected
            const OfxStatus del = keyDown(kOfxKey_Delete);
            const bool gone = val("pin3_on", 0)[0] == 0.0 && val("pin1_on", 0)[0] == 1.0;
            const OfxStatus none = keyDown(kOfxKey_Delete);
            std::printf("  Delete key: pin 3 removed %d (handled %d), with nothing selected handled %d\n", gone,
                        del == kOfxStatOK, none == kOfxStatOK);
            CHECK(gone && del == kOfxStatOK, "Delete should remove the selected pin");
            CHECK(none != kOfxStatOK && val("pin1_on", 0)[0] == 1.0, "Delete without a selected pin must do nothing");
            param("selectedPin")->value = {2};  // back to the hand for the handle checks
        }
        ppt::Image before;
        renderAt(0, before);
        drag(576, 222 - 18, 576, 222 - 30);  // radius handle sits 18 px below a pin without radius
        const double radius = param("pin2_radius")->value[0];
        drag(576 + 30, 222, 576 - 30, 222 + 1);  // knob at angle 0 -> over the top to about 180 degrees
        const double rotation = param("pin2_rotation")->value[0];
        ppt::Image turned;
        renderAt(0, turned);
        std::printf("  radius %.1f, rotation %.1f, image change %.1f\n", radius, rotation, diffImages(turned, before));
        CHECK(std::fabs(radius - 30.0) < 0.5, "radius handle gave %.2f instead of 30", radius);
        CHECK(std::fabs(rotation - 178.1) < 2.0, "rotation knob gave %.2f degrees instead of about 178", rotation);
        CHECK(diffImages(turned, before) > 100.0, "turning the pin did not change the image");
        ppt::Image ov(gW, gH);
        drawOverlay(ov);
        ppt::savePNG(outDir + "/host_15_handles.png", ov);
        {
            // Scale on a pin without a radius: the pin gets a starting radius and the image scales.
            param("pin2_rotation")->value = {0.0};
            ppt::Image unscaled, scaled;
            renderAt(0, unscaled);
            param("pin1_scale")->value = {0.6};
            notifyChanged("pin1_scale", kOfxChangeUserEdited);
            const double r1 = param("pin1_radius")->value[0];
            renderAt(0, scaled);
            std::printf("  scale 0.6 on a pin without radius: radius now %.1f, image change %.1f\n", r1,
                        diffImages(scaled, unscaled));
            CHECK(r1 > 0.0, "a scaled pin should get a radius");
            CHECK(diffImages(scaled, unscaled) > 100.0, "pin scale has no effect");
        }
        param("keyMode")->value = {1};
        pressButton("removeAll");
    }

    {
        // R: turn the selected pin by moving the mouse around it; Esc cancels; or drag anywhere.
        std::printf("[rotate key]\n");
        gTime = 0;
        param("keyMode")->value = {0};
        addPinAt(480, 200);  // hips
        addPinAt(576, 222);  // hand (selected, no radius yet)
        CHECK(keyDown(kOfxKey_e) == kOfxStatOK, "E with a selected pin should start turning it");
        const double radius = param("pin2_radius")->value[0];
        for (int i = 0; i <= 10; ++i) {  // hover from angle 0 to 90 degrees around the pin
            const double a = 1.5707963267948966 * i / 10.0;
            pen(kOfxInteractActionPenMotion, 576 + 60 * std::cos(a), 222 + 60 * std::sin(a));
        }
        const double hovered = param("pin2_rotation")->value[0];
        keyDown(kOfxKey_Escape);
        const double cancelled = param("pin2_rotation")->value[0];
        keyDown(kOfxKey_e);
        drag(576, 222 + 60, 576 - 60, 222);  // drag anywhere: from 90 to 180 degrees around the pin
        const double dragged = param("pin2_rotation")->value[0];
        const OfxStatus after = keyDown(kOfxKey_t);  // not ours
        // S: scale by the mouse's distance from the pin.
        CHECK(keyDown(kOfxKey_s) == kOfxStatOK, "S with a selected pin should start scaling it");
        for (int i = 0; i <= 10; ++i) pen(kOfxInteractActionPenMotion, 576 + 60 + 6 * i, 222);  // 60 -> 120 px away
        const double grown = param("pin2_scale")->value[0];
        keyDown(kOfxKey_Escape);
        const double restored = param("pin2_scale")->value[0];
        keyDown(kOfxKey_s);
        drag(576 + 60, 222, 576 + 30, 222);  // drag from 60 to 30 px away
        const double shrunk = param("pin2_scale")->value[0];
        std::printf("  S: hover scale %.3f, after Esc %.3f, drag scale %.3f\n", grown, restored, shrunk);
        CHECK(std::fabs(grown - 2.0) < 1e-6, "moving the mouse twice as far should double the scale (%.3f)", grown);
        CHECK(std::fabs(restored - 1.0) < 1e-9, "Esc should put the scale back");
        CHECK(std::fabs(shrunk - 0.5) < 1e-6, "dragging half as far after S should halve the scale (%.3f)", shrunk);
        param("selectedPin")->value = {0};
        const OfxStatus noPin = keyDown(kOfxKey_e);
        std::printf("  R: radius %.1f, hover turn %.1f, after Esc %.1f, drag turn %.1f, R without pin handled %d\n", radius,
                    hovered, cancelled, dragged, noPin == kOfxStatOK);
        CHECK(radius > 0.0, "R should give a pin without radius a starting radius");
        CHECK(std::fabs(hovered - 90.0) < 1.0, "moving the mouse around should turn the pin 90 degrees (%.2f)", hovered);
        CHECK(std::fabs(cancelled) < 1e-6, "Esc should put the rotation back");
        CHECK(std::fabs(dragged - 90.0) < 1.0, "dragging after R should turn the pin (%.2f)", dragged);
        CHECK(after != kOfxStatOK && noPin != kOfxStatOK, "other keys / no selected pin must be left to the host");
        param("keyMode")->value = {1};
        pressButton("removeAll");
    }

    {
        // Several pins: a box around them selects them (the pin where they hang on the rest becomes the
        // pivot), dragging one moves them all, E turns them around the pivot, Esc puts them back,
        // Shift+click adds or removes one, typed lists select, Delete removes them all.
        std::printf("[selection]\n");
        gTime = 0;
        param("keyMode")->value = {0};
        gUnblocked.clear();
        addPinAt(480, 200);  // 1 hips
        addPinAt(480, 325);  // 2 shoulder
        addPinAt(420, 276);  // 3 elbow
        addPinAt(384, 222);  // 4 hand
        drag(355, 300, 440, 205);  // a box from an empty spot around elbow and hand
        const std::string boxed = param("selectedPins")->sval;
        const int pivot = (int)param("selectedPin")->value[0];
        drag(384, 222, 394, 232);  // drag the hand: the elbow comes along
        const auto elbow = pinAt(3), hand = pinAt(4), shoulder = pinAt(2);
        std::printf("  box selected \"%s\" (pivot %d); dragged: elbow %.1f %.1f, hand %.1f %.1f, shoulder %.1f %.1f\n",
                    boxed.c_str(), pivot, elbow[0], elbow[1], hand[0], hand[1], shoulder[0], shoulder[1]);
        CHECK(boxed == "3, 4" && pivot == 3, "box selection should pick pins 3, 4 around pin 3 (\"%s\", %d)",
              boxed.c_str(), pivot);
        CHECK(std::hypot(elbow[0] - 430, elbow[1] - 286) < 1e-6 && std::hypot(hand[0] - 394, hand[1] - 232) < 1e-6 &&
                  std::hypot(shoulder[0] - 480, shoulder[1] - 325) < 1e-6,
              "dragging a selected pin should move every selected pin");
        CHECK(param("selectedPins")->sval == "3, 4" && param("selectedPin")->value[0] == 4,
              "the dragged pin should become the pivot and the selection stay");
        pen(kOfxInteractActionPenDown, 430, 286);  // click the elbow: it becomes the pivot again
        pen(kOfxInteractActionPenUp, 430, 286);
        CHECK(keyDown(kOfxKey_e) == kOfxStatOK, "E should turn the selected pins");
        auto hoverQuarter = [&](double cx, double cy) {
            for (int i = 0; i <= 10; ++i) {
                const double a = 1.5707963267948966 * i / 10.0;
                pen(kOfxInteractActionPenMotion, cx + 60 * std::cos(a), cy + 60 * std::sin(a));
            }
        };
        hoverQuarter(430, 286);
        const auto turned = pinAt(4);
        keyDown(kOfxKey_Escape);
        const auto back = pinAt(4);
        keyDown(kOfxKey_e);
        hoverQuarter(430, 286);
        keyDown(kOfxKey_e);  // keep
        const auto kept = pinAt(4), elbowKept = pinAt(3);
        std::printf("  E on 3, 4: hand turned to %.1f %.1f, after Esc %.1f %.1f, kept %.1f %.1f; rotation %.1f, radius %.1f\n",
                    turned[0], turned[1], back[0], back[1], kept[0], kept[1], param("pin4_rotation")->value[0],
                    param("pin4_radius")->value[0]);
        // the hand (-36, -54) from the elbow turns a quarter: (54, -36)
        CHECK(std::hypot(turned[0] - 484, turned[1] - 250) < 0.5, "E should turn the hand around the elbow");
        CHECK(std::hypot(back[0] - 394, back[1] - 232) < 1e-6, "Esc should put the turned pins back");
        CHECK(std::hypot(kept[0] - 484, kept[1] - 250) < 0.5 && std::hypot(elbowKept[0] - 430, elbowKept[1] - 286) < 1e-6,
              "E, move, E should keep the turn and leave the pivot");
        CHECK(param("pin4_radius")->value[0] == 0.0 && param("pin4_rotation")->value[0] == 0.0,
              "turning a group must not give its pins a radius");
        // S scales the group away from the pivot.
        keyDown(kOfxKey_s);
        for (int i = 0; i <= 10; ++i) pen(kOfxInteractActionPenMotion, 430 + 60 + 6 * i, 286);  // 60 -> 120 px away
        keyDown(kOfxKey_Return);
        const auto scaled = pinAt(4);
        CHECK(std::hypot(scaled[0] - 538, scaled[1] - 214) < 0.5, "S should push the hand twice as far (%.1f %.1f)",
              scaled[0], scaled[1]);
        // Shift+click adds the shoulder and takes it out again.
        keyDown(kOfxKey_Shift_L);
        pen(kOfxInteractActionPenDown, 480, 325);
        pen(kOfxInteractActionPenUp, 480, 325);
        const std::string withShoulder = param("selectedPins")->sval;
        pen(kOfxInteractActionPenDown, 480, 325);
        pen(kOfxInteractActionPenUp, 480, 325);
        keyUp(kOfxKey_Shift_L);
        const std::string without = param("selectedPins")->sval;
        std::printf("  Shift+click shoulder: \"%s\", again: \"%s\" (pivot %g)\n", withShoulder.c_str(), without.c_str(),
                    param("selectedPin")->value[0]);
        CHECK(withShoulder == "3, 4, 2" && without == "3, 4" && param("selectedPin")->value[0] == 3,
              "Shift+click should add / remove a pin");
        // Typed in the Inspector.
        param("selectedPins")->sval = "1, 2";
        notifyChanged("selectedPins", kOfxChangeUserEdited);
        const int typedPivot = (int)param("selectedPin")->value[0];
        param("selectedPin")->value = {4};
        notifyChanged("selectedPin", kOfxChangeUserEdited);
        const std::string single = param("selectedPins")->sval;
        CHECK(typedPivot == 1 && single == "4", "typed selections should stay in step (%d, \"%s\")", typedPivot,
              single.c_str());
        ppt::Image ov(gW, gH);
        param("selectedPins")->sval = "3, 4";
        param("selectedPin")->value = {3};
        drawOverlay(ov);
        CHECK(gCanvas.polys >= 4, "selected pins not drawn");
        // Delete removes every selected pin.
        keyDown(kOfxKey_Delete);
        const bool gone = val("pin3_on", 0)[0] == 0.0 && val("pin4_on", 0)[0] == 0.0 && val("pin1_on", 0)[0] == 1.0 &&
                          val("pin2_on", 0)[0] == 1.0;
        CHECK(gone && param("selectedPin")->value[0] == 0, "Delete should remove all selected pins");
        CHECK(gUnblocked.empty(), "the viewer changed %s outside an undo step",
              gUnblocked.empty() ? "" : gUnblocked[0].c_str());
        CHECK(gEditBlocks == 0, "unbalanced edit blocks (%d)", gEditBlocks);
        param("keyMode")->value = {1};
        pressButton("removeAll");
    }

    {
        // Every change in the viewer is one undo step: after adding two pins and moving the first, undo
        // puts the first back and keeps the second; a turn with E is undone in one go.
        std::printf("[undo]\n");
        gTime = 0;
        param("keyMode")->value = {0};
        gUndo.clear();
        gUnblocked.clear();
        addPinAt(480, 200);        // 1
        addPinAt(576, 222);        // 2
        drag(480, 200, 500, 230);  // move pin 1
        const size_t steps = gUndo.size();
        undo();
        const auto p1 = pinAt(1);
        const bool pin2 = val("pin2_on", 0)[0] == 1.0;
        pen(kOfxInteractActionPenDown, 576, 222);  // select pin 2
        pen(kOfxInteractActionPenUp, 576, 222);
        keyDown(kOfxKey_e);
        for (int i = 0; i <= 10; ++i) {
            const double a = 1.5707963267948966 * i / 10.0;
            pen(kOfxInteractActionPenMotion, 576 + 60 * std::cos(a), 222 + 60 * std::sin(a));
        }
        pen(kOfxInteractActionPenDown, 576, 282);  // a click keeps it
        pen(kOfxInteractActionPenUp, 576, 282);
        const double turned = param("pin2_rotation")->value[0];
        const size_t stepsTurn = gUndo.size();
        undo();
        const double back = param("pin2_rotation")->value[0], radiusBack = param("pin2_radius")->value[0];
        std::printf("  %zu undo steps for 2 pins + a move; after undo pin 1 at %.1f %.1f, pin 2 there %d; turn %.1f "
                    "(%zu steps), undone %.1f, radius %.1f\n",
                    steps, p1[0], p1[1], (int)pin2, turned, stepsTurn - steps + 1, back, radiusBack);
        CHECK(steps == 3, "adding 2 pins and moving one should be 3 undo steps (%zu)", steps);
        CHECK(std::hypot(p1[0] - 480, p1[1] - 200) < 1e-6 && pin2, "undo should put the moved pin back and keep pin 2");
        CHECK(std::fabs(turned - 90.0) < 1.0 && stepsTurn == steps, "a turn with E should be one undo step");
        CHECK(std::fabs(back) < 1e-9 && radiusBack == 0.0, "undo should take the whole turn back");
        CHECK(gUnblocked.empty(), "the viewer changed %s outside an undo step",
              gUnblocked.empty() ? "" : gUnblocked[0].c_str());
        CHECK(gEditBlocks == 0, "unbalanced edit blocks (%d)", gEditBlocks);
        param("keyMode")->value = {1};
        pressButton("removeAll");
    }

    {
        // Pose tools: Copy / Paste Pose, Mirror Pose (selected pins onto their partners, or the whole
        // pose), Key All Pins; then a rig saved to a file, wiped and loaded back.
        std::printf("[pose tools]\n");
        gTime = 0;
        param("keyMode")->value = {0};
        addPinAt(480, 200);  // 1 hips
        addPinAt(384, 222);  // 2 left hand
        addPinAt(576, 222);  // 3 right hand
        drag(384, 222, 404, 262);  // left hand: +20, +40
        pressButton("copyPose");
        drag(404, 262, 424, 282);
        pressButton("pastePose");
        const auto pasted = pinAt(2);
        pen(kOfxInteractActionPenDown, 404, 262);  // select the left hand
        pen(kOfxInteractActionPenUp, 404, 262);
        pressButton("mirrorPose");
        const auto right = pinAt(3), left = pinAt(2), hips = pinAt(1);
        std::printf("  pasted left hand %.1f %.1f; mirrored onto the right hand %.1f %.1f (left %.1f %.1f, hips %.1f %.1f)\n",
                    pasted[0], pasted[1], right[0], right[1], left[0], left[1], hips[0], hips[1]);
        CHECK(std::hypot(pasted[0] - 404, pasted[1] - 262) < 1e-6, "Paste Pose should put the copied pose back");
        CHECK(std::hypot(right[0] - 556, right[1] - 262) < 1e-6 && std::hypot(left[0] - 404, left[1] - 262) < 1e-6 &&
                  std::hypot(hips[0] - 480, hips[1] - 200) < 1e-6,
              "Mirror Pose should copy the left hand's pose, mirrored, onto the right hand only");
        drag(480, 200, 490, 210);  // hips off to the right
        param("selectedPin")->value = {0};
        pressButton("mirrorPose");  // nothing selected: the whole pose flips
        const auto hipsFlipped = pinAt(1), leftFlipped = pinAt(2);
        CHECK(std::hypot(hipsFlipped[0] - 470, hipsFlipped[1] - 210) < 1e-6 &&
                  std::hypot(leftFlipped[0] - 404, leftFlipped[1] - 262) < 1e-6,
              "Mirror Pose without a selection should flip the whole pose (hips %.1f %.1f)", hipsFlipped[0],
              hipsFlipped[1]);
        gTime = 10;
        pressButton("keyAllPins");
        const size_t keys1 = param("pin1_pos")->keys.size(), keys3 = param("pin3_pos")->keys.size();
        const size_t rotKeys = param("pin3_rotation")->keys.size();
        CHECK(keys1 == 1 && keys3 == 1 && rotKeys == 0, "Key All Pins should key every pin's position (%zu %zu %zu)",
              keys1, keys3, rotKeys);

        // Rig file.
        param("def1_type")->value = {2};
        param("def1_pins")->sval = "1, 2";
        param("def1_swing")->value = {7.5};
        param("pin3_radius")->value = {25.0};
        const std::string rigPath = outDir + "/test_rig.pprig";
        param("rigFile")->sval = rigPath;
        gMessages.clear();
        pressButton("saveRig");
        pressButton("removeAll");
        param("def1_type")->value = {0};
        param("def1_pins")->sval = "";
        param("def1_swing")->value = {15.0};
        pressButton("loadRig");
        const auto loaded3 = pinAt(3, 10);
        std::printf("  rig: %zu message(s); loaded pin 3 at %.1f %.1f radius %.1f, keys %zu; deformer type %g pins \"%s\" swing %g\n",
                    gMessages.size(), loaded3[0], loaded3[1], param("pin3_radius")->value[0],
                    param("pin3_pos")->keys.size(), param("def1_type")->value[0], param("def1_pins")->sval.c_str(),
                    param("def1_swing")->value[0]);
        CHECK(val("pin1_on", 0)[0] == 1 && val("pin2_on", 0)[0] == 1 && val("pin3_on", 0)[0] == 1 &&
                  val("pin4_on", 0)[0] == 0,
              "Load Rig should bring back pins 1-3");
        CHECK(std::hypot(loaded3[0] - 556, loaded3[1] - 262) < 1e-3 && param("pin3_radius")->value[0] == 25.0 &&
                  param("pin3_pos")->keys.empty(),
              "Load Rig should restore the pins' positions and settings without keyframes");
        CHECK(param("def1_type")->value[0] == 2 && param("def1_pins")->sval == "1, 2" &&
                  param("def1_swing")->value[0] == 7.5,
              "Load Rig should restore the deformers");
        std::remove(rigPath.c_str());
        param("def1_type")->value = {0};
        param("def1_pins")->sval = "";
        param("rigFile")->sval = "";
        CHECK(param("pin1_layer")->props[kOfxParamPropAnimates].i[0] == 1, "pin layers should be animatable");
        CHECK(gEditBlocks == 0, "unbalanced edit blocks (%d)", gEditBlocks);
        gTime = 0;
        param("keyMode")->value = {1};
        pressButton("removeAll");
    }

    {
        // Motion blur, onion skin and motion trails on a hand keyed from frame 0 to 10.
        std::printf("[motion blur]\n");
        gTime = 0;
        param("keyMode")->value = {1};
        addPinAt(480, 200);  // hips
        addPinAt(576, 222);  // hand, keyed at 0
        gTime = 10;
        drag(576, 222, 650, 400);  // keyed at 10
        ppt::Image sharp, blurred, closed;
        renderAt(5, sharp);
        param("motionBlur")->value = {1};
        renderAt(5, blurred);
        param("shutter")->value = {0.0};
        renderAt(5, closed);
        param("shutter")->value = {0.5};
        int softSharp = 0, softBlur = 0;
        for (int y = 0; y < gH; ++y)
            for (int x = 0; x < gW; ++x) {
                const float a = sharp.at(x, y)[3], b = blurred.at(x, y)[3];
                softSharp += a > 0.1f && a < 0.9f;
                softBlur += b > 0.1f && b < 0.9f;
            }
        std::printf("  blur changed the frame by %.1f, with the shutter closed by %.4f; soft edge pixels %d -> %d\n",
                    diffImages(blurred, sharp), diffImages(closed, sharp), softSharp, softBlur);
        CHECK(diffImages(blurred, sharp) > 100.0 && softBlur > softSharp + 300, "motion blur should smear the moving hand");
        CHECK(diffImages(closed, sharp) < 1.0, "a closed shutter should give the sharp frame");
        param("motionBlur")->value = {0};

        gTime = 5;
        ppt::Image ov(gW, gH);
        drawOverlay(ov);
        const int linesPlain = gCanvas.lines;
        param("onionSkin")->value = {1};
        drawOverlay(ov);
        const int linesOnion = gCanvas.lines;
        param("onionSkin")->value = {0};
        param("selectedPin")->value = {2};
        param("selectedPins")->sval = "2";
        drawOverlay(ov);
        const int rectsPlain = gCanvas.rects, polysPlain = gCanvas.polys;
        param("motionTrails")->value = {1};
        drawOverlay(ov);
        const int rectsTrail = gCanvas.rects, polysTrail = gCanvas.polys;
        ppt::savePNG(outDir + "/host_17_trails.png", ov);
        param("motionTrails")->value = {0};
        std::printf("  onion skin: %d -> %d lines; trails: %d -> %d dots, %d -> %d polygons\n", linesPlain, linesOnion,
                    rectsPlain, rectsTrail, polysPlain, polysTrail);
        CHECK(linesOnion > linesPlain + 50, "onion skin should draw the outlines of other frames");
        CHECK(rectsTrail >= rectsPlain + 20 && polysTrail >= polysPlain + 2,
              "motion trails should draw a dot per frame and the keyframes");
        param("keyMode")->value = {1};
        gTime = 0;
        pressButton("removeAll");
    }

    {
        // Locators: one on the hand follows it; baking writes a key per frame of the clip.
        std::printf("[locators]\n");
        gTime = 0;
        addPinAt(480, 200);              // hips
        addPinAt(576, 222);              // hand (selected)
        pressButton("addLocator");       // on the selected pin's point
        drag(576, 222, 620, 300);        // pose the hand
        ppt::Image img;
        renderAt(0, img);
        pressButton("bakeLocators");
        const auto& keys = param("loc1_pos")->keys;
        const auto at0 = keys.count(0.0) ? keys.at(0.0) : std::vector<double>{0, 0};
        const double ex = at0[0] * gW - 620, ey = at0[1] * gH - 300;
        std::printf("  loc1: %zu keys, at frame 0 off the hand by %.2f px, rotation keys %zu\n", keys.size(),
                    std::hypot(ex, ey), param("loc1_rot")->keys.size());
        CHECK(keys.size() == 101, "baking should key every frame 0..100 (%zu keys)", keys.size());
        CHECK(std::hypot(ex, ey) < 1.0, "the locator should sit on the posed hand (%.2f px off)", std::hypot(ex, ey));
        CHECK(param("loc1_rot")->keys.size() == 101, "rotation should be baked too");
        // Placing a locator by dragging it (in Edit Rest Pose, where it sits on its rest point).
        pressButton("addLocator");
        param("loc2_rest")->value = {480.0 / gW, 330.0 / gH};
        param("mode")->value = {1};
        drag(480, 330, 470, 360);
        const auto r2 = param("loc2_rest")->value;
        std::printf("  loc2 dragged to %.1f %.1f\n", r2[0] * gW, r2[1] * gH);
        CHECK(std::fabs(r2[0] * gW - 470) < 0.5 && std::fabs(r2[1] * gH - 360) < 0.5, "locator drag failed");
        param("mode")->value = {0};
        ppt::Image ov(gW, gH);
        drawOverlay(ov);
        ppt::savePNG(outDir + "/host_16_locators.png", ov);
        param("loc1_on")->value = {0};
        param("loc2_on")->value = {0};
        pressButton("removeAll");
    }

    {
        // Image layers and outputs: a green bar on Layer 2 crosses the character's chest.
        std::printf("[layers]\n");
        ppt::Image bar(gW, gH);
        for (int y = 300; y < 320; ++y)
            for (int x = 380; x < 640; ++x) {
                float* q = bar.at(x, y);
                q[0] = 0.1f; q[1] = 0.8f; q[2] = 0.2f; q[3] = 1.0f;
            }
        Clip* l2 = inst.clip("Layer2");
        CHECK(l2 != nullptr, "no Layer2 input");
        l2->image = &bar;
        l2->props.setI(kOfxImageClipPropConnected, {1});
        gTime = 0;
        ppt::Image still;
        bool ident = true;
        renderFrame(still, &ident);
        const float* c = still.at(480, 310);
        CHECK(!ident, "a connected layer must not pass the source through");
        CHECK(c[1] > 0.7f && c[0] < 0.2f, "layer 2 should be drawn over the source (%.2f %.2f %.2f)", c[0], c[1], c[2]);
        addPinAt(390, 310);              // the bar's left end holds
        addPinAt(630, 310, 630, 380);    // its right end goes up
        ppt::Image posed;
        renderFrame(posed);
        double legs = 0.0;
        for (int y = 0; y < 150; ++y)
            for (int x = 0; x < gW; ++x)
                for (int k = 0; k < 4; ++k) legs += std::fabs(posed.at(x, y)[k] - src.at(x, y)[k]);
        std::printf("  bar end lifted: alpha at the new end %.2f, character's legs changed by %.4f\n",
                    posed.at(628, 380)[3], legs);
        CHECK(posed.at(628, 380)[3] > 0.5f, "the bar layer did not follow its pin");
        CHECK(legs == 0.0, "pins on layer 2 moved the source layer (%.4f)", legs);
        param("outputMode")->value = {1};
        param("outputLayer")->value = {2};
        ppt::Image single;
        renderFrame(single);
        CHECK(single.at(440, 52)[3] == 0.0f && single.at(628, 380)[3] > 0.5f,
              "Single Layer 2 should show only the bar");
        // Motion vectors: the right end moves up 10 px per frame between keys at frames 0 and 4.
        param("outputMode")->value = {2};
        gTime = 4;
        drag(630, 380, 630, 420);
        gTime = 2;
        ppt::Image vec;
        renderFrame(vec);
        const float* tip = vec.at(626, 400);
        const float* leg = vec.at(440, 60);
        std::printf("  motion vectors: bar tip %.2f %.2f (alpha %.2f), leg %.2f %.2f (alpha %.2f)\n", tip[0], tip[1],
                    tip[3], leg[0], leg[1], leg[3]);
        CHECK(tip[3] > 0.5f && std::fabs(tip[1] - 10.0f) < 3.0f && std::fabs(tip[0]) < 3.0f,
              "bar tip should move about 10 px up per frame");
        CHECK(leg[3] > 0.5f && leg[0] == 0.0f && leg[1] == 0.0f, "the still character should have zero motion");
        ppt::savePNG(outDir + "/host_17_layers.png", posed);
        param("outputMode")->value = {0};
        l2->image = nullptr;
        l2->props.setI(kOfxImageClipPropConnected, {0});
        gTime = 0;
        pressButton("removeAll");
    }

    {
        // Loop / Ping-Pong: the hand is keyed at frames 0 and 10; after frame 10 the animation repeats.
        std::printf("[loop]\n");
        gTime = 0;
        addPinAt(480, 200);
        addPinAt(576, 222);
        gTime = 10;
        drag(576, 222, 640, 300);
        auto frame = [&](double t) {
            ppt::Image img;
            gTime = t;
            renderFrame(img);
            return img;
        };
        param("loopStart")->value = {0};
        param("loopEnd")->value = {10};
        param("loopMode")->value = {2};  // ping-pong: frame 18 plays frame 2
        const double pp18 = diffImages(frame(18), frame(2)), off18 = diffImages(frame(18), frame(8));
        param("loopMode")->value = {1};  // loop: frame 18 plays frame 8
        const double lp18 = diffImages(frame(18), frame(8));
        param("loopMode")->value = {0};  // off: frame 18 holds the last key (frame 10)
        const double no18 = diffImages(frame(18), frame(10));
        std::printf("  ping-pong 18 vs 2: %.1f (vs 8: %.1f), loop 18 vs 8: %.1f, off 18 vs 10: %.1f\n", pp18, off18,
                    lp18, no18);
        CHECK(pp18 == 0.0 && off18 > 100.0, "ping-pong should play frame 2 at frame 18");
        CHECK(lp18 == 0.0, "loop should play frame 8 at frame 18");
        CHECK(no18 == 0.0, "without a loop the last key should hold");
        gTime = 0;
        pressButton("removeAll");
        // 48 pins.
        for (int i = 0; i < 48; ++i) pressButton("addPin");
        CHECK(val("pin48_on", 0)[0] == 1.0, "48 pins should be available");
        pressButton("removeAll");
    }

    // Burn-in output and full-frame mesh mode.
    param("meshSource")->value = {1};
    renderFrame(out);
    addPinAt(100, 100);
    addPinAt(860, 100);
    addPinAt(480, 270, 560, 330);
    param("burnIn")->value = {1};
    CHECK(renderFrame(out), "burn-in render failed");
    ppt::savePNG(outDir + "/host_5_burnin_fullframe.png", out);

    {
        // The Filter context (Resolve's Edit / Color page) keeps the host's frame.
        OfxRectD r{};
        CHECK(!regionOfDefinition(r), "the Filter context should leave the region of definition to the host");
    }
    gOverlayMain(kOfxActionDestroyInstance, &gInteract, nullptr, nullptr);
    CHECK(call(kOfxActionDestroyInstance, &inst) == kOfxStatOK, "destroy failed");

    {
        // Fusion: Puppet Pin -> Merge. Fusion uses the General context and passes the node's output
        // on with the region of definition the plugin reports, so a limb moved past the edge of a
        // cut-out character must widen it, or the Merge only gets the cropped image.
        std::printf("[fusion merge]\n");
        Effect gdesc;
        gdesc.props = desc.props;
        PropSet gArgs;
        gArgs.setS(kOfxImageEffectPropContext, kOfxImageEffectContextGeneral);
        CHECK(call(kOfxImageEffectActionDescribeInContext, &gdesc, &gArgs) == kOfxStatOK, "describe General failed");
        Effect g;
        gInstance = &g;
        g.props = gdesc.props;
        g.props.setS(kOfxImageEffectPropContext, kOfxImageEffectContextGeneral);
        g.props.setS(kOfxPropType, kOfxTypeImageEffectInstance);
        g.props.setD(kOfxImageEffectPropProjectSize, {(double)gW, (double)gH});
        g.props.setD(kOfxImageEffectPropProjectExtent, {(double)gW, (double)gH});
        g.props.setD(kOfxImageEffectPropProjectOffset, {0.0, 0.0});
        g.props.setD(kOfxImageEffectPropProjectPixelAspectRatio, {1.0});
        g.props.setD(kOfxImageEffectInstancePropEffectDuration, {100.0});
        g.props.setD(kOfxImageEffectPropFrameRate, {24.0});
        g.props.setI(kOfxPropIsInteractive, {1});
        for (auto& p : gdesc.params.params) {
            auto q = std::make_unique<Param>(*p);
            const Prop& def = q->props[kOfxParamPropDefault];
            q->value.assign(q->dims(), 0.0);
            if (q->type == kOfxParamTypeString && !def.s.empty()) q->sval = def.s[0];
            for (int i = 0; i < q->dims(); ++i) {
                if (i < (int)def.d.size()) q->value[i] = def.d[i];
                else if (i < (int)def.i.size()) q->value[i] = def.i[i];
            }
            g.params.params.push_back(std::move(q));
        }
        ppt::Image cut = ppt::makeCharacter(gW, gH);
        for (auto& c : gdesc.clips) {
            auto k = std::make_unique<Clip>(*c);
            k->props.setI(kOfxImageClipPropConnected, {k->name == kOfxImageEffectSimpleSourceClipName || k->name == kOfxImageEffectOutputClipName ? 1 : 0});
            k->props.setS(kOfxImageEffectPropPixelDepth, kOfxBitDepthFloat);
            k->props.setS(kOfxImageEffectPropComponents, kOfxImageComponentRGBA);
            k->props.setS(kOfxImageClipPropUnmappedPixelDepth, kOfxBitDepthFloat);
            k->props.setS(kOfxImageClipPropUnmappedComponents, kOfxImageComponentRGBA);
            k->props.setS(kOfxImageEffectPropPreMultiplication, kOfxImagePreMultiplied);
            k->props.setD(kOfxImagePropPixelAspectRatio, {1.0});
            k->props.setD(kOfxImageEffectPropFrameRate, {24.0});
            k->props.setD(kOfxImageEffectPropFrameRange, {0.0, 100.0});
            k->props.setS(kOfxImageClipPropFieldOrder, kOfxImageFieldNone);
            if (k->name == kOfxImageEffectSimpleSourceClipName) k->image = &cut;
            g.clips.push_back(std::move(k));
        }
        CHECK(call(kOfxActionCreateInstance, &g) == kOfxStatOK, "createInstance (General) failed");
        gTime = 0;
        OfxRectD r{};
        CHECK(regionOfDefinition(r) && r.x1 == 0 && r.y1 == 0 && r.x2 == gW && r.y2 == gH,
              "without pins the output should be the Source's region (%.0f %.0f %.0f %.0f)", r.x1, r.y1, r.x2, r.y2);
        // Torso held, right hand thrown far past the right edge of the image.
        auto setPin = [&](int n, double rx, double ry, double px, double py) {
            const std::string s = "pin" + std::to_string(n) + "_";
            param(s + "on")->value = {1};
            param(s + "rest")->value = {rx / gW, ry / gH};
            param(s + "pos")->value = {px / gW, py / gH};
        };
        setPin(1, 480, 250, 480, 250);
        setPin(2, 480, 330, 480, 330);
        setPin(3, 576, 222, 1150, 300);
        CHECK(regionOfDefinition(r) && r.x2 > 1150, "before the first render the region should already reach the hand (x2 %.0f)", r.x2);
        ppt::Image frame;
        CHECK(renderFrame(frame), "General render failed");
        CHECK(regionOfDefinition(r), "no region of definition with pins");
        std::printf("  region with the hand thrown out: %.0f %.0f %.0f %.0f\n", r.x1, r.y1, r.x2, r.y2);
        CHECK(r.x2 > 1150 && r.x2 < 1400 && r.x1 <= 0 && r.y1 <= 0 && r.y2 >= gH,
              "the region should hold the frame and the thrown hand, not much more (%.0f %.0f %.0f %.0f)", r.x1, r.y1,
              r.x2, r.y2);
        ppt::Image wide;
        CHECK(renderWindow(wide, (int)r.x1, (int)r.y1, (int)r.x2, (int)r.y2), "render of the wide region failed");
        float handA = 0.0f;
        for (int y = 285; y < 315; ++y)
            for (int x = 1130; x < 1160; ++x) handA = std::max(handA, wide.at(x - (int)r.x1, y - (int)r.y1)[3]);
        std::printf("  alpha at the thrown hand (outside the original image): %.2f\n", handA);
        CHECK(handA > 0.5f, "the hand past the image edge was cut off");
        // inside the frame it is the same picture as a frame-sized render
        double same = 0.0;
        for (int y = 0; y < gH; y += 3)
            for (int x = 0; x < gW; x += 3)
                for (int k = 0; k < 4; ++k)
                    same = std::max(same, (double)std::fabs(wide.at(x - (int)r.x1, y - (int)r.y1)[k] - frame.at(x, y)[k]));
        CHECK(same < 1e-5, "the wide render differs from the frame render inside the frame (%g)", same);
        ppt::savePNG(outDir + "/host_18_merge_wide.png", wide);

        // Whatever part of the output is asked for, the whole Source is needed.
        {
            PropSet in, out;
            in.setD(kOfxPropTime, {gTime});
            in.setD(kOfxImageEffectPropRenderScale, {1.0, 1.0});
            in.setD(kOfxImageEffectPropRegionOfInterest, {1000.0, 280.0, 1100.0, 320.0});
            const std::string roiProp = std::string("OfxImageClipPropRoI_") + kOfxImageEffectSimpleSourceClipName;
            out.setD(roiProp.c_str(), {1000.0, 280.0, 1100.0, 320.0});
            call(kOfxImageEffectActionGetRegionsOfInterest, gInstance, &in, &out);
            const auto& v = out[roiProp].d;
            CHECK(v.size() == 4 && v[0] == 0 && v[1] == 0 && v[2] == gW && v[3] == gH,
                  "the Source should be asked for in full (%.0f %.0f %.0f %.0f)", v[0], v[1], v[2], v[3]);
        }

        {
            // "Puppet Pin - Add Room": the Source goes into a bigger see-through picture (as Fusion's
            // Merge over a Background does) and the pin frame says where the old picture went. The
            // pose must stay the same and the thrown hand must now be there.
            const int padX = 300, padY = 200, PW = gW + 2 * padX, PH = gH + 2 * padY;
            ppt::Image padded(PW, PH);
            for (int y = 0; y < gH; ++y)
                for (int x = 0; x < gW; ++x)
                    for (int k = 0; k < 4; ++k) padded.at(x + padX, y + padY)[k] = cut.at(x, y)[k];
            g.clip(kOfxImageEffectSimpleSourceClipName)->image = &padded;
            g.props.setD(kOfxImageEffectPropProjectSize, {(double)PW, (double)PH});
            param("pinFrameOrigin")->value = {(double)padX, (double)padY};
            param("pinFrameSize")->value = {(double)gW, (double)gH};
            ppt::Image big;
            const int bx = 0, by = 0;
            CHECK(renderWindow(big, bx, by, PW, PH), "render with room failed");
            double sum = 0.0;
            int n = 0;
            for (int y = 0; y < gH; y += 2)
                for (int x = 0; x < gW; x += 2, ++n)
                    for (int k = 0; k < 4; ++k) sum += std::fabs(big.at(x + padX, y + padY)[k] - frame.at(x, y)[k]);
            float hand = 0.0f;
            for (int y = 285; y < 315; ++y)
                for (int x = 1130; x < 1160; ++x) hand = std::max(hand, big.at(x + padX, y + padY)[3]);
            std::printf("  with room: mean difference inside the old picture %.5f, thrown hand alpha %.2f\n", sum / (n * 4.0), hand);
            CHECK(sum / (n * 4.0) < 0.004, "the pose changed after adding room (%.5f)", sum / (n * 4.0));
            CHECK(hand > 0.5f, "the thrown hand is still cut off with room");
            ppt::savePNG(outDir + "/host_19_room.png", big);
            g.clip(kOfxImageEffectSimpleSourceClipName)->image = &cut;
            g.props.setD(kOfxImageEffectPropProjectSize, {(double)gW, (double)gH});
            param("pinFrameOrigin")->value = {0.0, 0.0};
            param("pinFrameSize")->value = {0.0, 0.0};
            renderFrame(frame);
        }

        // A Merge with a background IN FRONT of Puppet Pin: the source is solid everywhere, so the
        // whole frame becomes the puppet. The plugin says so.
        ppt::Image merged = cut;
        for (int y = 0; y < gH; ++y)
            for (int x = 0; x < gW; ++x) {
                float* p = merged.at(x, y);
                const float ia = 1.0f - p[3];
                p[0] += 0.1f * ia;
                p[1] += 0.3f * ia;
                p[2] += 0.6f * ia;
                p[3] = 1.0f;
            }
        // Resolve shows no plugin status messages, so the viewer overlay says it (never the render).
        gInteract = InteractInst();
        gInteract.props.setP(kOfxPropEffectInstance, &g);
        gInteract.props.setD(kOfxInteractPropPixelScale, {1.0, 1.0});
        gInteract.props.setD(kOfxInteractPropBackgroundColour, {0, 0, 0});
        CHECK(gOverlayMain(kOfxActionCreateInstance, &gInteract, nullptr, nullptr) == kOfxStatOK,
              "overlay create (General) failed");
        auto overlayTexts = [&]() {
            ppt::Image canvas(gW, gH);
            drawOverlay(canvas);
            return gCanvas.texts;
        };
        const int textsCut = overlayTexts();
        g.clip(kOfxImageEffectSimpleSourceClipName)->image = &merged;
        renderFrame(frame);
        const int textsMerged = overlayTexts();
        std::printf("  viewer texts: cut-out %d, background merged in front %d\n", textsCut, textsMerged);
        CHECK(textsMerged >= textsCut + 2, "an opaque Source (background merged in front) should be pointed out in the viewer");
        g.clip(kOfxImageEffectSimpleSourceClipName)->image = &cut;
        renderFrame(frame);
        CHECK(overlayTexts() == textsCut, "the viewer warning should go with a cut-out Source again");
        param("meshSource")->value = {1};  // Full Frame on purpose: no warning
        g.clip(kOfxImageEffectSimpleSourceClipName)->image = &merged;
        renderFrame(frame);
        CHECK(overlayTexts() == textsCut, "Mesh From Full Frame should not warn");
        gOverlayMain(kOfxActionDestroyInstance, &gInteract, nullptr, nullptr);

        CHECK(call(kOfxActionDestroyInstance, &g) == kOfxStatOK, "destroy (General) failed");
    }
    call(kOfxActionUnload, nullptr);
    std::printf(gFailures ? "\n%d FAILURE(S)\n" : "\nALL OK\n", gFailures);
    return gFailures ? 1 : 0;
}
