// Test helpers: float RGBA images, a synthetic character, and a dependency-free PNG writer.
#pragma once

#include "../src/core/pp_render.h"

#include <cstdio>
#include <cstdint>
#include <string>
#include <vector>

namespace ppt {

struct Image {
    int w = 0, h = 0;
    std::vector<float> px;  // premultiplied RGBA, row 0 = bottom
    Image() = default;
    Image(int w_, int h_) : w(w_), h(h_), px((size_t)w_ * h_ * 4, 0.0f) {}
    float* at(int x, int y) { return &px[((size_t)y * w + x) * 4]; }
    const float* at(int x, int y) const { return &px[((size_t)y * w + x) * 4]; }
    pp::ImageView view()
    {
        pp::ImageView v;
        v.data = px.data();
        v.x1 = 0; v.y1 = 0; v.x2 = w; v.y2 = h;
        v.rowBytes = (ptrdiff_t)w * 4 * sizeof(float);
        return v;
    }
};

inline uint32_t crc32(const uint8_t* d, size_t n, uint32_t c = 0)
{
    static uint32_t table[256];
    static bool init = false;
    if (!init) {
        for (uint32_t i = 0; i < 256; ++i) {
            uint32_t k = i;
            for (int j = 0; j < 8; ++j) k = (k & 1) ? 0xEDB88320u ^ (k >> 1) : k >> 1;
            table[i] = k;
        }
        init = true;
    }
    c = ~c;
    for (size_t i = 0; i < n; ++i) c = table[(c ^ d[i]) & 0xFF] ^ (c >> 8);
    return ~c;
}

// Writes an 8-bit RGB (channels 3) or RGBA (channels 4) PNG, top-down rows, using
// uncompressed deflate blocks.
inline bool writePNG(const std::string& path, int w, int h, const std::vector<uint8_t>& rgb, int channels = 3)
{
    std::vector<uint8_t> raw;
    const size_t stride = (size_t)w * channels;
    raw.reserve((stride + 1) * h);
    for (int y = 0; y < h; ++y) {
        raw.push_back(0);
        raw.insert(raw.end(), rgb.begin() + (size_t)y * stride, rgb.begin() + (size_t)(y + 1) * stride);
    }
    std::vector<uint8_t> z = {0x78, 0x01};
    uint32_t a = 1, b = 0;
    for (uint8_t c : raw) { a = (a + c) % 65521; b = (b + a) % 65521; }
    size_t pos = 0;
    do {
        const size_t len = std::min<size_t>(65535, raw.size() - pos);
        const bool last = pos + len == raw.size();
        z.push_back(last ? 1 : 0);
        z.push_back(len & 0xFF); z.push_back((len >> 8) & 0xFF);
        z.push_back(~len & 0xFF); z.push_back((~len >> 8) & 0xFF);
        z.insert(z.end(), raw.begin() + pos, raw.begin() + pos + len);
        pos += len;
    } while (pos < raw.size());
    const uint32_t adler = (b << 16) | a;
    for (int s = 24; s >= 0; s -= 8) z.push_back((adler >> s) & 0xFF);

    FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return false;
    auto put32 = [&](uint32_t v) { uint8_t q[4] = {uint8_t(v >> 24), uint8_t(v >> 16), uint8_t(v >> 8), uint8_t(v)}; std::fwrite(q, 1, 4, f); };
    auto chunk = [&](const char* type, const std::vector<uint8_t>& data) {
        put32((uint32_t)data.size());
        std::vector<uint8_t> td(type, type + 4);
        td.insert(td.end(), data.begin(), data.end());
        std::fwrite(td.data(), 1, td.size(), f);
        put32(crc32(td.data(), td.size()));
    };
    const uint8_t sig[8] = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};
    std::fwrite(sig, 1, 8, f);
    std::vector<uint8_t> ihdr = {uint8_t(w >> 24), uint8_t(w >> 16), uint8_t(w >> 8), uint8_t(w),
                                 uint8_t(h >> 24), uint8_t(h >> 16), uint8_t(h >> 8), uint8_t(h), 8,
                                 uint8_t(channels == 4 ? 6 : 2), 0, 0, 0};
    chunk("IHDR", ihdr);
    chunk("IDAT", z);
    chunk("IEND", {});
    std::fclose(f);
    return true;
}

// Saves an image composited over a grey checkerboard (so transparency is visible).
inline bool savePNG(const std::string& path, const Image& img)
{
    std::vector<uint8_t> rgb((size_t)img.w * img.h * 3);
    for (int y = 0; y < img.h; ++y)
        for (int x = 0; x < img.w; ++x) {
            const float* p = img.at(x, y);
            const float bg = (((x / 12) + (y / 12)) & 1) ? 0.30f : 0.22f;
            uint8_t* o = &rgb[((size_t)(img.h - 1 - y) * img.w + x) * 3];
            for (int c = 0; c < 3; ++c) {
                const float v = p[c] + bg * (1.0f - p[3]);
                o[c] = (uint8_t)std::lround(std::min(1.0f, std::max(0.0f, v)) * 255.0f);
            }
        }
    return writePNG(path, img.w, img.h, rgb);
}

// Saves an image as RGBA PNG with straight (un-premultiplied) alpha.
inline bool saveRGBA(const std::string& path, const Image& img)
{
    std::vector<uint8_t> px((size_t)img.w * img.h * 4);
    for (int y = 0; y < img.h; ++y)
        for (int x = 0; x < img.w; ++x) {
            const float* p = img.at(x, y);
            uint8_t* o = &px[((size_t)(img.h - 1 - y) * img.w + x) * 4];
            const float a = p[3];
            for (int c = 0; c < 3; ++c) {
                const float v = a > 0.0f ? p[c] / a : 0.0f;
                o[c] = (uint8_t)std::lround(std::min(1.0f, std::max(0.0f, v)) * 255.0f);
            }
            o[3] = (uint8_t)std::lround(std::min(1.0f, std::max(0.0f, a)) * 255.0f);
        }
    return writePNG(path, img.w, img.h, px, 4);
}

inline double capsuleDist(double px, double py, double ax, double ay, double bx, double by)
{
    return std::sqrt(pp::segmentDist2({px, py}, {ax, ay}, {bx, by}));
}

// A textured stick figure plus a separate ball, with anti-aliased alpha. Coordinates are
// given for a 960x540 frame and scaled by s.
inline Image makeCharacter(int w, int h)
{
    Image img(w, h);
    const double s = w / 960.0;
    struct Cap { double ax, ay, bx, by, r; };
    const Cap caps[] = {
        {480, 200, 480, 335, 26},  // torso
        {480, 200, 438, 118, 15},  // left thigh
        {438, 118, 440, 52, 13},   // left shin
        {480, 200, 522, 118, 15},  // right thigh
        {522, 118, 520, 52, 13},   // right shin
        {440, 52, 412, 46, 10},    // left foot
        {520, 52, 548, 46, 10},    // right foot
        {480, 320, 420, 276, 12},  // left upper arm
        {420, 276, 384, 222, 11},  // left forearm
        {480, 320, 540, 276, 12},  // right upper arm
        {540, 276, 576, 222, 11},  // right forearm
        {480, 335, 480, 360, 11},  // neck
        {480, 392, 480, 392, 32},  // head
        {760, 120, 760, 120, 42},  // ball (separate island)
    };
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            const double px = (x + 0.5) / s, py = (y + 0.5) / s;
            double best = 1e9;
            int which = 0;
            for (int i = 0; i < (int)(sizeof caps / sizeof caps[0]); ++i) {
                const double d = capsuleDist(px, py, caps[i].ax, caps[i].ay, caps[i].bx, caps[i].by) - caps[i].r;
                if (d < best) { best = d; which = i; }
            }
            const double a = std::min(1.0, std::max(0.0, 0.5 - best * s));
            if (a <= 0.0) continue;
            const bool check = ((int)std::floor(px / 9.0) + (int)std::floor(py / 9.0)) & 1;
            double r, g, b;
            if (which == 13) { r = 0.95; g = 0.35; b = 0.25; }
            else if (which == 12) { r = 0.95; g = 0.78; b = 0.55; }
            else { r = 0.25 + 0.5 * (py / 540.0); g = 0.45; b = 0.85 - 0.4 * (py / 540.0); }
            if (check) { r *= 0.75; g *= 0.75; b *= 0.75; }
            float* o = img.at(x, y);
            o[0] = (float)(r * a); o[1] = (float)(g * a); o[2] = (float)(b * a); o[3] = (float)a;
        }
    return img;
}

// Opaque full-frame test card.
inline Image makeCard(int w, int h)
{
    Image img(w, h);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            const bool check = ((x * 16 / w) + (y * 9 / h)) & 1;
            float* o = img.at(x, y);
            o[0] = (float)x / w * (check ? 1.0f : 0.6f);
            o[1] = (float)y / h * (check ? 1.0f : 0.6f);
            o[2] = check ? 0.8f : 0.3f;
            o[3] = 1.0f;
        }
    return img;
}

}  // namespace ppt
