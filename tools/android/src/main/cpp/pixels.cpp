#include <algorithm>
#include <cstdint>
#include <jni.h>
namespace {
uint8_t clip(float v) { return static_cast<uint8_t>(std::clamp(v + 0.5f, 0.0f, 255.0f)); }
uint8_t* plane(JNIEnv* e, jobject b, int w, int h, int row, int pixel) {
    auto* p = static_cast<uint8_t*>(e->GetDirectBufferAddress(b));
    auto cap = e->GetDirectBufferCapacity(b);
    int64_t needed = int64_t(h - 1) * row + int64_t(w - 1) * pixel + 1;
    if (!p || cap < needed || row <= 0 || pixel <= 0) {
        e->ThrowNew(e->FindClass("java/lang/IllegalArgumentException"), "Invalid plane extent");
        return nullptr;
    }
    return p;
}
} // namespace
extern "C" JNIEXPORT void JNICALL Java_io_stillflow_runner_ExportRunner_convert(
    JNIEnv* e, jclass, jboolean to_rgba, jint w, jint h, jobject rgba, jobject yy, jint yr, jint yp, jobject uu,
    jint ur, jint up, jobject vv, jint vr, jint vp, jboolean bt709, jboolean full) {
    auto* rgb = plane(e, rgba, w * 4, h, w * 4, 1);
    if (!rgb)
        return;
    auto* y = plane(e, yy, w, h, yr, yp);
    if (!y)
        return;
    auto* u = plane(e, uu, w / 2, h / 2, ur, up);
    if (!u)
        return;
    auto* v = plane(e, vv, w / 2, h / 2, vr, vp);
    if (!v)
        return;
    float kr = bt709 ? .2126f : .299f, kb = bt709 ? .0722f : .114f, kg = 1 - kr - kb;
    if (to_rgba) {
        for (int j = 0; j < h; ++j)
            for (int i = 0; i < w; ++i) {
                const int cx = i / 2, cy = j / 2;
                float l = y[j * yr + i * yp], cb = u[cy * ur + cx * up] - 128.f, cr = v[cy * vr + cx * vp] - 128.f;
                if (!full) {
                    l = (l - 16) * (255.f / 219);
                    cb *= 255.f / 224;
                    cr *= 255.f / 224;
                }
                auto* p = rgb + (size_t(j) * w + i) * 4;
                p[0] = clip(l + 2 * (1 - kr) * cr);
                p[1] = clip(l - 2 * kb * (1 - kb) / kg * cb - 2 * kr * (1 - kr) / kg * cr);
                p[2] = clip(l + 2 * (1 - kb) * cb);
                p[3] = 255;
            }
    } else {
        for (int j = 0; j < h; j += 2)
            for (int i = 0; i < w; i += 2) {
                float cb = 0, cr = 0;
                for (int dy = 0; dy < 2; ++dy)
                    for (int dx = 0; dx < 2; ++dx) {
                        const auto* p = rgb + (size_t(j + dy) * w + i + dx) * 4;
                        float l = kr * p[0] + kg * p[1] + kb * p[2];
                        y[(j + dy) * yr + (i + dx) * yp] = clip(16 + l * (219.f / 255));
                        cb += (p[2] - l) / (2 * (1 - kb));
                        cr += (p[0] - l) / (2 * (1 - kr));
                    }
                u[(j / 2) * ur + (i / 2) * up] = clip(128 + cb * .25f * (224.f / 255));
                v[(j / 2) * vr + (i / 2) * vp] = clip(128 + cr * .25f * (224.f / 255));
            }
    }
}
