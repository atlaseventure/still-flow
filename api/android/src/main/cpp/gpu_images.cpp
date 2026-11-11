#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>
#include <GLES3/gl3.h>
#include <android/hardware_buffer_jni.h>
#include <jni.h>
#include <stdexcept>

namespace {
void failure(JNIEnv* env, const char* message) {
    if (auto cls = env->FindClass("java/lang/IllegalStateException")) {
        env->ThrowNew(cls, message);
        env->DeleteLocalRef(cls);
    }
}
} // namespace
extern "C" {
JNIEXPORT jlong JNICALL Java_io_stillflow_GpuFrames_nativeImport(JNIEnv* env, jclass, jobject buffer) {
    auto* hardware = AHardwareBuffer_fromHardwareBuffer(env, buffer);
    const auto get_buffer =
        reinterpret_cast<PFNEGLGETNATIVECLIENTBUFFERANDROIDPROC>(eglGetProcAddress("eglGetNativeClientBufferANDROID"));
    const auto create_image = reinterpret_cast<PFNEGLCREATEIMAGEKHRPROC>(eglGetProcAddress("eglCreateImageKHR"));
    const auto bind_image =
        reinterpret_cast<PFNGLEGLIMAGETARGETTEXTURE2DOESPROC>(eglGetProcAddress("glEGLImageTargetTexture2DOES"));
    if (!hardware || !get_buffer || !create_image || !bind_image) {
        failure(env, "HardwareBuffer EGL import unavailable");
        return 0;
    }
    const EGLint attributes[] = {EGL_IMAGE_PRESERVED_KHR, EGL_TRUE, EGL_NONE};
    const auto image = create_image(eglGetCurrentDisplay(), EGL_NO_CONTEXT, EGL_NATIVE_BUFFER_ANDROID,
                                    get_buffer(hardware), attributes);
    if (image == EGL_NO_IMAGE_KHR) {
        failure(env, "Cannot import decoder HardwareBuffer");
        return 0;
    }
    bind_image(GL_TEXTURE_EXTERNAL_OES, image);
    return reinterpret_cast<jlong>(image);
}
JNIEXPORT void JNICALL Java_io_stillflow_GpuFrames_nativeRelease(JNIEnv*, jclass, jlong image) {
    const auto destroy = reinterpret_cast<PFNEGLDESTROYIMAGEKHRPROC>(eglGetProcAddress("eglDestroyImageKHR"));
    if (destroy && image)
        destroy(eglGetCurrentDisplay(), reinterpret_cast<EGLImageKHR>(image)); // NOLINT(performance-no-int-to-ptr)
}
}
