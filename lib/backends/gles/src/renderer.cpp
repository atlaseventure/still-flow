#include "stillflow/backends/gles.hpp"
#include <EGL/egl.h>
#include <GLES3/gl31.h>
#include <algorithm>
#include <array>
#include <cstring>
#include <limits>

namespace stillflow {
namespace {
void check_gl(const char* operation) {
    if (const auto error = glGetError(); error != GL_NO_ERROR)
        throw BackendError(std::string(operation) + ": GL error " + std::to_string(error));
}

class Context {
  public:
    EGLDisplay display = EGL_NO_DISPLAY;
    EGLContext context = EGL_NO_CONTEXT;
    EGLSurface surface = EGL_NO_SURFACE;

    Context() {
        // EGL_DEFAULT_DISPLAY belongs to the process. Do not terminate a display
        // also used by the host app; only destroy resources owned by this backend.
        display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
        if (display == EGL_NO_DISPLAY || !eglInitialize(display, nullptr, nullptr))
            throw BackendError("EGL display unavailable");
        const EGLint attributes[] = {EGL_SURFACE_TYPE,
                                     EGL_PBUFFER_BIT,
                                     EGL_RENDERABLE_TYPE,
                                     EGL_OPENGL_ES3_BIT,
                                     EGL_RED_SIZE,
                                     8,
                                     EGL_GREEN_SIZE,
                                     8,
                                     EGL_BLUE_SIZE,
                                     8,
                                     EGL_NONE};
        EGLConfig config{};
        EGLint count = 0;
        if (!eglChooseConfig(display, attributes, &config, 1, &count) || count != 1)
            throw BackendError("EGL ES3 configuration unavailable");
        const EGLint context_attributes[] = {EGL_CONTEXT_CLIENT_VERSION, 3, EGL_NONE};
        const EGLenum previous_api = eglQueryAPI();
        eglBindAPI(EGL_OPENGL_ES_API);
        context = eglCreateContext(display, config, EGL_NO_CONTEXT, context_attributes);
        eglBindAPI(previous_api);
        if (context == EGL_NO_CONTEXT)
            throw BackendError("EGL context unavailable");
        const EGLint surface_attributes[] = {EGL_WIDTH, 1, EGL_HEIGHT, 1, EGL_NONE};
        surface = eglCreatePbufferSurface(display, config, surface_attributes);
        if (surface == EGL_NO_SURFACE) {
            eglDestroyContext(display, context);
            context = EGL_NO_CONTEXT;
            throw BackendError("EGL pbuffer unavailable");
        }
    }
    ~Context() {
        if (surface != EGL_NO_SURFACE)
            eglDestroySurface(display, surface);
        if (context != EGL_NO_CONTEXT)
            eglDestroyContext(display, context);
    }
    Context(const Context&) = delete;
    Context& operator=(const Context&) = delete;
};

class Current {
  public:
    explicit Current(const Context& context)
        : own_display_(context.display), display_(eglGetCurrentDisplay()), context_(eglGetCurrentContext()),
          draw_(eglGetCurrentSurface(EGL_DRAW)), read_(eglGetCurrentSurface(EGL_READ)) {
        if (!eglMakeCurrent(context.display, context.surface, context.surface, context.context))
            throw BackendError("Cannot activate EGL context");
    }
    ~Current() {
        if (display_ != EGL_NO_DISPLAY)
            eglMakeCurrent(display_, draw_, read_, context_);
        else
            eglMakeCurrent(own_display_, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    }
    Current(const Current&) = delete;
    Current& operator=(const Current&) = delete;

  private:
    EGLDisplay own_display_, display_;
    EGLContext context_;
    EGLSurface draw_, read_;
};

constexpr const char* shader_body = R"glsl(
precision highp float;
precision highp int;
layout(local_size_x = 16, local_size_y = 16) in;
layout(std430, binding = 0) readonly buffer Input { uint inputPixels[]; };
layout(std430, binding = 1) writeonly buffer Output { uint outputPixels[]; };
layout(std430, binding = 2) readonly buffer Mesh { vec2 offsets[]; };
layout(std430, binding = 3) buffer Counts { uint invalidCount; uint badCoordinate; };
uniform ivec2 dimensions;
uniform uint stride;
uniform uint channels;
uniform ivec2 meshSize;
uniform vec2 gridScale;
uniform vec4 viewport;

uvec4 samplePixel(ivec2 p) {
    p = clamp(p, ivec2(0), dimensions - 1);
    uint base = uint(p.y) * stride + uint(p.x) * channels;
    uvec4 result = uvec4(0);
    for (uint c = 0u; c < channels; ++c) {
        uint address = base + c;
        result[c] = (inputPixels[address / 4u] >> ((address % 4u) * 8u)) & 255u;
    }
    return result;
}
vec4 weights(float t) {
    float t2=t*t, t3=t2*t;
    return vec4(-0.5*t+t2-0.5*t3, 1.0-2.5*t2+1.5*t3, 0.5*t+2.0*t2-1.5*t3, -0.5*t2+0.5*t3);
}
void main() {
    ivec2 at = ivec2(gl_GlobalInvocationID.xy);
    if (any(greaterThanEqual(at, dimensions))) return;
    vec2 target = viewport.xy + vec2(at) * viewport.zw;
    vec2 p = target;
    for (int iteration = 0; iteration < INVERSE_ITERATIONS; ++iteration) {
        vec2 g = clamp(p * gridScale, vec2(0), vec2(meshSize - 1));
        ivec2 cell = min(ivec2(g), meshSize - 2);
        vec2 f = g - vec2(cell);
        int a = cell.y * meshSize.x + cell.x;
        int b = a + meshSize.x;
        vec2 offset = (offsets[a] * (1.0-f.x) + offsets[a+1] * f.x) * (1.0-f.y)
                    + (offsets[b] * (1.0-f.x) + offsets[b+1] * f.x) * f.y;
        p = target - offset;
    }
    if (any(isnan(p)) || any(isinf(p)) || any(greaterThan(abs(p), vec2(10000000)))) {
        atomicOr(badCoordinate, 1u);
        return;
    }
    if (any(lessThan(p, vec2(0))) || any(greaterThan(p, vec2(dimensions - 1))))
        atomicAdd(invalidCount, 1u);
    p = clamp(p, vec2(0), vec2(dimensions - 1));
    ivec2 lo = ivec2(floor(p));
    vec4 wx = weights(p.x-float(lo.x)), wy = weights(p.y-float(lo.y));
    vec4 sum = vec4(0);
    for (int y=0; y<4; ++y)
        for (int x=0; x<4; ++x)
            sum += vec4(samplePixel(lo+ivec2(x-1,y-1))) * wx[x] * wy[y];
    uvec4 value = uvec4(roundEven(clamp(sum, 0.0, 255.0)));
    outputPixels[at.y * dimensions.x + at.x] = value.x | (value.y << 8u) | (value.z << 16u) | (value.w << 24u);
}
)glsl";

class GlesBackend final : public RenderBackend {
  public:
    void initialize() {
        Current current(context_);
        GLint major = 0, minor = 0;
        glGetIntegerv(GL_MAJOR_VERSION, &major);
        glGetIntegerv(GL_MINOR_VERSION, &minor);
        if (major < 3 || (major == 3 && minor < 1))
            throw BackendError("OpenGL ES 3.1 compute is required");
        const auto* renderer = glGetString(GL_RENDERER);
        name_ = renderer ? reinterpret_cast<const char*>(renderer) : "OpenGL ES";
        glGetInteger64v(GL_MAX_SHADER_STORAGE_BLOCK_SIZE, &max_buffer_);
        const std::string source = "#version 310 es\n#define INVERSE_ITERATIONS " +
                                   std::to_string(mesh_inverse_iterations) + "\n" + shader_body;
        const char* text = source.c_str();
        const GLuint shader = glCreateShader(GL_COMPUTE_SHADER);
        glShaderSource(shader, 1, &text, nullptr);
        glCompileShader(shader);
        GLint success = 0;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
        if (!success) {
            std::array<char, 4096> log{};
            glGetShaderInfoLog(shader, log.size(), nullptr, log.data());
            glDeleteShader(shader);
            throw BackendError(std::string("GLES shader compilation failed: ") + log.data());
        }
        program_ = glCreateProgram();
        glAttachShader(program_, shader);
        glLinkProgram(program_);
        glDeleteShader(shader);
        glGetProgramiv(program_, GL_LINK_STATUS, &success);
        if (!success) {
            std::array<char, 4096> log{};
            glGetProgramInfoLog(program_, log.size(), nullptr, log.data());
            throw BackendError(std::string("GLES program link failed: ") + log.data());
        }
        glGenBuffers(buffers_.size(), buffers_.data());
        check_gl("Initialize GLES compute");
    }

    ~GlesBackend() override {
        try {
            Current current(context_);
            glDeleteBuffers(buffers_.size(), buffers_.data());
            if (program_)
                glDeleteProgram(program_);
        } catch (...) {
            // Destroying the context also releases its GL objects after context loss.
            return;
        }
    }

    BackendInfo info() const override {
        return {"gles",
                name_,
                {{PixelFormat::gray8, PixelFormat::bgr8, PixelFormat::rgba8}, {"host"}},
                {{"sampler", "catmull-rom"}, {"api", "OpenGL ES 3.1 compute"}}};
    }

    void upload(const Image& input) override {
        auto mapping = map_host_image(input);
        const auto channels = pixel_bytes(mapping.format);
        const auto& plane = mapping.planes[0];
        const size_t bytes = (mapping.size.height - 1) * plane.row_stride + mapping.size.width * channels;
        const size_t padded = (bytes + 3) & ~size_t(3);
        const size_t output_bytes = static_cast<size_t>(mapping.size.width) * mapping.size.height * 4;
        if (padded > static_cast<uint64_t>(max_buffer_) || output_bytes > static_cast<uint64_t>(max_buffer_) ||
            plane.row_stride > std::numeric_limits<GLuint>::max())
            throw BackendError("Frame exceeds GLES storage limits");
        Current current(context_);
        reserve(0, padded);
        glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, bytes, plane.bytes.data());
        reserve(1, output_bytes);
        check_gl("Upload GLES frame");
        size_ = mapping.size;
        format_ = mapping.format;
        color_ = mapping.color;
        stride_ = static_cast<GLuint>(plane.row_stride);
        channels_ = static_cast<GLuint>(channels);
        uploaded_ = true;
        rendered_ = false;
    }

    double warp_gyro(const GyroFrameMap&) override {
        throw NotImplementedError("Gyro stabilization rendering is not implemented for GLES backend");
    }
    double warp(const MapParameters& map) override {
        if (!uploaded_)
            throw std::logic_error("No uploaded image");
        if (map.columns < 2 || map.rows < 2 || map.columns > 32 || map.rows > 32 ||
            map.columns * map.rows > max_mesh_vertices)
            throw std::invalid_argument("Invalid displacement mesh");
        Current current(context_);
        std::array<float, static_cast<size_t>(max_mesh_vertices) * 2> offsets{};
        for (size_t i = 0; i < static_cast<size_t>(map.columns) * map.rows; ++i) {
            offsets[2 * i] = static_cast<float>(map.offset_x[i]);
            offsets[2 * i + 1] = static_cast<float>(map.offset_y[i]);
        }
        reserve(2, sizeof(offsets));
        glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, sizeof(offsets), offsets.data());
        std::array<GLuint, 2> counts{};
        reserve(3, sizeof(counts));
        glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, sizeof(counts), counts.data());
        for (GLuint i = 0; i < buffers_.size(); ++i)
            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, i, buffers_[i]);
        glUseProgram(program_);
        glUniform2i(location("dimensions"), size_.width, size_.height);
        glUniform1ui(location("stride"), stride_);
        glUniform1ui(location("channels"), channels_);
        glUniform2i(location("meshSize"), map.columns, map.rows);
        glUniform2f(location("gridScale"), map.grid_x, map.grid_y);
        glUniform4f(location("viewport"), map.origin_x, map.origin_y, map.step_x, map.step_y);
        glDispatchCompute((size_.width + 15) / 16, (size_.height + 15) / 16, 1);
        glMemoryBarrier(GL_BUFFER_UPDATE_BARRIER_BIT | GL_SHADER_STORAGE_BARRIER_BIT);
        check_gl("Dispatch GLES warp");
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, buffers_[3]);
        const void* readback = glMapBufferRange(GL_SHADER_STORAGE_BUFFER, 0, sizeof(counts), GL_MAP_READ_BIT);
        if (!readback)
            throw BackendError("Cannot read GLES boundary counts");
        std::memcpy(counts.data(), readback, sizeof(counts));
        if (!glUnmapBuffer(GL_SHADER_STORAGE_BUFFER))
            throw BackendError("GLES boundary buffer was lost");
        if (counts[1])
            throw std::invalid_argument("Invalid sample coordinate");
        rendered_ = true;
        return double(counts[0]) / (static_cast<double>(size_.width) * size_.height);
    }

    Image download() override {
        if (!rendered_)
            throw std::logic_error("No rendered image");
        const size_t count = static_cast<size_t>(size_.width) * size_.height;
        std::vector<uint8_t> pixels(count * channels_);
        Current current(context_);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, buffers_[1]);
        const auto* mapped =
            static_cast<const uint8_t*>(glMapBufferRange(GL_SHADER_STORAGE_BUFFER, 0, count * 4, GL_MAP_READ_BIT));
        if (!mapped)
            throw BackendError("Cannot read GLES output");
        if (channels_ == 4)
            std::memcpy(pixels.data(), mapped, pixels.size());
        else
            for (size_t i = 0; i < count; ++i)
                std::memcpy(pixels.data() + i * channels_, mapped + i * 4, channels_);
        if (!glUnmapBuffer(GL_SHADER_STORAGE_BUFFER))
            throw BackendError("GLES output buffer was lost");
        check_gl("Download GLES frame");
        auto result = map_host_image(make_host_image(size_, format_, std::move(pixels)));
        result.color = color_;
        return borrow_host_image(std::move(result));
    }

  private:
    void reserve(size_t index, size_t bytes) {
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, buffers_[index]);
        if (capacities_[index] < bytes) {
            glBufferData(GL_SHADER_STORAGE_BUFFER, bytes, nullptr, GL_DYNAMIC_COPY);
            check_gl("Allocate GLES buffer");
            capacities_[index] = bytes;
        }
    }
    GLint location(const char* name) const { return glGetUniformLocation(program_, name); }
    Context context_;
    std::string name_;
    GLuint program_ = 0;
    std::array<GLuint, 4> buffers_{};
    std::array<size_t, 4> capacities_{};
    GLint64 max_buffer_ = 0;
    Size size_{};
    PixelFormat format_{};
    ColorInfo color_{};
    GLuint stride_ = 0, channels_ = 0;
    bool uploaded_ = false, rendered_ = false;
};
} // namespace

std::unique_ptr<RenderBackend> make_gles_backend() {
    auto backend = std::make_unique<GlesBackend>();
    backend->initialize();
    return backend;
}
} // namespace stillflow
