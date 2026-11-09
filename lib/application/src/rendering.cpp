#include "stillflow/application/rendering.hpp"
#include <algorithm>
#include <cmath>
#include <set>

namespace stillflow {
namespace {
class CropRenderer {
  public:
    explicit CropRenderer(std::unique_ptr<RenderBackend> backend)
        : backend_(std::move(backend)), info_(backend_->info()) {}
    const BackendInfo& info() const { return info_; }
    Image render(const Image& input, Warp& warp, Size analysis, double crop, bool horizontal, Profile& profile) {
        if (!input || input->size().width < 2 || input->size().height < 2 || analysis.width < 2 ||
            analysis.height < 2 || !std::isfinite(crop) || crop < 0 || crop > 0.25)
            throw std::invalid_argument("Invalid render geometry");
        const auto& formats = info_.capabilities.formats;
        if (std::find(formats.begin(), formats.end(), input->format()) == formats.end())
            throw BackendError("Backend does not support the input pixel format");
        const std::string stage = "render." + info_.id;
        ScopeTimer total(profile, stage);
        {
            ScopeTimer timer(profile, stage + ".upload");
            try {
                backend_->upload(input);
            } catch (const ImageAccessError& error) {
                throw BackendError(error.what());
            }
        }
        {
            ScopeTimer timer(profile, stage + ".warp");
            for (int attempt = 0;; ++attempt) {
                const auto map = prepare_map(warp, analysis, input->size(), crop, horizontal);
                warp.invalid_fraction = backend_->warp(map);
                if (!std::isfinite(warp.invalid_fraction) || warp.invalid_fraction < 0 || warp.invalid_fraction > 1)
                    throw BackendError("Backend returned an invalid boundary fraction");
                if (warp.invalid_fraction == 0)
                    break;
                if (attempt > 4)
                    throw std::runtime_error("Cannot construct a valid crop");
                warp.strength = attempt < 3 ? warp.strength * 0.75 : 0;
            }
        }
        ScopeTimer timer(profile, stage + ".download");
        auto output = backend_->download();
        if (!output || output->size().width != input->size().width || output->size().height != input->size().height ||
            output->format() != input->format())
            throw BackendError("Backend returned incompatible output geometry or format");
        return output;
    }

  private:
    std::unique_ptr<RenderBackend> backend_;
    BackendInfo info_;
};

class FallbackRenderer final : public Renderer {
  public:
    FallbackRenderer(std::string requested, std::vector<BackendProvider> candidates)
        : candidates_(std::move(candidates)) {
        report_.requested = std::move(requested);
        activate_next();
    }
    Image render(const Image& input, Warp& warp, Size analysis, double crop, bool horizontal,
                 Profile& profile) override {
        if (!active_)
            throw BackendError("No active rendering backend");
        const Warp original = warp;
        for (;;) {
            try {
                auto result = active_->render(input, warp, analysis, crop, horizontal, profile);
                ++report_.backends.back().frames;
                return result;
            } catch (const BackendError& error) {
                report_.failures.push_back({active_->info().id, "render", error.what()});
                warp = original;
                if (report_.requested != "auto")
                    throw;
                active_.reset();
                activate_next();
            }
        }
    }
    RenderingReport report() const override {
        auto result = report_;
        size_t used = 0;
        for (const auto& backend : result.backends) {
            if (backend.frames) {
                result.backend = backend.info.id;
                ++used;
            }
        }
        if (used > 1)
            result.backend = "mixed";
        if (!used && active_)
            result.backend = active_->info().id;
        return result;
    }

  private:
    void activate_next() {
        while (next_ < candidates_.size()) {
            const auto& provider = candidates_[next_++];
            try {
                auto backend = provider.create();
                if (!backend || backend->info().id != provider.id)
                    throw std::logic_error("Backend provider returned an invalid identity: " + provider.id);
                active_ = std::make_unique<CropRenderer>(std::move(backend));
                report_.backends.push_back({active_->info(), 0, provider.gpu});
                return;
            } catch (const BackendError& error) {
                report_.failures.push_back({provider.id, "initialize", error.what()});
                if (report_.requested != "auto")
                    throw;
            }
        }
        std::string message = "No rendering backend available";
        for (const auto& failure : report_.failures)
            message += "; " + failure.backend + ": " + failure.reason;
        throw BackendError(message);
    }
    std::vector<BackendProvider> candidates_;
    size_t next_ = 0;
    std::unique_ptr<CropRenderer> active_;
    RenderingReport report_;
};

class RegisteredRendererFactory final : public RendererFactory {
  public:
    explicit RegisteredRendererFactory(std::vector<BackendProvider> providers) : providers_(std::move(providers)) {
        std::set<std::string> ids;
        for (const auto& provider : providers_) {
            if (provider.id.empty() || provider.id == "auto" || !provider.create || !ids.insert(provider.id).second)
                throw std::invalid_argument("Invalid or duplicate backend registration: " + provider.id);
        }
    }
    std::unique_ptr<Renderer> create(const std::string& requested) const override {
        if (requested == "auto")
            return std::make_unique<FallbackRenderer>(requested, providers_);
        const auto found = std::find_if(providers_.begin(), providers_.end(),
                                        [&](const auto& provider) { return provider.id == requested; });
        if (found == providers_.end())
            throw std::invalid_argument("Rendering backend is not registered: " + requested);
        return std::make_unique<FallbackRenderer>(requested, std::vector<BackendProvider>{*found});
    }
    bool prefers_gpu(const std::string& requested) const override {
        const auto found = std::find_if(providers_.begin(), providers_.end(), [&](const auto& provider) {
            return requested == "auto" || provider.id == requested;
        });
        return found != providers_.end() && found->gpu;
    }

  private:
    std::vector<BackendProvider> providers_;
};
} // namespace
std::unique_ptr<RendererFactory> make_renderer_factory(std::vector<BackendProvider> providers) {
    return std::make_unique<RegisteredRendererFactory>(std::move(providers));
}
} // namespace stillflow
