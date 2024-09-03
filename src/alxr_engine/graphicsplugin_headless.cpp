#include "pch.h"
#include "common.h"
#include "geometry.h"
#include "graphicsplugin.h"

struct HeadlessGraphicsPlugin final : public IGraphicsPlugin {

    HeadlessGraphicsPlugin(const std::shared_ptr<Options>&, std::shared_ptr<IPlatformPlugin>) {};
    
    virtual std::vector<std::string> GetInstanceExtensions() const override { return {}; }

    // Create an instance of this graphics api for the provided instance and systemId.
    virtual void InitializeDevice(XrInstance /*instance*/, XrSystemId /*systemId*/, const XrEnvironmentBlendMode /*newMode*/, const bool /*enableVisibilityMask*/) override { return ; }

    // Select the preferred swapchain format from the list of available formats.
    virtual int64_t SelectColorSwapchainFormat(const std::vector<int64_t>& /*runtimeFormats*/) const override { return 0; }

    // Get the graphics binding header for session creation.
    virtual const XrBaseInStructure* GetGraphicsBinding() const override { return nullptr; }

    // Allocate space for the swapchain image structures. These are different for each graphics API. The returned
    // pointers are valid for the lifetime of the graphics plugin.
    virtual std::vector<XrSwapchainImageBaseHeader*> AllocateSwapchainImageStructs(
        uint32_t /*capacity*/, const XrSwapchainCreateInfo& /*swapchainCreateInfo*/) override {
        return {};
    }

    // Render to a swapchain image for a projection view.
    virtual void RenderView
    (
        const std::array<XrCompositionLayerProjectionView, 2>& /*layerViews*/,
        const std::array<XrSwapchainImageBaseHeader*, 2>& /*swapchainImages*/,
        const std::int64_t /*swapchainFormat*/,
        const PassthroughMode /*newMode*/,
        const std::vector<Cube>& /*cubes*/
    ) override {
        return ;
    }
};

std::shared_ptr<IGraphicsPlugin> CreateGraphicsPlugin_Headless(const std::shared_ptr<Options>& options,
    std::shared_ptr<IPlatformPlugin> platformPlugin) {
    return std::make_shared<HeadlessGraphicsPlugin>(options, platformPlugin);
}
