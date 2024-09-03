// Copyright (c) 2017-2022, The Khronos Group Inc.
//
// SPDX-License-Identifier: Apache-2.0

#include "pch.h"
#include "common.h"
#include "geometry.h"
#include "options.h"
#include "graphicsplugin.h"

#if defined(XR_USE_GRAPHICS_API_D3D11)

#include <array>
#include <vector>
#include <atomic>

#include "xr_eigen.h"
#include <DirectXColors.h>
#include <D3Dcompiler.h>

#include "d3d_common.h"
#include "d3d_fence_event.h"

#ifdef XR_ENABLE_CUDA_INTEROP
#include "cuda/d3d11cuda_interop.h"
#endif
#include "foveation.h"

using namespace Microsoft::WRL;
using namespace DirectX;

namespace {
void InitializeD3D11DeviceForAdapter(IDXGIAdapter1* adapter, const std::vector<D3D_FEATURE_LEVEL>& featureLevels,
                                     ID3D11Device** device, ID3D11DeviceContext** deviceContext,
                                     const UINT extraCreationFlags = 0) {
    UINT creationFlags = D3D11_CREATE_DEVICE_BGRA_SUPPORT | extraCreationFlags;

#if !defined(NDEBUG)
    creationFlags |= D3D11_CREATE_DEVICE_DEBUG;
#endif

    // Create the Direct3D 11 API device object and a corresponding context.
    D3D_DRIVER_TYPE driverType = ((adapter == nullptr) ? D3D_DRIVER_TYPE_HARDWARE : D3D_DRIVER_TYPE_UNKNOWN);
    Log::Write(Log::Level::Verbose, Fmt("Selected driver type: %d", static_cast<int>(driverType)));

TryAgain:
    HRESULT hr = D3D11CreateDevice(adapter, driverType, 0, creationFlags, featureLevels.data(), (UINT)featureLevels.size(),
                                   D3D11_SDK_VERSION, device, nullptr, deviceContext);
    if (FAILED(hr)) {
        // If initialization failed, it may be because device debugging isn't supported, so retry without that.
        if ((creationFlags & D3D11_CREATE_DEVICE_DEBUG) && (hr == DXGI_ERROR_SDK_COMPONENT_MISSING)) {
            creationFlags &= ~D3D11_CREATE_DEVICE_DEBUG;
            goto TryAgain;
        }

        // If the initialization still fails, fall back to the WARP device.
        // For more information on WARP, see: http://go.microsoft.com/fwlink/?LinkId=286690
        if (driverType != D3D_DRIVER_TYPE_WARP) {
            driverType = D3D_DRIVER_TYPE_WARP;
            goto TryAgain;
        }
    }
}

constexpr inline DXGI_FORMAT MapFormat(const XrPixelFormat pixfmt) {
    switch (pixfmt) {
    case XrPixelFormat::NV12: return DXGI_FORMAT_NV12;
    case XrPixelFormat::P010LE: return DXGI_FORMAT_P010;
    }
    return DXGI_FORMAT_UNKNOWN;
}

constexpr inline DXGI_FORMAT GetLumaFormat(const XrPixelFormat yuvFmt) {
    switch (yuvFmt) {
    case XrPixelFormat::G8_B8_R8_3PLANE_420:          return DXGI_FORMAT_R8_UNORM;
    case XrPixelFormat::G10X6_B10X6_R10X6_3PLANE_420: return DXGI_FORMAT_R16_UNORM;
    }
    return ALXR::GetLumaFormat(MapFormat(yuvFmt));
}

constexpr inline DXGI_FORMAT GetChromaFormat(const XrPixelFormat yuvFmt) {
    switch (yuvFmt) {
    case XrPixelFormat::G8_B8_R8_3PLANE_420:          return DXGI_FORMAT_R8G8_UNORM;
    case XrPixelFormat::G10X6_B10X6_R10X6_3PLANE_420: return DXGI_FORMAT_R16G16_UNORM;
    }
    return ALXR::GetChromaFormat(MapFormat(yuvFmt));
}

constexpr inline DXGI_FORMAT GetChromaUFormat(const XrPixelFormat yuvFmt) {
    switch (yuvFmt) {
    case XrPixelFormat::G8_B8_R8_3PLANE_420:          return DXGI_FORMAT_R8_UNORM;
    case XrPixelFormat::G10X6_B10X6_R10X6_3PLANE_420: return DXGI_FORMAT_R16_UNORM;
    }
    return GetChromaFormat(yuvFmt);
}

constexpr inline DXGI_FORMAT GetChromaVFormat(const XrPixelFormat yuvFmt) {
    switch (yuvFmt) {
    case XrPixelFormat::G8_B8_R8_3PLANE_420:          return DXGI_FORMAT_R8_UNORM;
    case XrPixelFormat::G10X6_B10X6_R10X6_3PLANE_420: return DXGI_FORMAT_R16_UNORM;
    }
    return GetChromaFormat(yuvFmt);
}

inline bool SetMultithreadProtected(ComPtr<ID3D11Device> device)
{
    if (device == nullptr)
        return false;
    ID3D10Multithread* pMultithread = nullptr;
    if (!SUCCEEDED(device->QueryInterface(__uuidof(ID3D10Multithread), (void**)&pMultithread))
        || pMultithread == nullptr)
        return false;
    pMultithread->SetMultithreadProtected(TRUE);
    pMultithread->Release();
    return true;
}

struct D3D11GraphicsPlugin final : public IGraphicsPlugin {
    
    using VideoPShader = ALXR::VideoPShader;

    struct D3D11ShaderByteCode {
        const void* data;
        std::size_t size;
    };
    using CoreShaders = ALXR::CoreShaders<D3D11ShaderByteCode>;

    std::shared_ptr<Options> m_options{};

    D3D11GraphicsPlugin(const std::shared_ptr<Options>& opts, std::shared_ptr<IPlatformPlugin>)
    : m_options{ opts } {
        assert(m_options != nullptr);
    }

    std::vector<std::string> GetInstanceExtensions() const override { return {XR_KHR_D3D11_ENABLE_EXTENSION_NAME}; }

    using FeatureLvlList = std::vector<D3D_FEATURE_LEVEL>;

    void InitializeDevice(XrInstance instance, XrSystemId systemId, const XrEnvironmentBlendMode newMode, const bool /*enableVisibilityMask*/) override {
        PFN_xrGetD3D11GraphicsRequirementsKHR pfnGetD3D11GraphicsRequirementsKHR = nullptr;
        CHECK_XRCMD(xrGetInstanceProcAddr(instance, "xrGetD3D11GraphicsRequirementsKHR",
                                          reinterpret_cast<PFN_xrVoidFunction*>(&pfnGetD3D11GraphicsRequirementsKHR)));

        // Create the D3D11 device for the adapter associated with the system.
        XrGraphicsRequirementsD3D11KHR graphicsRequirements{.type=XR_TYPE_GRAPHICS_REQUIREMENTS_D3D11_KHR, .next=nullptr};
        CHECK_XRCMD(pfnGetD3D11GraphicsRequirementsKHR(instance, systemId, &graphicsRequirements));
        const ComPtr<IDXGIAdapter1> adapter = ALXR::GetAdapter(graphicsRequirements.adapterLuid);
        if (adapter == nullptr) {
            Log::Write(Log::Level::Warning, "Failed to find suitable adaptor, client will fallback to an unknown device type.");
        }
        m_d3d11DeviceLUID = graphicsRequirements.adapterLuid;
        // Create a list of feature levels which are both supported by the OpenXR runtime and this application.
        FeatureLvlList featureLevels = {D3D_FEATURE_LEVEL_12_1, D3D_FEATURE_LEVEL_12_0, D3D_FEATURE_LEVEL_11_1,
                                        D3D_FEATURE_LEVEL_11_0, D3D_FEATURE_LEVEL_10_1, D3D_FEATURE_LEVEL_10_0};
        featureLevels.erase(std::remove_if(featureLevels.begin(), featureLevels.end(),
                                           [&](D3D_FEATURE_LEVEL fl) { return fl < graphicsRequirements.minFeatureLevel; }),
                            featureLevels.end());
        CHECK_MSG(featureLevels.size() != 0, "Unsupported minimum feature level!");

        InitializeD3D11DeviceForAdapter(adapter.Get(), featureLevels, m_device.ReleaseAndGetAddressOf(),
                                        m_deviceContext.ReleaseAndGetAddressOf());
        SetMultithreadProtected(m_device);
        InitializeD3D11VADevice(adapter, featureLevels);
        
        InitializeResources();
        CHECK(m_coreShaders.IsValid());

        m_graphicsBinding.device = m_device.Get();

        SetEnvironmentBlendMode(newMode);
    }

    void InitializeD3D11VADevice(const ComPtr<IDXGIAdapter1>& adapter, const FeatureLvlList& featureLevels)
    {
        InitializeD3D11DeviceForAdapter
        (
            adapter.Get(), featureLevels, m_d3d11va_device.ReleaseAndGetAddressOf(),
            nullptr, D3D11_CREATE_DEVICE_VIDEO_SUPPORT
        );
        SetMultithreadProtected(m_d3d11va_device);
    }

    void InitializeVideoRenderResources()
    {
        CHECK(m_device != nullptr);

        ComPtr<ID3D11Device5> d3d11device5;
        m_device.As(&d3d11device5);
        CHECK(d3d11device5 != nullptr);

        m_texRendereComplete.CreateFence(d3d11device5, D3D11_FENCE_FLAG_SHARED);
        m_texCopy.CreateFence(d3d11device5, D3D11_FENCE_FLAG_SHARED);
#ifdef XR_ENABLE_CUDA_INTEROP
        InitCuda();
#endif
        CHECK_HRCMD(m_device->CreateDeferredContext(0, m_uploadContext.ReleaseAndGetAddressOf()));
        CHECK(m_uploadContext != nullptr);

        CHECK(m_coreShaders.IsValid());
        
        CHECK_HRCMD(m_device->CreateVertexShader
        (
            m_coreShaders.videoVS.data(), m_coreShaders.videoVS.size(), nullptr,
            m_videoVertexShader.ReleaseAndGetAddressOf())
        );

        std::size_t shaderIndex = 0;
        for (const auto& videoPixelShader : m_coreShaders.videoPSMap[0])
        {
            CHECK_HRCMD(m_device->CreatePixelShader(videoPixelShader.data(), videoPixelShader.size(), nullptr,
                m_videoPixelShader[shaderIndex++].ReleaseAndGetAddressOf()));
        }

        // Create the sample state
        const D3D11_SAMPLER_DESC sampDesc {
            .Filter = D3D11_FILTER_MIN_MAG_MIP_LINEAR,
            .AddressU = D3D11_TEXTURE_ADDRESS_CLAMP,
            .AddressV = D3D11_TEXTURE_ADDRESS_CLAMP,
            .AddressW = D3D11_TEXTURE_ADDRESS_CLAMP,
            .ComparisonFunc = D3D11_COMPARISON_NEVER,
            .MinLOD = 0,
            .MaxLOD = D3D11_FLOAT32_MAX,
        };
        CHECK_HRCMD(m_device->CreateSamplerState(&sampDesc, m_lumaSampler.ReleaseAndGetAddressOf()));
        CHECK_HRCMD(m_device->CreateSamplerState(&sampDesc, m_chromaSampler.ReleaseAndGetAddressOf()));
    }

    void InitializeResources() {
        D3D11_FEATURE_DATA_D3D11_OPTIONS3 options{
            .VPAndRTArrayIndexFromAnyShaderFeedingRasterizer = false
        };
        if (SUCCEEDED(m_device->CheckFeatureSupport(D3D11_FEATURE_D3D11_OPTIONS3, &options, sizeof(options)))) {
            m_isMultiViewSupported = options.VPAndRTArrayIndexFromAnyShaderFeedingRasterizer;
        }

        CoreShaders::Path smDir = m_isMultiViewSupported ? "SM5/multivew" : "SM5";
        m_coreShaders = { smDir, m_options->InternalDataPath };

        const auto& lobbyVS = m_coreShaders.lobbyVS;
        CHECK_HRCMD(m_device->CreateVertexShader(lobbyVS.data(), lobbyVS.size(), nullptr, m_vertexShader.ReleaseAndGetAddressOf()));

        const auto& lobbyPS = m_coreShaders.lobbyPS;
        CHECK_HRCMD(m_device->CreatePixelShader(lobbyPS.data(), lobbyPS.size(), nullptr,
                                                m_pixelShader.ReleaseAndGetAddressOf()));

        const D3D11_INPUT_ELEMENT_DESC vertexDesc[] = {
            {"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0},
            {"COLOR", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0},
        };
        CHECK_HRCMD(m_device->CreateInputLayout(vertexDesc, (UINT)std::size(vertexDesc), lobbyVS.data(), lobbyVS.size(), &m_inputLayout));

        const CD3D11_BUFFER_DESC modelConstantBufferDesc(sizeof(ALXR::ModelConstantBuffer), D3D11_BIND_CONSTANT_BUFFER);
        CHECK_HRCMD(m_device->CreateBuffer(&modelConstantBufferDesc, nullptr, m_modelCBuffer.ReleaseAndGetAddressOf()));

        const CD3D11_BUFFER_DESC viewProjectionConstantBufferDesc
        (
            m_isMultiViewSupported ? sizeof(ALXR::MultiViewProjectionConstantBuffer) : sizeof(ALXR::ViewProjectionConstantBuffer),
            D3D11_BIND_CONSTANT_BUFFER
        );
        CHECK_HRCMD(
            m_device->CreateBuffer(&viewProjectionConstantBufferDesc, nullptr, m_viewProjectionCBuffer.ReleaseAndGetAddressOf()));

        const CD3D11_BUFFER_DESC fovDecodeBufferDesc(sizeof(ALXR::FoveatedDecodeParams), D3D11_BIND_CONSTANT_BUFFER);
        CHECK_HRCMD(m_device->CreateBuffer(&fovDecodeBufferDesc, nullptr, m_fovDecodeCBuffer.ReleaseAndGetAddressOf()));

        const D3D11_SUBRESOURCE_DATA vertexBufferData{Geometry::c_cubeVertices};
        const CD3D11_BUFFER_DESC vertexBufferDesc(sizeof(Geometry::c_cubeVertices), D3D11_BIND_VERTEX_BUFFER);
        CHECK_HRCMD(m_device->CreateBuffer(&vertexBufferDesc, &vertexBufferData, m_cubeVertexBuffer.ReleaseAndGetAddressOf()));

        const D3D11_SUBRESOURCE_DATA indexBufferData{Geometry::c_cubeIndices};
        const CD3D11_BUFFER_DESC indexBufferDesc(sizeof(Geometry::c_cubeIndices), D3D11_BIND_INDEX_BUFFER);
        CHECK_HRCMD(m_device->CreateBuffer(&indexBufferDesc, &indexBufferData, m_cubeIndexBuffer.ReleaseAndGetAddressOf()));

        InitializeVideoRenderResources();
    }

    int64_t SelectColorSwapchainFormat(const std::vector<int64_t>& runtimeFormats) const override {
        // List of supported color swapchain formats, ordered by preference.
        constexpr const DXGI_FORMAT SupportedColorSwapchainFormats[] = {
            DXGI_FORMAT_R8G8B8A8_UNORM_SRGB,
            DXGI_FORMAT_B8G8R8A8_UNORM_SRGB,
            DXGI_FORMAT_R8G8B8A8_UNORM,
            DXGI_FORMAT_B8G8R8A8_UNORM,
        };
        for (const auto acceptedFormat : SupportedColorSwapchainFormats) {
            const auto swapchainFormatIt = std::find(runtimeFormats.begin(), runtimeFormats.end(), acceptedFormat);
            if (swapchainFormatIt != runtimeFormats.end()) {
                assert(acceptedFormat == *swapchainFormatIt);
                return acceptedFormat;
            }
        }
        return 0;
    }

    const XrBaseInStructure* GetGraphicsBinding() const override {
        return reinterpret_cast<const XrBaseInStructure*>(&m_graphicsBinding);
    }

    std::vector<XrSwapchainImageBaseHeader*> AllocateSwapchainImageStructs(
        uint32_t capacity, const XrSwapchainCreateInfo& /*swapchainCreateInfo*/) override {
        // Allocate and initialize the buffer of image structs (must be sequential in memory for xrEnumerateSwapchainImages).
        // Return back an array of pointers to each swapchain image struct so the consumer doesn't need to know the type/size.
        std::vector<XrSwapchainImageD3D11KHR> swapchainImageBuffer(capacity, {
            .type = XR_TYPE_SWAPCHAIN_IMAGE_D3D11_KHR,
            .next = nullptr,
            .texture = nullptr
        });
        std::vector<XrSwapchainImageBaseHeader*> swapchainImageBase;
        swapchainImageBase.reserve(capacity);
        for (XrSwapchainImageD3D11KHR& image : swapchainImageBuffer) {
            swapchainImageBase.push_back(reinterpret_cast<XrSwapchainImageBaseHeader*>(&image));
        }

        // Keep the buffer alive by moving it into the list of buffers.
        m_swapchainImageBuffers.push_back(std::move(swapchainImageBuffer));

        return swapchainImageBase;
    }

    ComPtr<ID3D11DepthStencilView> GetDepthStencilView(ID3D11Texture2D* colorTexture, const D3D11_DSV_DIMENSION viewDimension = D3D11_DSV_DIMENSION_TEXTURE2D) {
        // If a depth-stencil view has already been created for this back-buffer, use it.
        auto depthBufferIt = m_colorToDepthMap.find(colorTexture);
        if (depthBufferIt != m_colorToDepthMap.end()) {
            return depthBufferIt->second;
        }
        assert(colorTexture != nullptr);

        // This back-buffer has no corresponding depth-stencil texture, so create one with matching dimensions.
        D3D11_TEXTURE2D_DESC colorDesc;
        colorTexture->GetDesc(&colorDesc);

        const D3D11_TEXTURE2D_DESC depthDesc {
            .Width = colorDesc.Width,
            .Height = colorDesc.Height,
            .MipLevels = 1,
            .ArraySize = colorDesc.ArraySize,            
            .Format = DXGI_FORMAT_R32_TYPELESS,
            .SampleDesc {.Count = 1,.Quality = 0},
            .Usage = D3D11_USAGE_DEFAULT,
            .BindFlags = D3D11_BIND_SHADER_RESOURCE | D3D11_BIND_DEPTH_STENCIL,
            .CPUAccessFlags = 0,
            .MiscFlags = 0
        };
        ComPtr<ID3D11Texture2D> depthTexture;
        CHECK_HRCMD(m_device->CreateTexture2D(&depthDesc, nullptr, depthTexture.ReleaseAndGetAddressOf()));

        // Create and cache the depth stencil view.
        ComPtr<ID3D11DepthStencilView> depthStencilView;
        const CD3D11_DEPTH_STENCIL_VIEW_DESC depthStencilViewDesc(viewDimension, DXGI_FORMAT_D32_FLOAT);
        CHECK_HRCMD(m_device->CreateDepthStencilView(depthTexture.Get(), &depthStencilViewDesc, depthStencilView.GetAddressOf()));
        depthBufferIt = m_colorToDepthMap.insert(std::make_pair(colorTexture, depthStencilView)).first;

        return depthStencilView;
    }

    virtual void ClearSwapchainImageStructs() override
    {
        m_colorToDepthMap.clear();
        m_swapchainImageBuffers.clear();
    }

    template < typename RenderFun >
    void RenderMultiViewImpl(const XrCompositionLayerProjectionView& layerView, const XrSwapchainImageBaseHeader* swapchainImage,
        int64_t swapchainFormat, const ALXR::CColorType& clearColour, RenderFun&& renderFn) {
        assert(IsMultiViewEnabled());

        ID3D11Texture2D* const colorTexture = reinterpret_cast<const XrSwapchainImageD3D11KHR*>(swapchainImage)->texture;

        const CD3D11_VIEWPORT viewport((float)layerView.subImage.imageRect.offset.x, (float)layerView.subImage.imageRect.offset.y,
            (float)layerView.subImage.imageRect.extent.width,
            (float)layerView.subImage.imageRect.extent.height);
        m_deviceContext->RSSetViewports(1, &viewport);

        // Create RenderTargetView with original swapchain format (swapchain is typeless).
        ComPtr<ID3D11RenderTargetView> renderTargetView;
        const CD3D11_RENDER_TARGET_VIEW_DESC renderTargetViewDesc(D3D11_RTV_DIMENSION_TEXTURE2DARRAY, (DXGI_FORMAT)swapchainFormat);
        CHECK_HRCMD(
            m_device->CreateRenderTargetView(colorTexture, &renderTargetViewDesc, renderTargetView.ReleaseAndGetAddressOf()));

        const ComPtr<ID3D11DepthStencilView> depthStencilView = GetDepthStencilView(colorTexture, D3D11_DSV_DIMENSION_TEXTURE2DARRAY);

        // Clear swapchain and depth buffer. NOTE: This will clear the entire render target view, not just the specified view.
        // TODO: Do not clear to a color when using a pass-through view configuration.
        m_deviceContext->ClearRenderTargetView(renderTargetView.Get(), clearColour);//);
        m_deviceContext->ClearDepthStencilView(depthStencilView.Get(), D3D11_CLEAR_DEPTH | D3D11_CLEAR_STENCIL, 1.0f, 0);

        ID3D11RenderTargetView* renderTargets[] = { renderTargetView.Get() };
        m_deviceContext->OMSetRenderTargets((UINT)std::size(renderTargets), renderTargets, depthStencilView.Get());

        renderFn();
    }

    static inline DirectX::XMFLOAT4X4A XM_CALLCONV MakeViewProjMatrix(const XrCompositionLayerProjectionView& layerView)
    {
        const XMMATRIX spaceToView = XMMatrixInverse(nullptr, ALXR::LoadXrPose(layerView.pose));
        Eigen::Matrix4f projectionMatrix = ALXR::CreateProjectionFov(ALXR::GraphicsAPI::D3D, layerView.fov, 0.05f, 100.0f);
        DirectX::XMFLOAT4X4A viewProj;
        XMStoreFloat4x4(&viewProj, XMMatrixTranspose(spaceToView * ALXR::LoadXrMatrix(projectionMatrix)));
        return viewProj;
    }

    virtual void RenderMultiView
    (
        const std::array<XrCompositionLayerProjectionView, 2>& layerViews,
        const XrSwapchainImageBaseHeader* swapchainImage,
        const std::int64_t swapchainFormat, const PassthroughMode mode,
        const std::vector<Cube>& cubes
    ) override
    {
        RenderMultiViewImpl(layerViews[0], swapchainImage, swapchainFormat, ALXR::ClearColors[ClearColorIndex(mode)], [&]()
        {
            ALXR::MultiViewProjectionConstantBuffer viewProjections;
            for (std::uint32_t viewIndex = 0; viewIndex < 2; ++viewIndex) {
                viewProjections.ViewProjection[viewIndex] = MakeViewProjMatrix(layerViews[viewIndex]);
            }
            m_deviceContext->UpdateSubresource(m_viewProjectionCBuffer.Get(), 0, nullptr, &viewProjections, 0, 0);

            ID3D11Buffer* const constantBuffers[] = { m_modelCBuffer.Get(), m_viewProjectionCBuffer.Get() };
            m_deviceContext->VSSetConstantBuffers(0, (UINT)std::size(constantBuffers), constantBuffers);
            m_deviceContext->VSSetShader(m_vertexShader.Get(), nullptr, 0);
            m_deviceContext->PSSetShader(m_pixelShader.Get(), nullptr, 0);

            // Set cube primitive data.
            constexpr static const UINT strides[] = { sizeof(Geometry::Vertex) };
            constexpr static const UINT offsets[] = { 0 };
            ID3D11Buffer* vertexBuffers[] = { m_cubeVertexBuffer.Get() };
            m_deviceContext->IASetVertexBuffers(0, (UINT)std::size(vertexBuffers), vertexBuffers, strides, offsets);
            m_deviceContext->IASetIndexBuffer(m_cubeIndexBuffer.Get(), DXGI_FORMAT_R16_UINT, 0);
            m_deviceContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
            m_deviceContext->IASetInputLayout(m_inputLayout.Get());

            // Render each cube
            for (const Cube& cube : cubes) {
                // Compute and update the model transform.
                ALXR::ModelConstantBuffer model;
                XMStoreFloat4x4(&model.Model,
                    XMMatrixTranspose(XMMatrixScaling(cube.Scale.x, cube.Scale.y, cube.Scale.z) * ALXR::LoadXrPose(cube.Pose)));
                m_deviceContext->UpdateSubresource(m_modelCBuffer.Get(), 0, nullptr, &model, 0, 0);

                // Draw the cube.
                m_deviceContext->DrawIndexedInstanced((UINT)std::size(Geometry::c_cubeIndices), 2, 0, 0, 0);
            }
        });
    }

    virtual void RenderVideoMultiView
    (
        const std::array<XrCompositionLayerProjectionView, 2>& layerViews,
        const XrSwapchainImageBaseHeader* swapchainImage, const std::int64_t swapchainFormat,
        const PassthroughMode newMode /*= PassthroughMode::None*/
    ) override
    {
        RenderMultiViewImpl(layerViews[0], swapchainImage, swapchainFormat, ALXR::VideoClearColors[ClearColorIndex(newMode)], [&]()
        {
            if (currentTextureIdx == std::size_t(-1))
                return;
            const auto& videoTex = m_videoTextures[currentTextureIdx];

            m_deviceContext->VSSetShader(m_videoVertexShader.Get(), nullptr, 0);

            if (const auto fovDecParmPtr = m_fovDecodeParams) {
                alignas(16) const ALXR::FoveatedDecodeParams fdParam = *fovDecParmPtr;
                m_deviceContext->UpdateSubresource(m_fovDecodeCBuffer.Get(), 0, nullptr, &fdParam, 0, 0);
                ID3D11Buffer* const psConstantBuffers[] = { m_fovDecodeCBuffer.Get() };
                m_deviceContext->PSSetConstantBuffers(2, (UINT)std::size(psConstantBuffers), psConstantBuffers);
            }
            const bool is3PlaneFormat = videoTex.chromaVSRV != nullptr;
            m_deviceContext->PSSetShader(m_videoPixelShader[VideoShaderIndex(is3PlaneFormat, newMode)].Get(), nullptr, 0);

            const std::array<ID3D11ShaderResourceView*, 3> srvs{
                videoTex.lumaSRV.Get(),
                videoTex.chromaSRV.Get(),
                videoTex.chromaVSRV.Get()
            };
            const UINT srvSize = is3PlaneFormat ? (UINT)srvs.size() : 2u;
            m_deviceContext->PSSetShaderResources(0, srvSize, srvs.data());

            const std::array<ID3D11SamplerState*, 2> samplers = { m_lumaSampler.Get(), m_chromaSampler.Get() };
            m_deviceContext->PSSetSamplers(0, (UINT)samplers.size(), samplers.data());

            m_deviceContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
            m_deviceContext->IASetInputLayout(nullptr);
            m_deviceContext->DrawInstanced(3u, 2u, 0u, 0u);
        });
    }

    template < typename RenderFun >
    void RenderViewImpl(const XrCompositionLayerProjectionView& layerView, const XrSwapchainImageBaseHeader* swapchainImage,
                    int64_t swapchainFormat, const ALXR::CColorType& clearColour, RenderFun&& renderFn) {
        CHECK(layerView.subImage.imageArrayIndex == 0);  // Texture arrays not supported.
        assert(!IsMultiViewEnabled());

        ID3D11Texture2D* const colorTexture = reinterpret_cast<const XrSwapchainImageD3D11KHR*>(swapchainImage)->texture;

        const CD3D11_VIEWPORT viewport( (float)layerView.subImage.imageRect.offset.x, (float)layerView.subImage.imageRect.offset.y,
                                        (float)layerView.subImage.imageRect.extent.width,
                                        (float)layerView.subImage.imageRect.extent.height);
        m_deviceContext->RSSetViewports(1, &viewport);

        // Create RenderTargetView with original swapchain format (swapchain is typeless).
        ComPtr<ID3D11RenderTargetView> renderTargetView;
        const CD3D11_RENDER_TARGET_VIEW_DESC renderTargetViewDesc(D3D11_RTV_DIMENSION_TEXTURE2D, (DXGI_FORMAT)swapchainFormat);
        CHECK_HRCMD(
            m_device->CreateRenderTargetView(colorTexture, &renderTargetViewDesc, renderTargetView.ReleaseAndGetAddressOf()));

        const ComPtr<ID3D11DepthStencilView> depthStencilView = GetDepthStencilView(colorTexture);

        // Clear swapchain and depth buffer. NOTE: This will clear the entire render target view, not just the specified view.
        // TODO: Do not clear to a color when using a pass-through view configuration.
        m_deviceContext->ClearRenderTargetView(renderTargetView.Get(), clearColour);//);
        m_deviceContext->ClearDepthStencilView(depthStencilView.Get(), D3D11_CLEAR_DEPTH | D3D11_CLEAR_STENCIL, 1.0f, 0);

        ID3D11RenderTargetView* renderTargets[] = {renderTargetView.Get()};
        m_deviceContext->OMSetRenderTargets((UINT)std::size(renderTargets), renderTargets, depthStencilView.Get());

        renderFn();
    }

    inline std::size_t ClearColorIndex(const PassthroughMode /*ptMode*/) const {
        static_assert(ALXR::ClearColors.size() >= 3);
        static_assert(ALXR::VideoClearColors.size() >= 3);
        return m_clearColorIndex;
    }

    virtual void RenderView
    (
        const std::array<XrCompositionLayerProjectionView, 2>& layerViews,
        const std::array<XrSwapchainImageBaseHeader*, 2>& swapchainImages,
        const std::int64_t swapchainFormat, const PassthroughMode mode,
        const std::vector<Cube>& cubes
    ) override
    {
        for (std::size_t idx = 0; idx < layerViews.size(); ++idx) {
            const auto& layerView = layerViews[idx];
            RenderViewImpl(layerView, swapchainImages[idx], swapchainFormat, ALXR::ClearColors[ClearColorIndex(mode)], [&]()
            {
                // Set shaders and constant buffers.
                ALXR::ViewProjectionConstantBuffer viewProjection{
                    .ViewProjection = MakeViewProjMatrix(layerView),
                };
                m_deviceContext->UpdateSubresource(m_viewProjectionCBuffer.Get(), 0, nullptr, &viewProjection, 0, 0);

                ID3D11Buffer* const constantBuffers[] = { m_modelCBuffer.Get(), m_viewProjectionCBuffer.Get() };
                m_deviceContext->VSSetConstantBuffers(0, (UINT)std::size(constantBuffers), constantBuffers);
                m_deviceContext->VSSetShader(m_vertexShader.Get(), nullptr, 0);
                m_deviceContext->PSSetShader(m_pixelShader.Get(), nullptr, 0);

                // Set cube primitive data.
                constexpr static const UINT strides[] = { sizeof(Geometry::Vertex) };
                constexpr static const UINT offsets[] = { 0 };
                ID3D11Buffer* vertexBuffers[] = { m_cubeVertexBuffer.Get() };
                m_deviceContext->IASetVertexBuffers(0, (UINT)std::size(vertexBuffers), vertexBuffers, strides, offsets);
                m_deviceContext->IASetIndexBuffer(m_cubeIndexBuffer.Get(), DXGI_FORMAT_R16_UINT, 0);
                m_deviceContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
                m_deviceContext->IASetInputLayout(m_inputLayout.Get());

                // Render each cube
                for (const Cube& cube : cubes) {
                    // Compute and update the model transform.
                    ALXR::ModelConstantBuffer model;
                    XMStoreFloat4x4(&model.Model,
                        XMMatrixTranspose(XMMatrixScaling(cube.Scale.x, cube.Scale.y, cube.Scale.z) * ALXR::LoadXrPose(cube.Pose)));
                    m_deviceContext->UpdateSubresource(m_modelCBuffer.Get(), 0, nullptr, &model, 0, 0);

                    // Draw the cube.
                    m_deviceContext->DrawIndexed((UINT)std::size(Geometry::c_cubeIndices), 0, 0);
                }
            });
        }
    }

    uint32_t GetSupportedSwapchainSampleCount(const XrViewConfigurationView&) override { return 1; }

    virtual const void* GetD3D11AVDevice() const override {
        CHECK(m_d3d11va_device != nullptr);
        return m_d3d11va_device.Get();
    }

    virtual void* GetD3D11AVDevice() override {
        CHECK(m_d3d11va_device != nullptr);
        return m_d3d11va_device.Get();
    }

    virtual void ClearVideoTextures() override
    {
        m_renderTex = std::size_t(-1);
        m_currentVideoTex = 0;
        //std::lock_guard<std::mutex> lk(m_renderMutex);
        m_texRendereComplete.WaitForGpu();
        m_videoTextures = { NV12Texture {}, NV12Texture {} };
    }

    void CreateVideoTextures
    (
        const std::size_t width, const std::size_t height, const DXGI_FORMAT pixfmt,
        //const bool createUploadBuffer,
        const D3D11_BIND_FLAG binding_flags = D3D11_BIND_SHADER_RESOURCE,
        const D3D11_USAGE res_flags = D3D11_USAGE_DEFAULT,
        const UINT miscFlags = 0,
        const UINT cpuAccessFlags = 0
    )
    {
        if (m_device == nullptr)
            return;
        CHECK(width % 2 == 0);
        
        ClearVideoTextures();
        for (auto& vidTex : m_videoTextures)
        {
            const D3D11_TEXTURE2D_DESC descDepth {
                .Width = static_cast<UINT> (width),
                .Height = static_cast<UINT>(height),
                .MipLevels = 1,
                .ArraySize = 1,
                .Format = pixfmt,
                .SampleDesc {
                    .Count = 1,
                    .Quality = 0
                },
                .Usage = res_flags,
                .BindFlags = static_cast<UINT>(binding_flags),// | D3D11_BIND_UNORDERED_ACCESS,
                .CPUAccessFlags = cpuAccessFlags,
                .MiscFlags = miscFlags//D3D11_RESOURCE_MISC_SHARED;// D3D11_RESOURCE_MISC_SHARED_KEYEDMUTEX,
            };
            ComPtr<ID3D11Texture2D> newTex{};
            if (FAILED(m_device->CreateTexture2D(&descDepth, nullptr, newTex.ReleaseAndGetAddressOf())))
            {
                Log::Write(Log::Level::Info, "CreateTexture2D Failed");
                return /*nullptr*/;
            }
            vidTex.data.push_back(newTex);

            const auto lumaPlaneSRVDesc = CD3D11_SHADER_RESOURCE_VIEW_DESC
            (
                newTex.Get(),
                D3D11_SRV_DIMENSION_TEXTURE2D,
                ALXR::GetLumaFormat(descDepth.Format)
            );
            CHECK_HRCMD(m_device->CreateShaderResourceView(newTex.Get(), &lumaPlaneSRVDesc, vidTex.lumaSRV.ReleaseAndGetAddressOf()));

            const auto chromaPlaneSRVDesc = CD3D11_SHADER_RESOURCE_VIEW_DESC
            (
                newTex.Get(),
                D3D11_SRV_DIMENSION_TEXTURE2D,
                ALXR::GetChromaFormat(descDepth.Format)
            );
            CHECK_HRCMD(m_device->CreateShaderResourceView(newTex.Get(), &chromaPlaneSRVDesc, vidTex.chromaSRV.ReleaseAndGetAddressOf()));
        }
    }

    virtual void CreateVideoTextures(const std::size_t width, const std::size_t height, const XrPixelFormat pixfmt) override
    {
        // Mapping NV12/P010 formats for dynamic textures do not work, at least on Nvidia drivers
        // Workaround is to just create seperate textures for each plane.
#if 0
        CreateVideoTextures
        (
            width, height, MapFormat(pixfmt),
            D3D11_BIND_SHADER_RESOURCE,
            D3D11_USAGE_DYNAMIC,
            0,
            D3D11_CPU_ACCESS_WRITE
        );
#else
        if (m_device == nullptr)
            return;
        CHECK(width % 2 == 0);

        
        ClearVideoTextures();

        const bool is3PlaneFmt = PlaneCount(pixfmt) > 2;
        const auto lumaFormat = GetLumaFormat(pixfmt);
        const auto chromaFormat = GetChromaFormat(pixfmt);
        const auto chromaUFormat = GetChromaUFormat(pixfmt);
        const auto chromaVFormat = GetChromaVFormat(pixfmt);

        for (auto& vidTex : m_videoTextures)
        {
            const D3D11_TEXTURE2D_DESC lumaDesc {
                .Width = static_cast<UINT> (width),
                .Height = static_cast<UINT>(height),
                .MipLevels = 1,
                .ArraySize = 1,
                .Format = lumaFormat,
                .SampleDesc {
                    .Count = 1,
                    .Quality = 0
                },
                .Usage = D3D11_USAGE_DYNAMIC,
                .BindFlags = D3D11_BIND_SHADER_RESOURCE,
                .CPUAccessFlags = D3D11_CPU_ACCESS_WRITE,
                .MiscFlags = 0
            };
            ComPtr<ID3D11Texture2D> newLumaTex{};
            if (FAILED(m_device->CreateTexture2D(&lumaDesc, nullptr, newLumaTex.ReleaseAndGetAddressOf())))
            {
                Log::Write(Log::Level::Info, "CreateTexture2D Failed");
                return /*nullptr*/;
            }
            vidTex.data.push_back(newLumaTex);

            auto chromaDesc = lumaDesc;
            chromaDesc.Format = is3PlaneFmt ? chromaUFormat : chromaFormat;
            chromaDesc.Width /= 2;
            chromaDesc.Height /= 2;
            ComPtr<ID3D11Texture2D> newChromaTex{};
            if (FAILED(m_device->CreateTexture2D(&chromaDesc, nullptr, newChromaTex.ReleaseAndGetAddressOf())))
            {
                Log::Write(Log::Level::Info, "CreateTexture2D Failed");
                return /*nullptr*/;
            }
            vidTex.data.push_back(newChromaTex);

            if (is3PlaneFmt) {
                auto chromaVDesc = chromaDesc;
                chromaVDesc.Format = chromaVFormat;
                ComPtr<ID3D11Texture2D> newChromaVTex{};
                if (FAILED(m_device->CreateTexture2D(&chromaVDesc, nullptr, newChromaVTex.ReleaseAndGetAddressOf())))
                {
                    Log::Write(Log::Level::Info, "CreateTexture2D Failed");
                    return /*nullptr*/;
                }
                vidTex.data.push_back(newChromaVTex);

                const auto chromaPlaneSRVDesc = CD3D11_SHADER_RESOURCE_VIEW_DESC
                (
                    newChromaTex.Get(),
                    D3D11_SRV_DIMENSION_TEXTURE2D,
                    chromaVDesc.Format
                );
                CHECK_HRCMD(m_device->CreateShaderResourceView(newChromaVTex.Get(), &chromaPlaneSRVDesc, vidTex.chromaVSRV.ReleaseAndGetAddressOf()));
            }

            const auto lumaPlaneSRVDesc = CD3D11_SHADER_RESOURCE_VIEW_DESC
            (
                newLumaTex.Get(),
                D3D11_SRV_DIMENSION_TEXTURE2D,
                lumaDesc.Format
            );
            CHECK_HRCMD(m_device->CreateShaderResourceView(newLumaTex.Get(), &lumaPlaneSRVDesc, vidTex.lumaSRV.ReleaseAndGetAddressOf()));

            const auto chromaPlaneSRVDesc = CD3D11_SHADER_RESOURCE_VIEW_DESC
            (
                newChromaTex.Get(),
                D3D11_SRV_DIMENSION_TEXTURE2D,
                chromaDesc.Format
            );
            CHECK_HRCMD(m_device->CreateShaderResourceView(newChromaTex.Get(), &chromaPlaneSRVDesc, vidTex.chromaSRV.ReleaseAndGetAddressOf()));
        }
#endif
    }

    static constexpr inline std::size_t SizeOfFmt(const DXGI_FORMAT fmt)
    {
        switch (fmt) {
        case DXGI_FORMAT_R8_UINT:
        case DXGI_FORMAT_R8_UNORM:      return sizeof(std::uint8_t);
        case DXGI_FORMAT_R8G8_UINT:
        case DXGI_FORMAT_R8G8_UNORM:    return sizeof(std::uint8_t) * 2;
        case DXGI_FORMAT_R16_UINT:
        case DXGI_FORMAT_R16_UNORM:     return sizeof(std::uint16_t);
        case DXGI_FORMAT_R16G16_UINT:
        case DXGI_FORMAT_R16G16_UNORM:  return sizeof(std::uint16_t) * 2;
        }
        return 0;
    }

    virtual void UpdateVideoTexture(const YUVBuffer& yuvBuffer) override
    {
        CHECK(m_uploadContext != nullptr);

        const bool is3PlaneFmt = yuvBuffer.chroma2.data != nullptr;
        const std::size_t freeIndex = m_currentVideoTex.load();
        {
            using TexturePtr = ComPtr<ID3D11Texture2D>;
            /*const*/ auto& videoTex = m_videoTextures[freeIndex];
            TexturePtr lumaTexture = videoTex.data[0]; //videoTex.data.back();
            TexturePtr chromaTexture = videoTex.data[1];
            TexturePtr chromaVTexture{};
            if (is3PlaneFmt && videoTex.data.size() > 2) {
                chromaVTexture = videoTex.data[2];
                assert(chromaVTexture != nullptr);
            }            
            assert(lumaTexture != nullptr && chromaTexture != nullptr);

            constexpr const auto copy2d = []
            (
                std::uint8_t* dst, const std::size_t dstPitchInBytes,
                const std::uint8_t* src, const std::size_t srcPitchInBytes,
                const std::size_t width, const std::size_t height, const std::size_t formatSize
            )
            {
                if (dstPitchInBytes == srcPitchInBytes) {
                    std::memcpy(dst, src, dstPitchInBytes * height);
                    return;
                }
                const std::size_t lineDstSize = width * formatSize;
                for (std::size_t h = 0; h < height; ++h)
                {
                    auto lineDstPtr = dst + (dstPitchInBytes * h);
                    const auto lineSrcPtr = src + (srcPitchInBytes * h);
                    std::memcpy(lineDstPtr, lineSrcPtr, lineDstSize);
                    //std::copy_n(par_unseq, src, chromaLineSize, dst);
                }
            };
            const auto mapCopy2d = [&](const TexturePtr& dstTex, const Buffer& srcData)
            {
                D3D11_TEXTURE2D_DESC texDesc{};
                dstTex->GetDesc(&texDesc);
                const auto formatSize = SizeOfFmt(texDesc.Format);
                D3D11_MAPPED_SUBRESOURCE mappedResource{};
                CHECK_HRCMD(m_uploadContext->Map(dstTex.Get(), 0, D3D11_MAP_WRITE_DISCARD, 0, &mappedResource));
                {
                    copy2d
                    (
                        reinterpret_cast<std::uint8_t*>(mappedResource.pData), (std::size_t)mappedResource.RowPitch,
                        (const std::uint8_t*)srcData.data, srcData.pitch, (std::size_t)texDesc.Width, srcData.height, formatSize
                    );
                }
                m_uploadContext->Unmap(dstTex.Get(), 0);
            };

            mapCopy2d(lumaTexture, yuvBuffer.luma);
            mapCopy2d(chromaTexture, yuvBuffer.chroma);
            if (is3PlaneFmt && chromaVTexture != nullptr)
                mapCopy2d(chromaVTexture, yuvBuffer.chroma2);

            videoTex.frameIndex = yuvBuffer.frameIndex;

            CommandListPtr newCmdList{};
            CHECK_HRCMD(m_uploadContext->FinishCommandList(FALSE, newCmdList.ReleaseAndGetAddressOf()));
            videoTex.cmdList.Swap(newCmdList);
        }
        m_currentVideoTex.store((freeIndex + 1) % m_videoTextures.size());
        //CHECK_HRCMD(m_texCopy.Signal(m_videoTexCmdCpyQueue));
        m_renderTex.store(freeIndex);
    }

    virtual void CreateVideoTexturesD3D11VA(const std::size_t width, const std::size_t height, const XrPixelFormat pixfmt) override
    {
        CHECK(m_d3d11va_device != nullptr);
        CHECK_MSG((pixfmt != XrPixelFormat::G8_B8_R8_3PLANE_420 &&
            pixfmt != XrPixelFormat::G10X6_B10X6_R10X6_3PLANE_420), "3-Planes formats are not supported!");

        CreateVideoTextures
        (
            width, height, MapFormat(pixfmt),
            D3D11_BIND_SHADER_RESOURCE,
            D3D11_USAGE_DEFAULT,
            D3D11_RESOURCE_MISC_SHARED
        );
        for (auto& vidTex : m_videoTextures)
        {
            auto data = vidTex.data.back();
            CHECK(data != nullptr);

            ComPtr<IDXGIResource> dxgi_resource;
            CHECK_HRCMD(data->QueryInterface(__uuidof(IDXGIResource), reinterpret_cast<void**>(dxgi_resource.GetAddressOf())));
            CHECK(dxgi_resource != nullptr);
            HANDLE sharedHandle;
            dxgi_resource->GetSharedHandle(&sharedHandle);
            CHECK(sharedHandle);
            
            ComPtr<ID3D11Resource> shared_resource{};
            CHECK(sharedHandle);
            CHECK_HRCMD(m_d3d11va_device->OpenSharedResource(sharedHandle, __uuidof(ID3D11Resource), (void**)shared_resource.ReleaseAndGetAddressOf()));
            CHECK(shared_resource != nullptr);

            ComPtr<ID3D11Texture2D> shared_texture{};
            CHECK_HRCMD(shared_resource->QueryInterface(__uuidof(ID3D11Texture2D), (void**)shared_texture.ReleaseAndGetAddressOf()));

            vidTex.d3d11vaSharedData.push_back(shared_texture);
        }
    }

    virtual void UpdateVideoTextureD3D11VA(const YUVBuffer& yuvBuffer) override
    {
        CHECK(m_device != nullptr);
        CHECK(yuvBuffer.frameIndex != std::uint64_t(-1));

//        WaitForAvailableBuffer();

        const std::size_t freeIndex = m_currentVideoTex.load();
        {
            /*const*/ auto& videoTex = m_videoTextures[freeIndex];
            videoTex.frameIndex = yuvBuffer.frameIndex;
            auto dstVideoTexture = videoTex.d3d11vaSharedData.back();
            CHECK(dstVideoTexture != nullptr)
            
            D3D11_TEXTURE2D_DESC desc;
            dstVideoTexture->GetDesc(&desc);

            ComPtr<ID3D11Texture2D> src_texture = reinterpret_cast<ID3D11Texture2D*>(yuvBuffer.luma.data);
            const auto texture_index = (UINT)reinterpret_cast<std::intptr_t>(yuvBuffer.chroma.data);

            ID3D11DeviceContext* devCtx = nullptr;
            m_d3d11va_device->GetImmediateContext(&devCtx);
            CHECK(devCtx);

            const D3D11_BOX sourceRegion {
                .left = 0,
                .top = 0,
                .front = 0,
                .right = desc.Width,                
                .bottom = desc.Height,                
                .back = 1,
            };
            devCtx->CopySubresourceRegion(dstVideoTexture.Get(), 0, 0, 0, 0, src_texture.Get(), texture_index, &sourceRegion);
            // Flush to submit the 11 command list to the shared command queue.
            devCtx->Flush();
        }

        m_currentVideoTex.store((freeIndex + 1) % m_videoTextures.size());
        //CHECK_HRCMD(m_texCopy.Signal(m_videoTexCmdCpyQueue));
        m_renderTex.store(freeIndex);
    }

    virtual void BeginVideoView() override
    {
#if 0
#ifdef XR_ENABLE_CUDA_INTEROP
        const cudaExternalSemaphoreWaitParams externalSemaphoreWaitParams{
            .params{.fence{.value = m_texCopy.fenceValue.load()}},
            .flags = 0
        };
        if (cudaWaitExternalSemaphoresAsync(&m_texCopyExtSemaphore, &externalSemaphoreWaitParams, 1, videoBufferStream) != cudaSuccess)
        {
            Log::Write(Log::Level::Error, "cudaWaitExternalSemaphoresAsync failed.");
            CHECK(false);
        }
#endif
#endif
        currentTextureIdx = m_renderTex.load();
        if (currentTextureIdx == std::size_t(-1))
            return;
        auto& vidTex = m_videoTextures[currentTextureIdx];
        if (vidTex.cmdList != nullptr) {
            m_deviceContext->ExecuteCommandList(vidTex.cmdList.Get(), FALSE);
            vidTex.cmdList.Reset();
        }
    }

    virtual void EndVideoView() override
    {
#if 0
#ifdef XR_ENABLE_CUDA_INTEROP
        const auto nextVal = m_texRendereComplete.fenceValue.load() + 1;
        const cudaExternalSemaphoreSignalParams externalSemaphoreSignalParams{
            .params{.fence{.value = nextVal}},
            .flags = 0
        };
        if (cudaSignalExternalSemaphoresAsync(&m_texRenderExtSemaphore, &externalSemaphoreSignalParams, 1, videoBufferStream) != cudaSuccess)
        {
            Log::Write(Log::Level::Error, "m_texRendereComplete cudaSignalExternalSemaphoresAsync failed.");
            CHECK(false);
        }
        m_texRendereComplete.fenceValue.store(nextVal);
#endif
#endif
    }

    virtual std::uint64_t GetVideoFrameIndex() const override {
        return currentTextureIdx == std::uint64_t(-1) ?
            currentTextureIdx :
            m_videoTextures[currentTextureIdx].frameIndex;
    }

    constexpr static inline std::size_t VideoShaderIndex(const bool is3PlaneFmt, const PassthroughMode newMode) {
        return static_cast<const std::size_t>(newMode) + (is3PlaneFmt ? VideoPShader::Normal3Plane : VideoPShader::Normal);
    }

    virtual void RenderVideoView
    (
        const std::array<XrCompositionLayerProjectionView, 2>& layerViews,
        const std::array<XrSwapchainImageBaseHeader*, 2>& swapchainImages,
        const std::int64_t swapchainFormat,
        const PassthroughMode newMode /*= PassthroughMode::None*/
    ) override
    {
        for (std::uint32_t viewID = 0; viewID < layerViews.size(); ++viewID) {
            RenderViewImpl(layerViews[viewID], swapchainImages[viewID], swapchainFormat, ALXR::VideoClearColors[ClearColorIndex(newMode)], [&]()
            {
                if (currentTextureIdx == std::size_t(-1))
                    return;
                const auto& videoTex = m_videoTextures[currentTextureIdx];

                const ALXR::ViewProjectionConstantBuffer viewProjection{ .ViewID = viewID };
                m_deviceContext->UpdateSubresource(m_viewProjectionCBuffer.Get(), 0, nullptr, &viewProjection, 0, 0);

                ID3D11Buffer* const constantBuffers[] = { m_viewProjectionCBuffer.Get() };
                m_deviceContext->VSSetConstantBuffers(1, (UINT)std::size(constantBuffers), constantBuffers);
                m_deviceContext->VSSetShader(m_videoVertexShader.Get(), nullptr, 0);

                if (const auto fovDecParmPtr = m_fovDecodeParams) {
                    alignas(16) const ALXR::FoveatedDecodeParams fdParam = *fovDecParmPtr;
                    m_deviceContext->UpdateSubresource(m_fovDecodeCBuffer.Get(), 0, nullptr, &fdParam, 0, 0);
                    ID3D11Buffer* const psConstantBuffers[] = { m_fovDecodeCBuffer.Get() };
                    m_deviceContext->PSSetConstantBuffers(2, (UINT)std::size(psConstantBuffers), psConstantBuffers);
                }
                const bool is3PlaneFormat = videoTex.chromaVSRV != nullptr;
                m_deviceContext->PSSetShader(m_videoPixelShader[VideoShaderIndex(is3PlaneFormat, newMode)].Get(), nullptr, 0);

                const std::array<ID3D11ShaderResourceView*, 3> srvs{
                    videoTex.lumaSRV.Get(),
                    videoTex.chromaSRV.Get(),
                    videoTex.chromaVSRV.Get()
                };
                const UINT srvSize = is3PlaneFormat ? (UINT)srvs.size() : 2u;
                m_deviceContext->PSSetShaderResources(0, srvSize, srvs.data());

                const std::array<ID3D11SamplerState*, 2> samplers = { m_lumaSampler.Get(), m_chromaSampler.Get() };
                m_deviceContext->PSSetSamplers(0, (UINT)samplers.size(), samplers.data());

                m_deviceContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
                m_deviceContext->IASetInputLayout(nullptr);
                m_deviceContext->Draw(3, 0);
            });
        }
    }

    inline void SetEnvironmentBlendMode(const XrEnvironmentBlendMode newMode) {
        static_assert(XR_ENVIRONMENT_BLEND_MODE_OPAQUE == 1);
        static_assert(ALXR::ClearColors.size() >= 3);
        static_assert(ALXR::VideoClearColors.size() >= 3);
        assert(newMode > 0 && newMode < 4);
        m_clearColorIndex = newMode-1;
    }

    virtual inline bool IsMultiViewEnabled() const override {
        return m_isMultiViewSupported;
    }

    virtual void SetFoveatedDecode(const ALXR::FoveatedDecodeParams* newFovDecParmPtr) override {
        CHECK(m_device != nullptr);
        const auto fovDecodeParams = m_fovDecodeParams;
        const bool changePShaders  = (fovDecodeParams == nullptr && newFovDecParmPtr != nullptr) ||
                                     (fovDecodeParams != nullptr && newFovDecParmPtr == nullptr);
        if (changePShaders) {
            CHECK(m_coreShaders.IsValid());
            decltype(m_videoPixelShader) newVideoPixelShaders{};
            const auto& pixelShaderByteList = m_coreShaders.videoPSMap[newFovDecParmPtr ? 1 : 0];
            assert(pixelShaderByteList.size() == newVideoPixelShaders.size());
            std::size_t shaderIndex = 0;
            for (const auto& pixelShaderBytes : pixelShaderByteList)
            {
                CHECK_HRCMD(m_device->CreatePixelShader(pixelShaderBytes.data(), pixelShaderBytes.size(), nullptr,
                    newVideoPixelShaders[shaderIndex++].ReleaseAndGetAddressOf()));
            }
            m_videoPixelShader = newVideoPixelShaders;
        }
        m_fovDecodeParams = newFovDecParmPtr ?
            std::make_shared<ALXR::FoveatedDecodeParams>(*newFovDecParmPtr) : nullptr;
    }

#include "cuda/d3d11cuda_interop.inl"

   private:
    ComPtr<ID3D11Device> m_device, m_d3d11va_device;
    ComPtr<ID3D11DeviceContext> m_deviceContext;
    LUID                        m_d3d11DeviceLUID {};
    XrGraphicsBindingD3D11KHR m_graphicsBinding{.type=XR_TYPE_GRAPHICS_BINDING_D3D11_KHR, .next=nullptr};
    std::list<std::vector<XrSwapchainImageD3D11KHR>> m_swapchainImageBuffers;
    CoreShaders m_coreShaders{};
    ComPtr<ID3D11VertexShader> m_vertexShader;
    ComPtr<ID3D11PixelShader> m_pixelShader;
    ComPtr<ID3D11InputLayout> m_inputLayout;
    ComPtr<ID3D11Buffer> m_modelCBuffer;
    ComPtr<ID3D11Buffer> m_viewProjectionCBuffer;
    ComPtr<ID3D11Buffer> m_fovDecodeCBuffer;
    ComPtr<ID3D11Buffer> m_cubeVertexBuffer;
    ComPtr<ID3D11Buffer> m_cubeIndexBuffer;
//video textures /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    using FoveatedDecodeParamsPtr = std::shared_ptr<ALXR::FoveatedDecodeParams>;
    FoveatedDecodeParamsPtr m_fovDecodeParams{};

    ComPtr<ID3D11DeviceContext> m_uploadContext;
    
    ComPtr<ID3D11SamplerState> m_lumaSampler;
    ComPtr<ID3D11SamplerState> m_chromaSampler;
    ComPtr<ID3D11VertexShader> m_videoVertexShader;
    std::array<ComPtr<ID3D11PixelShader>,VideoPShader::TypeCount> m_videoPixelShader;

    D3D11FenceEvent                 m_texRendereComplete{};
    D3D11FenceEvent                 m_texCopy{};
    constexpr static const std::size_t VideoTexCount = 2;
    using CommandListPtr = ComPtr<ID3D11CommandList>;
    struct NV12Texture {
        using Texture2DList = std::vector< ComPtr<ID3D11Texture2D> >;
        using HandleList = std::vector<HANDLE>;

        Texture2DList  data{};
        Texture2DList  d3d11vaSharedData {};
        ComPtr<ID3D11ShaderResourceView> lumaSRV{};
        ComPtr<ID3D11ShaderResourceView> chromaSRV{};
        ComPtr<ID3D11ShaderResourceView> chromaVSRV{};
        CommandListPtr cmdList{};
        std::uint64_t frameIndex = std::uint64_t(-1);
    };
    std::array<NV12Texture, 2> m_videoTextures{};
    std::atomic<std::size_t>   m_currentVideoTex{ (std::size_t)0 }, m_renderTex{ (std::size_t)-1 };
    std::size_t currentTextureIdx = std::size_t(-1);
    //std::mutex                     m_renderMutex{};
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Map color buffer to associated depth buffer. This map is populated on demand.
    std::map<ID3D11Texture2D*, ComPtr<ID3D11DepthStencilView>> m_colorToDepthMap;

    static_assert(XR_ENVIRONMENT_BLEND_MODE_OPAQUE == 1);
    std::size_t m_clearColorIndex{ (XR_ENVIRONMENT_BLEND_MODE_OPAQUE - 1) };

    bool m_isMultiViewSupported = false;
};
}  // namespace

std::shared_ptr<IGraphicsPlugin> CreateGraphicsPlugin_D3D11(const std::shared_ptr<Options>& options,
                                                            std::shared_ptr<IPlatformPlugin> platformPlugin) {
    return std::make_shared<D3D11GraphicsPlugin>(options, platformPlugin);
}

#endif
