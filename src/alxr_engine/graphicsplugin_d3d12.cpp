// Copyright (c) 2017-2022, The Khronos Group Inc.
//
// SPDX-License-Identifier: Apache-2.0
#include "pch.h"
#include "common.h"
#include "geometry.h"
#include "options.h"
#include "graphicsplugin.h"

#if defined(XR_USE_GRAPHICS_API_D3D12)
#include <type_traits>
#include <array>
#include <map>
#include <unordered_map>
#include <variant>
#include <thread>
#include <chrono>
#include "xr_eigen.h"

#include <DirectXColors.h>
#include <D3Dcompiler.h>
#include <d3d11on12.h>
#include "d3dx12.h"
#include "d3d_common.h"
#include "d3d_fence_event.h"
#include "foveation.h"
#include "concurrent_queue.h"
#include "cuda/WindowsSecurityAttributes.h"
#ifdef XR_ENABLE_CUDA_INTEROP
#include "cuda/d3d12cuda_interop.h"
#endif

using namespace Microsoft::WRL;
using namespace DirectX;

namespace {
void InitializeD3D12DeviceForAdapter(IDXGIAdapter1* adapter, D3D_FEATURE_LEVEL minimumFeatureLevel, ID3D12Device** device) {
#if !defined(NDEBUG)
    ComPtr<ID3D12Debug> debugCtrl;
    if (SUCCEEDED(D3D12GetDebugInterface(__uuidof(ID3D12Debug), &debugCtrl))) {
        debugCtrl->EnableDebugLayer();
    }
#endif

    // ID3D12Device2 is required for view-instancing support.
    for (const auto d3d12DeviceUuid : { __uuidof(ID3D12Device2), __uuidof(ID3D12Device) }) {
        if (SUCCEEDED(D3D12CreateDevice(adapter, minimumFeatureLevel, d3d12DeviceUuid, reinterpret_cast<void**>(device)))) {
            return;
        }
    }
    CHECK_MSG(false, "Failed to create D3D12Device.");
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

template <uint32_t alignment>
constexpr inline uint32_t AlignTo(uint32_t n) {
    static_assert((alignment & (alignment - 1)) == 0, "The alignment must be power-of-two");
    return (n + alignment - 1) & ~(alignment - 1);
}

ComPtr<ID3D12Resource> CreateBuffer(ID3D12Device* d3d12Device, uint32_t size, D3D12_HEAP_TYPE heapType) {
    D3D12_RESOURCE_STATES d3d12ResourceState;
    if (heapType == D3D12_HEAP_TYPE_UPLOAD) {
        d3d12ResourceState = D3D12_RESOURCE_STATE_GENERIC_READ;
        size = AlignTo<D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT>(size);
    } else {
        d3d12ResourceState = D3D12_RESOURCE_STATE_COMMON;
    }

    const D3D12_HEAP_PROPERTIES heapProp {
        .Type = heapType,
        .CPUPageProperty = D3D12_CPU_PAGE_PROPERTY_UNKNOWN,
        .MemoryPoolPreference = D3D12_MEMORY_POOL_UNKNOWN
    };
    const D3D12_RESOURCE_DESC buffDesc{
        .Dimension = D3D12_RESOURCE_DIMENSION_BUFFER,
        .Alignment = 0,
        .Width = size,
        .Height = 1,
        .DepthOrArraySize = 1,
        .MipLevels = 1,
        .Format = DXGI_FORMAT_UNKNOWN,
        .SampleDesc {
            .Count = 1,
            .Quality = 0
        },
        .Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR,
        .Flags = D3D12_RESOURCE_FLAG_NONE
    };
    ComPtr<ID3D12Resource> buffer;
    CHECK_HRCMD(d3d12Device->CreateCommittedResource(&heapProp, D3D12_HEAP_FLAG_NONE, &buffDesc, d3d12ResourceState, nullptr,
                                                     __uuidof(ID3D12Resource),
                                                     reinterpret_cast<void**>(buffer.ReleaseAndGetAddressOf())));
    return buffer;
}

ComPtr<ID3D12Resource> CreateTexture2D
(
    ID3D12Device* d3d12Device, 
    const std::size_t width, const std::size_t height, const DXGI_FORMAT pixfmt,
    const D3D12_RESOURCE_FLAGS flags = D3D12_RESOURCE_FLAG_NONE,
    const D3D12_HEAP_FLAGS heap_flags = D3D12_HEAP_FLAG_NONE,
    const std::size_t arraySize = 1
)
{
    const D3D12_RESOURCE_DESC textureDesc {
        .Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D,
        .Alignment = 0,
        .Width = width,
        .Height = static_cast<UINT>(height),
        .DepthOrArraySize = static_cast<UINT16>(arraySize),
        .MipLevels = 1,
        .Format = pixfmt,
        .SampleDesc {
            .Count = 1,
            .Quality = 0
        },
        .Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN,
        .Flags = flags,
    };
    const auto heapProps = CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT);
    ComPtr<ID3D12Resource> newTexture;
    CHECK_HRCMD(d3d12Device->CreateCommittedResource(&heapProps,
        heap_flags,
        &textureDesc,
        D3D12_RESOURCE_STATE_COMMON,//D3D12_RESOURCE_STATE_COPY_DEST,
        nullptr,
        IID_PPV_ARGS(&newTexture)));
    return newTexture;
}

ComPtr<ID3D12Resource> CreateTextureUploadBuffer
(
    ID3D12Device* d3d12Device, const ComPtr<ID3D12Resource>& texture,
    const std::uint32_t firstSubResource = 0,
    const std::uint32_t numSubResources = 1
)
{
    const std::uint64_t uploadBufferSize = GetRequiredIntermediateSize(texture.Get(), firstSubResource, numSubResources);

    const auto heapProps = CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD);
    const auto buffDesc = CD3DX12_RESOURCE_DESC::Buffer(uploadBufferSize);
    ComPtr<ID3D12Resource> textureUploadHeap;
    CHECK_HRCMD(d3d12Device->CreateCommittedResource(&heapProps,
        D3D12_HEAP_FLAG_NONE, &buffDesc,
        D3D12_RESOURCE_STATE_GENERIC_READ,
        nullptr,
        IID_PPV_ARGS(&textureUploadHeap)));
    return textureUploadHeap;
}

struct SwapchainImageContext {

    using FoveatedDecodeParamsPtr = std::shared_ptr<ALXR::FoveatedDecodeParams>;

    std::vector<XrSwapchainImageBaseHeader*> Create
    (
        ID3D12Device* d3d12Device, const std::uint32_t capacity, const std::uint32_t viewProjbufferSize,
        const FoveatedDecodeParamsPtr fdParamPtr = nullptr // don't pass by ref.
    ) {
        m_d3d12Device = d3d12Device;

        m_swapchainImages.resize(capacity, {
            .type = XR_TYPE_SWAPCHAIN_IMAGE_D3D12_KHR,
            .next = nullptr,
            .texture = nullptr
        });
        std::vector<XrSwapchainImageBaseHeader*> bases(capacity);
        for (uint32_t i = 0; i < capacity; ++i) {
            bases[i] = reinterpret_cast<XrSwapchainImageBaseHeader*>(&m_swapchainImages[i]);
        }

        CHECK_HRCMD(m_d3d12Device->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, __uuidof(ID3D12CommandAllocator),
                                                          reinterpret_cast<void**>(m_commandAllocator.ReleaseAndGetAddressOf())));
        m_commandAllocator->SetName(L"SwapchainImageCtx_CmdAllocator");
        m_viewProjectionCBuffer = CreateBuffer(m_d3d12Device, viewProjbufferSize, D3D12_HEAP_TYPE_UPLOAD);
        m_viewProjectionCBuffer->SetName(L"SwapchainImageCtx_ViewProjectionCBuffer");

        constexpr const std::uint32_t foveationParamsSize =
            AlignTo<D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT>(sizeof(ALXR::FoveatedDecodeParams));
        m_foveationParamCBuffer = CreateBuffer(m_d3d12Device, foveationParamsSize, D3D12_HEAP_TYPE_UPLOAD);
        m_foveationParamCBuffer->SetName(L"SwapchainImageCtx_FoveationParamCBuffer");
        if (fdParamPtr)
            SetFoveationDecodeData(*fdParamPtr);

        return bases;
    }

    uint32_t ImageIndex(const XrSwapchainImageBaseHeader* swapchainImageHeader) const {
        const auto p = reinterpret_cast<const XrSwapchainImageD3D12KHR*>(swapchainImageHeader);
        return (uint32_t)(p - &m_swapchainImages[0]);
    }

    ID3D12Resource* GetDepthStencilTexture(ID3D12Resource* colorTexture) {
        if (!m_depthStencilTexture) {
            // This back-buffer has no corresponding depth-stencil texture, so create one with matching dimensions.
            const D3D12_RESOURCE_DESC colorDesc = colorTexture->GetDesc();
            constexpr const D3D12_HEAP_PROPERTIES heapProp {
                .Type = D3D12_HEAP_TYPE_DEFAULT,
                .CPUPageProperty = D3D12_CPU_PAGE_PROPERTY_UNKNOWN,
                .MemoryPoolPreference = D3D12_MEMORY_POOL_UNKNOWN
            };
            const D3D12_RESOURCE_DESC depthDesc {
                .Dimension = colorDesc.Dimension,
                .Alignment = colorDesc.Alignment,
                .Width = colorDesc.Width,
                .Height = colorDesc.Height,
                .DepthOrArraySize = colorDesc.DepthOrArraySize,
                .MipLevels = 1,
                .Format = DXGI_FORMAT_R32_TYPELESS,
                .SampleDesc{.Count = 1},
                .Layout = colorDesc.Layout,
                .Flags = D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL,
            };
            constexpr const D3D12_CLEAR_VALUE clearValue {
                .Format = DXGI_FORMAT_D32_FLOAT,
                .DepthStencil{.Depth = 1.0f}
            };
            CHECK_HRCMD(m_d3d12Device->CreateCommittedResource(
                &heapProp, D3D12_HEAP_FLAG_NONE, &depthDesc, D3D12_RESOURCE_STATE_DEPTH_WRITE, &clearValue,
                __uuidof(ID3D12Resource), reinterpret_cast<void**>(m_depthStencilTexture.ReleaseAndGetAddressOf())));

            m_depthStencilTexture->SetName(L"SwapchainImageCtx_DepthStencilTexture");
        }

        return m_depthStencilTexture.Get();
    }

    ID3D12CommandAllocator* GetCommandAllocator() const { return m_commandAllocator.Get(); }

    std::uint64_t GetFrameFenceValue() const { return m_fenceValue; }
    void SetFrameFenceValue(std::uint64_t fenceValue) { m_fenceValue = fenceValue; }

    void ResetCommandAllocator() { CHECK_HRCMD(m_commandAllocator->Reset()); }

    void RequestModelCBuffer(const std::uint32_t requiredSize) {
        if (!m_modelCBuffer || (requiredSize > m_modelCBuffer->GetDesc().Width)) {
            m_modelCBuffer = CreateBuffer(m_d3d12Device, requiredSize, D3D12_HEAP_TYPE_UPLOAD);
            m_modelCBuffer->SetName(L"SwapchainImageCtx_ModelCBuffer");
        }
    }

    ID3D12Resource* GetModelCBuffer() const { return m_modelCBuffer.Get(); }
    ID3D12Resource* GetViewProjectionCBuffer() const { return m_viewProjectionCBuffer.Get(); }
    ID3D12Resource* GetFoveationParamCBuffer() const {
        assert(m_foveationParamCBuffer != nullptr);
        return m_foveationParamCBuffer.Get();
    }

    void SetFoveationDecodeData(const ALXR::FoveatedDecodeParams& fdParams) {
        if (m_foveationParamCBuffer == nullptr)
            return;
        constexpr const std::size_t AlignSize = D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT;
        const alignas(AlignSize) ALXR::FoveatedDecodeParams fovParams = fdParams;
        {
            constexpr const D3D12_RANGE NoReadRange{ 0, 0 };
            void* data = nullptr;
            CHECK_HRCMD(m_foveationParamCBuffer->Map(0, &NoReadRange, &data));
            assert(data != nullptr);
            std::memcpy(data, &fovParams, sizeof(fovParams));
            m_foveationParamCBuffer->Unmap(0, nullptr);
        }
    }

   private:
    ID3D12Device* m_d3d12Device{nullptr};

    std::vector<XrSwapchainImageD3D12KHR> m_swapchainImages;
    ComPtr<ID3D12CommandAllocator> m_commandAllocator;
    ComPtr<ID3D12Resource> m_depthStencilTexture;
    ComPtr<ID3D12Resource> m_modelCBuffer;
    ComPtr<ID3D12Resource> m_viewProjectionCBuffer;
    ComPtr<ID3D12Resource> m_foveationParamCBuffer;
    uint64_t m_fenceValue = 0;
};

struct D3D12GraphicsPlugin final : public IGraphicsPlugin {

    using VideoPShader = ALXR::VideoPShader;
    using CoreShaders  = ALXR::CoreShaders<D3D12_SHADER_BYTECODE>;

    template < const std::size_t N >
    using ShaderByteCodeList = CoreShaders::ShaderByteCodeSpanList<N>;

    enum RootParamIndex : UINT {
        ModelTransform=0,
        ViewProjTransform,
        LumaTexture,
        ChromaTexture,
        ChromaUTexture = ChromaTexture,
        ChromaVTexture,
        FoveatedDecodeParams,
        TypeCount
    };

    std::shared_ptr<Options> m_options{};

    D3D12GraphicsPlugin(const std::shared_ptr<Options>& opts, std::shared_ptr<IPlatformPlugin>)
    : m_options{ opts } {
        assert(m_options != nullptr);
    }

    inline ~D3D12GraphicsPlugin() override { CloseHandle(m_fenceEvent); }

    std::vector<std::string> GetInstanceExtensions() const override { return { XR_KHR_D3D12_ENABLE_EXTENSION_NAME }; }

    D3D_SHADER_MODEL GetHighestSupportedShaderModel() const {
        if (m_device == nullptr)
            return D3D_SHADER_MODEL_5_1;
        constexpr const D3D_SHADER_MODEL ShaderModels[] = {
            D3D_SHADER_MODEL_6_7, D3D_SHADER_MODEL_6_6, D3D_SHADER_MODEL_6_5,
            D3D_SHADER_MODEL_6_4, D3D_SHADER_MODEL_6_3, D3D_SHADER_MODEL_6_2,
            D3D_SHADER_MODEL_6_1, D3D_SHADER_MODEL_6_0, D3D_SHADER_MODEL_5_1,
        };
        for (const auto sm : ShaderModels) {
            D3D12_FEATURE_DATA_SHADER_MODEL shaderModelData {
                .HighestShaderModel = sm
            };
            if (SUCCEEDED(m_device->CheckFeatureSupport(D3D12_FEATURE_SHADER_MODEL, &shaderModelData, sizeof(shaderModelData)))) {
                return shaderModelData.HighestShaderModel;
            }
        }
        return D3D_SHADER_MODEL_5_1;
    }

    void CheckMultiViewSupport()
    {
        if (m_device == nullptr)
            return;

        const auto highestShaderModel = GetHighestSupportedShaderModel();
        Log::Write(Log::Level::Verbose, Fmt("Highest supported shader model: 0x%02x", highestShaderModel));

        D3D12_FEATURE_DATA_D3D12_OPTIONS3 options {
            .ViewInstancingTier = D3D12_VIEW_INSTANCING_TIER_NOT_SUPPORTED
        };
        ComPtr<ID3D12Device2> device2{ nullptr };
        if (SUCCEEDED(m_device.As(&device2)) && device2 != nullptr &&
            SUCCEEDED(m_device->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS3, &options, sizeof(options)))) {
            m_isMultiViewSupported = 
                highestShaderModel >= D3D_SHADER_MODEL_6_1 &&
                options.ViewInstancingTier != D3D12_VIEW_INSTANCING_TIER_NOT_SUPPORTED;
            Log::Write(Log::Level::Verbose, Fmt("D3D12 View-instancing tier: %d", options.ViewInstancingTier));
        }

        CoreShaders::Path smDir{ "SM5" };
        if (m_isMultiViewSupported) {
            Log::Write(Log::Level::Verbose, "Setting SM6 core (multi-view) shaders.");
            smDir = "multiview";
        }
        m_coreShaders = { smDir, m_options->InternalDataPath };
    }

    inline std::uint32_t GetViewProjectionBufferSize() const {
        return static_cast<std::uint32_t>(m_isMultiViewSupported ?
            sizeof(ALXR::MultiViewProjectionConstantBuffer) : sizeof(ALXR::ViewProjectionConstantBuffer));
    }

    void InitializeDevice(XrInstance instance, XrSystemId systemId, const XrEnvironmentBlendMode newMode, const bool /*enableVisibilityMask*/) override {
        PFN_xrGetD3D12GraphicsRequirementsKHR pfnGetD3D12GraphicsRequirementsKHR = nullptr;
        CHECK_XRCMD(xrGetInstanceProcAddr(instance, "xrGetD3D12GraphicsRequirementsKHR",
            reinterpret_cast<PFN_xrVoidFunction*>(&pfnGetD3D12GraphicsRequirementsKHR)));

        // Create the D3D12 device for the adapter associated with the system.
        XrGraphicsRequirementsD3D12KHR graphicsRequirements{ .type=XR_TYPE_GRAPHICS_REQUIREMENTS_D3D12_KHR, .next=nullptr };
        CHECK_XRCMD(pfnGetD3D12GraphicsRequirementsKHR(instance, systemId, &graphicsRequirements));
        const ComPtr<IDXGIAdapter1> adapter = ALXR::GetAdapter(graphicsRequirements.adapterLuid);

        // Create a list of feature levels which are both supported by the OpenXR runtime and this application.
        InitializeD3D12DeviceForAdapter(adapter.Get(), graphicsRequirements.minFeatureLevel, m_device.ReleaseAndGetAddressOf());
        m_dx12deviceluid = graphicsRequirements.adapterLuid;

        CheckMultiViewSupport();
        CHECK(m_coreShaders.IsValid());
        
        const D3D12_COMMAND_QUEUE_DESC queueDesc{
            .Type = D3D12_COMMAND_LIST_TYPE_DIRECT,
            .Flags = D3D12_COMMAND_QUEUE_FLAG_NONE
        };
        CHECK_HRCMD(m_device->CreateCommandQueue(&queueDesc, __uuidof(ID3D12CommandQueue),
            reinterpret_cast<void**>(m_cmdQueue.ReleaseAndGetAddressOf())));
        m_cmdQueue->SetName(L"MainRenderCMDQueue");

        InitializeResources();

        m_graphicsBinding.device = m_device.Get();
        m_graphicsBinding.queue = m_cmdQueue.Get();

        SetEnvironmentBlendMode(newMode);
    }

    void InitializeResources() {
        CHECK(m_device != nullptr);
        InitializeVideoTextureResources();
        {
            constexpr const D3D12_DESCRIPTOR_HEAP_DESC heapDesc{
                .Type = D3D12_DESCRIPTOR_HEAP_TYPE_RTV,
                .NumDescriptors = 1,                
                .Flags = D3D12_DESCRIPTOR_HEAP_FLAG_NONE
            };
            CHECK_HRCMD(m_device->CreateDescriptorHeap(&heapDesc, __uuidof(ID3D12DescriptorHeap),
                reinterpret_cast<void**>(m_rtvHeap.ReleaseAndGetAddressOf())));
        }
        {
            constexpr const D3D12_DESCRIPTOR_HEAP_DESC heapDesc{
                .Type = D3D12_DESCRIPTOR_HEAP_TYPE_DSV,
                .NumDescriptors = 1,                
                .Flags = D3D12_DESCRIPTOR_HEAP_FLAG_NONE
            };
            CHECK_HRCMD(m_device->CreateDescriptorHeap(&heapDesc, __uuidof(ID3D12DescriptorHeap),
                reinterpret_cast<void**>(m_dsvHeap.ReleaseAndGetAddressOf())));
        }
        {
            constexpr const D3D12_DESCRIPTOR_HEAP_DESC heapDesc{
                .Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV,
                .NumDescriptors = 6,                
                .Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE
            };
            CHECK_HRCMD(m_device->CreateDescriptorHeap(&heapDesc, __uuidof(ID3D12DescriptorHeap),
                reinterpret_cast<void**>(m_srvHeap.ReleaseAndGetAddressOf())));
        }

        CD3DX12_DESCRIPTOR_RANGE1 texture1Range1, texture2Range1, texture3Range1;
        texture1Range1.Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 0);
        texture2Range1.Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 1);
        texture3Range1.Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 2);

        CD3DX12_ROOT_PARAMETER1  rootParams1[RootParamIndex::TypeCount];
        rootParams1[RootParamIndex::ModelTransform      ].InitAsConstantBufferView(0, 0, D3D12_ROOT_DESCRIPTOR_FLAG_NONE, D3D12_SHADER_VISIBILITY_VERTEX);
        rootParams1[RootParamIndex::ViewProjTransform   ].InitAsConstantBufferView(1, 0, D3D12_ROOT_DESCRIPTOR_FLAG_NONE, D3D12_SHADER_VISIBILITY_VERTEX);
        rootParams1[RootParamIndex::LumaTexture         ].InitAsDescriptorTable(1, &texture1Range1, D3D12_SHADER_VISIBILITY_PIXEL);
        rootParams1[RootParamIndex::ChromaUTexture      ].InitAsDescriptorTable(1, &texture2Range1, D3D12_SHADER_VISIBILITY_PIXEL);
        rootParams1[RootParamIndex::ChromaVTexture      ].InitAsDescriptorTable(1, &texture3Range1, D3D12_SHADER_VISIBILITY_PIXEL);
        rootParams1[RootParamIndex::FoveatedDecodeParams].InitAsConstantBufferView(2, 0, D3D12_ROOT_DESCRIPTOR_FLAG_NONE, D3D12_SHADER_VISIBILITY_PIXEL);

        constexpr const D3D12_STATIC_SAMPLER_DESC sampler {
            .Filter = D3D12_FILTER_MIN_MAG_MIP_LINEAR,
            .AddressU = D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
            .AddressV = D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
            .AddressW = D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
            .MipLODBias = 0,
            .MaxAnisotropy = 0,
            .ComparisonFunc = D3D12_COMPARISON_FUNC_NEVER,
            .BorderColor = D3D12_STATIC_BORDER_COLOR_TRANSPARENT_BLACK,
            .MinLOD = 0.0f,
            .MaxLOD = D3D12_FLOAT32_MAX,
            .ShaderRegister = 0,
            .RegisterSpace = 0,
            .ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL,
        };
        D3D12_STATIC_SAMPLER_DESC samplers[2]{ sampler, sampler };
        samplers[1].ShaderRegister = 1;

        CD3DX12_VERSIONED_ROOT_SIGNATURE_DESC rootSignatureDesc{};
        rootSignatureDesc.Init_1_1
        (
            (UINT)std::size(rootParams1),
            rootParams1,
            (UINT)std::size(samplers), samplers,
            D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT
        );

        D3D12_FEATURE_DATA_ROOT_SIGNATURE featureData {
            .HighestVersion = D3D_ROOT_SIGNATURE_VERSION_1_1
        };
        if (FAILED(m_device->CheckFeatureSupport(D3D12_FEATURE_ROOT_SIGNATURE, &featureData, sizeof(featureData))))
            featureData.HighestVersion = D3D_ROOT_SIGNATURE_VERSION_1_0;

        ComPtr<ID3DBlob> rootSignatureBlob;
        ComPtr<ID3DBlob> error;
        
        CHECK_HRCMD(D3DX12SerializeVersionedRootSignature(&rootSignatureDesc, featureData.HighestVersion,
            rootSignatureBlob.ReleaseAndGetAddressOf(), error.ReleaseAndGetAddressOf()));

        CHECK_HRCMD(m_device->CreateRootSignature(0, rootSignatureBlob->GetBufferPointer(), rootSignatureBlob->GetBufferSize(),
            __uuidof(ID3D12RootSignature),
            reinterpret_cast<void**>(m_rootSignature.ReleaseAndGetAddressOf())));

        SwapchainImageContext initializeContext;
        const auto _ = initializeContext.Create(m_device.Get(), 1, GetViewProjectionBufferSize());
        
        ComPtr<ID3D12GraphicsCommandList> cmdList;
        CHECK_HRCMD(m_device->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, initializeContext.GetCommandAllocator(), nullptr,
            __uuidof(ID3D12GraphicsCommandList),
            reinterpret_cast<void**>(cmdList.ReleaseAndGetAddressOf())));

        ComPtr<ID3D12Resource> cubeVertexBufferUpload;
        m_cubeVertexBuffer = CreateBuffer(m_device.Get(), sizeof(Geometry::c_cubeVertices), D3D12_HEAP_TYPE_DEFAULT);
        {
            cubeVertexBufferUpload = CreateBuffer(m_device.Get(), sizeof(Geometry::c_cubeVertices), D3D12_HEAP_TYPE_UPLOAD);

            void* data = nullptr;
            const D3D12_RANGE readRange{ 0, 0 };
            CHECK_HRCMD(cubeVertexBufferUpload->Map(0, &readRange, &data));
            memcpy(data, Geometry::c_cubeVertices, sizeof(Geometry::c_cubeVertices));
            cubeVertexBufferUpload->Unmap(0, nullptr);

            cmdList->CopyBufferRegion(m_cubeVertexBuffer.Get(), 0, cubeVertexBufferUpload.Get(), 0,
                sizeof(Geometry::c_cubeVertices));
        }

        ComPtr<ID3D12Resource> cubeIndexBufferUpload;
        m_cubeIndexBuffer = CreateBuffer(m_device.Get(), sizeof(Geometry::c_cubeIndices), D3D12_HEAP_TYPE_DEFAULT);
        {
            cubeIndexBufferUpload = CreateBuffer(m_device.Get(), sizeof(Geometry::c_cubeIndices), D3D12_HEAP_TYPE_UPLOAD);

            void* data = nullptr;
            const D3D12_RANGE readRange{ 0, 0 };
            CHECK_HRCMD(cubeIndexBufferUpload->Map(0, &readRange, &data));
            memcpy(data, Geometry::c_cubeIndices, sizeof(Geometry::c_cubeIndices));
            cubeIndexBufferUpload->Unmap(0, nullptr);

            cmdList->CopyBufferRegion(m_cubeIndexBuffer.Get(), 0, cubeIndexBufferUpload.Get(), 0, sizeof(Geometry::c_cubeIndices));
        }

        CHECK_HRCMD(cmdList->Close());
        ID3D12CommandList* cmdLists[] = { cmdList.Get() };
        m_cmdQueue->ExecuteCommandLists((UINT)std::size(cmdLists), cmdLists);

        CHECK_HRCMD(m_device->CreateFence(m_fenceValue, D3D12_FENCE_FLAG_NONE, __uuidof(ID3D12Fence),
            reinterpret_cast<void**>(m_fence.ReleaseAndGetAddressOf())));
        m_fenceEvent = CreateEvent(nullptr, FALSE, FALSE, nullptr);
        CHECK(m_fenceEvent != nullptr);

        WaitForGpu();
    }

    void InitializeVideoTextureResources() {

        m_texRendereComplete.CreateFence(m_device, D3D12_FENCE_FLAG_SHARED);

        constexpr const D3D12_COMMAND_QUEUE_DESC queueDesc {
            .Type = D3D12_COMMAND_LIST_TYPE_DIRECT,//D3D12_COMMAND_LIST_TYPE_COPY,
            .Flags = D3D12_COMMAND_QUEUE_FLAG_NONE            
        };
        CHECK_HRCMD(m_device->CreateCommandQueue(&queueDesc, __uuidof(ID3D12CommandQueue),
            reinterpret_cast<void**>(m_videoTexCmdCpyQueue.ReleaseAndGetAddressOf())));
        m_videoTexCmdCpyQueue->SetName(L"VideoTextureCpyQueue");

        m_texCopy.CreateFence(m_device, D3D12_FENCE_FLAG_SHARED);

        assert(m_videoTexCmdAllocator == nullptr);
        CHECK_HRCMD(m_device->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT/*COPY*/, __uuidof(ID3D12CommandAllocator),
            reinterpret_cast<void**>(m_videoTexCmdAllocator.ReleaseAndGetAddressOf())));

        InitD3D11OnD3D12();
#ifdef XR_ENABLE_CUDA_INTEROP
        InitCuda();
#endif
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

        m_swapchainImageContexts.emplace_back();
        SwapchainImageContext& swapchainImageContext = m_swapchainImageContexts.back();

        std::vector<XrSwapchainImageBaseHeader*> bases = swapchainImageContext.Create
        (
            m_device.Get(), capacity, GetViewProjectionBufferSize(), m_fovDecodeParams
        );

        // Map every swapchainImage base pointer to this context
        for (auto& base : bases) {
            m_swapchainImageContextMap[base] = &swapchainImageContext;
        }

        return bases;
    }

    virtual void ClearSwapchainImageStructs() override
    {
        m_swapchainImageContextMap.clear();
        for (auto& swapchainContext : m_swapchainImageContexts) {
            CpuWaitForFence(swapchainContext.GetFrameFenceValue());
        }
        m_swapchainImageContexts.clear();
    }

    struct PipelineStateStream
    {
        CD3DX12_PIPELINE_STATE_STREAM_ROOT_SIGNATURE pRootSignature;
        CD3DX12_PIPELINE_STATE_STREAM_VS VS;
        CD3DX12_PIPELINE_STATE_STREAM_PS PS;
        CD3DX12_PIPELINE_STATE_STREAM_BLEND_DESC BlendState;
        CD3DX12_PIPELINE_STATE_STREAM_SAMPLE_MASK SampleMask;
        CD3DX12_PIPELINE_STATE_STREAM_RASTERIZER RasterizerState;
        CD3DX12_PIPELINE_STATE_STREAM_DEPTH_STENCIL DepthStencilState;
        CD3DX12_PIPELINE_STATE_STREAM_INPUT_LAYOUT InputLayout;
        CD3DX12_PIPELINE_STATE_STREAM_IB_STRIP_CUT_VALUE IBStripCutValue;
        CD3DX12_PIPELINE_STATE_STREAM_PRIMITIVE_TOPOLOGY PrimitiveTopologyType;
        CD3DX12_PIPELINE_STATE_STREAM_RENDER_TARGET_FORMATS RTVFormats;
        CD3DX12_PIPELINE_STATE_STREAM_DEPTH_STENCIL_FORMAT DSVFormat;
        CD3DX12_PIPELINE_STATE_STREAM_SAMPLE_DESC SampleDesc;
        CD3DX12_PIPELINE_STATE_STREAM_NODE_MASK NodeMask;
        CD3DX12_PIPELINE_STATE_STREAM_CACHED_PSO CachedPSO;
        CD3DX12_PIPELINE_STATE_STREAM_FLAGS Flags;
        CD3DX12_PIPELINE_STATE_STREAM_VIEW_INSTANCING ViewInstancing;
    };

    template < typename PipelineStateStreamT, const std::size_t N >
    static void MakeDefaultPipelineStateDesc
    (
        PipelineStateStreamT& pipelineStateStream,
        const DXGI_FORMAT swapchainFormat,
        const std::array<D3D12_SHADER_BYTECODE, 2>& shaders,
        const std::array<const D3D12_INPUT_ELEMENT_DESC, N>& inputElementDescs
    )
    {
        pipelineStateStream.VS = shaders[0];
        pipelineStateStream.PS = shaders[1];
        {
            D3D12_BLEND_DESC blendState{
                .AlphaToCoverageEnable = false,
                .IndependentBlendEnable = false
            };
            for (size_t i = 0; i < std::size(blendState.RenderTarget); ++i) {
                blendState.RenderTarget[i] = {
                    .BlendEnable = false,
                    .SrcBlend = D3D12_BLEND_ONE,
                    .DestBlend = D3D12_BLEND_ZERO,
                    .BlendOp = D3D12_BLEND_OP_ADD,
                    .SrcBlendAlpha = D3D12_BLEND_ONE,
                    .DestBlendAlpha = D3D12_BLEND_ZERO,
                    .BlendOpAlpha = D3D12_BLEND_OP_ADD,
                    .LogicOp = D3D12_LOGIC_OP_NOOP,
                    .RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL
                };
            }
            const CD3DX12_BLEND_DESC desc(blendState);
            pipelineStateStream.BlendState = desc;
        }
        pipelineStateStream.SampleMask = 0xFFFFFFFF;
        {
            constexpr static const D3D12_RASTERIZER_DESC rasterizerState {
                .FillMode = D3D12_FILL_MODE_SOLID,
                .CullMode = D3D12_CULL_MODE_BACK,
                .FrontCounterClockwise = FALSE,
                .DepthBias = D3D12_DEFAULT_DEPTH_BIAS,
                .DepthBiasClamp = D3D12_DEFAULT_DEPTH_BIAS_CLAMP,
                .SlopeScaledDepthBias = D3D12_DEFAULT_SLOPE_SCALED_DEPTH_BIAS,
                .DepthClipEnable = TRUE,
                .MultisampleEnable = FALSE,
                .AntialiasedLineEnable = FALSE,
                .ForcedSampleCount = 0,
                .ConservativeRaster = D3D12_CONSERVATIVE_RASTERIZATION_MODE_OFF,
            };
            const CD3DX12_RASTERIZER_DESC desc(rasterizerState);
            pipelineStateStream.RasterizerState = desc;
        }
        {
            constexpr static const D3D12_DEPTH_STENCILOP_DESC stencil_op{
                D3D12_STENCIL_OP_KEEP, D3D12_STENCIL_OP_KEEP, D3D12_STENCIL_OP_KEEP, D3D12_COMPARISON_FUNC_ALWAYS
            };
            constexpr static const D3D12_DEPTH_STENCIL_DESC depthStencilState {
                .DepthEnable = TRUE,
                .DepthWriteMask = D3D12_DEPTH_WRITE_MASK_ALL,
                .DepthFunc = D3D12_COMPARISON_FUNC_LESS,
                .StencilEnable = FALSE,
                .StencilReadMask = D3D12_DEFAULT_STENCIL_READ_MASK,
                .StencilWriteMask = D3D12_DEFAULT_STENCIL_WRITE_MASK,
                .FrontFace = stencil_op,
                .BackFace = stencil_op
            };
            const CD3DX12_DEPTH_STENCIL_DESC desc(depthStencilState);
            pipelineStateStream.DepthStencilState = desc;
        }
        pipelineStateStream.InputLayout = { 
            inputElementDescs.size() == 0 ? nullptr : inputElementDescs.data(),
            (UINT)inputElementDescs.size()
        };
        pipelineStateStream.IBStripCutValue = D3D12_INDEX_BUFFER_STRIP_CUT_VALUE_0xFFFF;
        pipelineStateStream.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
        pipelineStateStream.DSVFormat = DXGI_FORMAT_D32_FLOAT;
        pipelineStateStream.SampleDesc = { 1, 0 };
        pipelineStateStream.NodeMask = 0;
        pipelineStateStream.CachedPSO = { nullptr, 0 };
        pipelineStateStream.Flags = D3D12_PIPELINE_STATE_FLAG_NONE;

        if constexpr (std::is_same<PipelineStateStreamT, PipelineStateStream>::value) {
            constexpr static const std::array<const D3D12_VIEW_INSTANCE_LOCATION, 2> ViewInstanceLocations {
                D3D12_VIEW_INSTANCE_LOCATION {
                    .ViewportArrayIndex = 0u,
                    .RenderTargetArrayIndex = 0u
                },
                D3D12_VIEW_INSTANCE_LOCATION {
                    .ViewportArrayIndex = 0u,
                    .RenderTargetArrayIndex = 1u
                },
            };
            pipelineStateStream.RTVFormats = D3D12_RT_FORMAT_ARRAY{
                .RTFormats { swapchainFormat },
                .NumRenderTargets = 1u,
            };
            pipelineStateStream.ViewInstancing = CD3DX12_VIEW_INSTANCING_DESC
            (
                (UINT)ViewInstanceLocations.size(),
                ViewInstanceLocations.data(),
                D3D12_VIEW_INSTANCING_FLAG_NONE
            );
        } else {
            pipelineStateStream.NumRenderTargets = 1u;
            pipelineStateStream.RTVFormats[0] = swapchainFormat;
        }
    }

    template < const std::size_t N >
    inline ComPtr<ID3D12PipelineState> MakePipelineState
    (
        const DXGI_FORMAT swapchainFormat,
        const std::array<D3D12_SHADER_BYTECODE, 2>& shaders,
        const std::array<const D3D12_INPUT_ELEMENT_DESC, N>& inputElementDescs
    ) const {
        assert(m_device != nullptr);
        ComPtr<ID3D12PipelineState> pipelineState{ nullptr };
        if (m_isMultiViewSupported) {
            ComPtr<ID3D12Device2> device2{ nullptr };
            CHECK_HRCMD(m_device.As(&device2));
            CHECK(device2 != nullptr);

            PipelineStateStream pipelineStateDesc{ .pRootSignature = m_rootSignature.Get() };
            MakeDefaultPipelineStateDesc(pipelineStateDesc, swapchainFormat, shaders, inputElementDescs);
            const D3D12_PIPELINE_STATE_STREAM_DESC pipelineStateStreamDesc{
                .SizeInBytes = sizeof(PipelineStateStream),
                .pPipelineStateSubobjectStream = &pipelineStateDesc
            };
            CHECK_HRCMD(device2->CreatePipelineState(&pipelineStateStreamDesc, __uuidof(ID3D12PipelineState),
                reinterpret_cast<void**>(pipelineState.ReleaseAndGetAddressOf())));
        }
        else {
            D3D12_GRAPHICS_PIPELINE_STATE_DESC pipelineStateDesc{ .pRootSignature = m_rootSignature.Get() };
            MakeDefaultPipelineStateDesc(pipelineStateDesc, swapchainFormat, shaders, inputElementDescs);
            CHECK_HRCMD(m_device->CreateGraphicsPipelineState(&pipelineStateDesc, __uuidof(ID3D12PipelineState),
                reinterpret_cast<void**>(pipelineState.ReleaseAndGetAddressOf())));
        }
        assert(pipelineState != nullptr);
        return pipelineState;
    }

    ID3D12PipelineState* GetOrCreateDefaultPipelineState(const DXGI_FORMAT swapchainFormat) {
        const auto iter = m_pipelineStates.find(swapchainFormat);
        if (iter != m_pipelineStates.end()) {
            return iter->second.Get();
        }

        constexpr static const std::array<const D3D12_INPUT_ELEMENT_DESC, 2> inputElementDescs = {
            D3D12_INPUT_ELEMENT_DESC {
                "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, D3D12_APPEND_ALIGNED_ELEMENT,
                 D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0
            },
            D3D12_INPUT_ELEMENT_DESC {
                "COLOR", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, D3D12_APPEND_ALIGNED_ELEMENT,
                D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0
            },
        };
        const auto shaders = m_coreShaders.GetLobbyByteCodes();
        ComPtr<ID3D12PipelineState> pipelineState = MakePipelineState(swapchainFormat, shaders, inputElementDescs);

        ID3D12PipelineState* const pipelineStateRaw = pipelineState.Get();
        m_pipelineStates.emplace(swapchainFormat, std::move(pipelineState));
        return pipelineStateRaw;
    }

    constexpr static inline std::size_t VideoPipelineIndex(const bool is3PlaneFmt, const PassthroughMode newMode) {
        return static_cast<const std::size_t>(newMode) + (is3PlaneFmt ? VideoPShader::Normal3Plane : VideoPShader::Normal);
    }

    ID3D12PipelineState* GetOrCreateVideoPipelineState(const DXGI_FORMAT swapchainFormat, const PassthroughMode newMode) {
        const bool is3PlaneFormat = m_is3PlaneFormat.load();
        const auto iter = m_VideoPipelineStates.find(swapchainFormat);
        if (iter != m_VideoPipelineStates.end()) {
            return iter->second[VideoPipelineIndex(is3PlaneFormat, newMode)].Get();
        }

        constexpr static const std::array<const D3D12_INPUT_ELEMENT_DESC, 0> EmptyInputElementDescs {};
        const auto makePipeline = [&, this](const ShaderByteCodeList<2>& shaders) -> ComPtr<ID3D12PipelineState>
        {
            return MakePipelineState(swapchainFormat, shaders, EmptyInputElementDescs);
        };
        
        const auto videoShaderBCodes = m_coreShaders.GetVideoByteCodes(m_fovDecodeParams != nullptr);
        const auto newPipelineState = m_VideoPipelineStates.emplace(swapchainFormat, VidePipelineStateList{
            makePipeline({ videoShaderBCodes[0], videoShaderBCodes[1+VideoPShader::Normal] }),
            makePipeline({ videoShaderBCodes[0], videoShaderBCodes[1+VideoPShader::PassthroughBlend] }),
            makePipeline({ videoShaderBCodes[0], videoShaderBCodes[1+VideoPShader::PassthroughMask] }),
            makePipeline({ videoShaderBCodes[0], videoShaderBCodes[1+VideoPShader::Normal3Plane] }),
            makePipeline({ videoShaderBCodes[0], videoShaderBCodes[1+VideoPShader::PassthroughBlend3Plane] }),
            makePipeline({ videoShaderBCodes[0], videoShaderBCodes[1+VideoPShader::PassthroughMask3Plane] }),
        });
        CHECK(newPipelineState.second);
        return newPipelineState.first->second[VideoPipelineIndex(is3PlaneFormat, newMode)].Get();
    }

    enum class RenderPipelineType {
        Default,
        Video
    };
    inline ID3D12PipelineState* GetOrCreatePipelineState
    (
        const DXGI_FORMAT swapchainFormat,
        const RenderPipelineType pt = RenderPipelineType::Default,
        const PassthroughMode newMode = PassthroughMode::None
    ) {
        switch (pt) {
        case RenderPipelineType::Video:
            return GetOrCreateVideoPipelineState(swapchainFormat, newMode);
        case RenderPipelineType::Default:
        default: return GetOrCreateDefaultPipelineState(swapchainFormat);
        }
    }

    template < typename RenderFun >
    inline void RenderViewImpl
    (
        const XrCompositionLayerProjectionView& layerView, const XrSwapchainImageBaseHeader* swapchainImage, const int64_t swapchainFormat,
        RenderFun&& renderFn,
        const RenderPipelineType pt = RenderPipelineType::Default,
        const PassthroughMode newMode = PassthroughMode::None
    )
    {
        auto& swapchainContext = *m_swapchainImageContextMap[swapchainImage];
        CpuWaitForFence(swapchainContext.GetFrameFenceValue());
        swapchainContext.ResetCommandAllocator();

        ComPtr<ID3D12GraphicsCommandList> cmdList;
        CHECK_HRCMD(m_device->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, swapchainContext.GetCommandAllocator(), nullptr,
            __uuidof(ID3D12GraphicsCommandList),
            reinterpret_cast<void**>(cmdList.ReleaseAndGetAddressOf())));

        ID3D12PipelineState* const pipelineState = GetOrCreatePipelineState((DXGI_FORMAT)swapchainFormat, pt, newMode);
        cmdList->SetPipelineState(pipelineState);
        cmdList->SetGraphicsRootSignature(m_rootSignature.Get());

        ID3D12Resource* const colorTexture = reinterpret_cast<const XrSwapchainImageD3D12KHR*>(swapchainImage)->texture;
        const D3D12_RESOURCE_DESC colorTextureDesc = colorTexture->GetDesc();

        const D3D12_VIEWPORT viewport = { (float)layerView.subImage.imageRect.offset.x,
                                         (float)layerView.subImage.imageRect.offset.y,
                                         (float)layerView.subImage.imageRect.extent.width,
                                         (float)layerView.subImage.imageRect.extent.height,
                                         0,
                                         1 };
        cmdList->RSSetViewports(1, &viewport);

        const D3D12_RECT scissorRect = { layerView.subImage.imageRect.offset.x, layerView.subImage.imageRect.offset.y,
                                        layerView.subImage.imageRect.offset.x + layerView.subImage.imageRect.extent.width,
                                        layerView.subImage.imageRect.offset.y + layerView.subImage.imageRect.extent.height };
        cmdList->RSSetScissorRects(1, &scissorRect);

        // Create RenderTargetView with original swapchain format (swapchain is typeless).
        D3D12_CPU_DESCRIPTOR_HANDLE renderTargetView = m_rtvHeap->GetCPUDescriptorHandleForHeapStart();
        D3D12_RENDER_TARGET_VIEW_DESC renderTargetViewDesc{};
        renderTargetViewDesc.Format = (DXGI_FORMAT)swapchainFormat;
        if (colorTextureDesc.DepthOrArraySize > 1) {
            if (colorTextureDesc.SampleDesc.Count > 1) {
                renderTargetViewDesc.ViewDimension = D3D12_RTV_DIMENSION_TEXTURE2DMSARRAY;
                renderTargetViewDesc.Texture2DMSArray.ArraySize = colorTextureDesc.DepthOrArraySize;
            }
            else {
                renderTargetViewDesc.ViewDimension = D3D12_RTV_DIMENSION_TEXTURE2DARRAY;
                renderTargetViewDesc.Texture2DArray.ArraySize = colorTextureDesc.DepthOrArraySize;
            }
        }
        else {
            if (colorTextureDesc.SampleDesc.Count > 1) {
                renderTargetViewDesc.ViewDimension = D3D12_RTV_DIMENSION_TEXTURE2DMS;
            }
            else {
                renderTargetViewDesc.ViewDimension = D3D12_RTV_DIMENSION_TEXTURE2D;
            }
        }
        m_device->CreateRenderTargetView(colorTexture, &renderTargetViewDesc, renderTargetView);

        ID3D12Resource* depthStencilTexture = swapchainContext.GetDepthStencilTexture(colorTexture);
        const D3D12_RESOURCE_DESC depthStencilTextureDesc = depthStencilTexture->GetDesc();
        D3D12_CPU_DESCRIPTOR_HANDLE depthStencilView = m_dsvHeap->GetCPUDescriptorHandleForHeapStart();
        D3D12_DEPTH_STENCIL_VIEW_DESC depthStencilViewDesc{};
        depthStencilViewDesc.Format = DXGI_FORMAT_D32_FLOAT;
        if (depthStencilTextureDesc.DepthOrArraySize > 1) {
            if (depthStencilTextureDesc.SampleDesc.Count > 1) {
                depthStencilViewDesc.ViewDimension = D3D12_DSV_DIMENSION_TEXTURE2DMSARRAY;
                depthStencilViewDesc.Texture2DMSArray.ArraySize = colorTextureDesc.DepthOrArraySize;
            }
            else {
                depthStencilViewDesc.ViewDimension = D3D12_DSV_DIMENSION_TEXTURE2DARRAY;
                depthStencilViewDesc.Texture2DArray.ArraySize = colorTextureDesc.DepthOrArraySize;
            }
        }
        else {
            if (depthStencilTextureDesc.SampleDesc.Count > 1) {
                depthStencilViewDesc.ViewDimension = D3D12_DSV_DIMENSION_TEXTURE2DMS;
            }
            else {
                depthStencilViewDesc.ViewDimension = D3D12_DSV_DIMENSION_TEXTURE2D;
            }
        }
        m_device->CreateDepthStencilView(depthStencilTexture, &depthStencilViewDesc, depthStencilView);

        renderFn(cmdList, renderTargetView, depthStencilView, swapchainContext);

        CHECK_HRCMD(cmdList->Close());
        ID3D12CommandList* const cmdLists[] = { cmdList.Get() };
        m_cmdQueue->ExecuteCommandLists((UINT)std::size(cmdLists), cmdLists);

        SignalFence();
        swapchainContext.SetFrameFenceValue(m_fenceValue);
    }

    inline std::size_t ClearColorIndex(const PassthroughMode /*ptMode*/) const {
        static_assert(ALXR::ClearColors.size() >= 3);
        static_assert(ALXR::VideoClearColors.size() >= 3);
        return m_clearColorIndex;
    }

    inline DirectX::XMFLOAT4X4A XM_CALLCONV MakeViewProjMatrix(const XrCompositionLayerProjectionView& layerView) {
        const XMMATRIX spaceToView = XMMatrixInverse(nullptr, ALXR::LoadXrPose(layerView.pose));
        Eigen::Matrix4f projectionMatrix = ALXR::CreateProjectionFov(ALXR::GraphicsAPI::D3D, layerView.fov, 0.05f, 100.0f);
        DirectX::XMFLOAT4X4A viewProj;
        XMStoreFloat4x4(&viewProj, XMMatrixTranspose(spaceToView * ALXR::LoadXrMatrix(projectionMatrix)));
        return viewProj;
    }

    virtual void RenderMultiView
    (
        const std::array<XrCompositionLayerProjectionView,2>& layerViews, const XrSwapchainImageBaseHeader* swapchainImage,
        const std::int64_t swapchainFormat, const PassthroughMode ptMode,
        const std::vector<Cube>& cubes
    ) override
    {
        assert(m_isMultiViewSupported);
        using CpuDescHandle = D3D12_CPU_DESCRIPTOR_HANDLE;
        using CommandListPtr = ComPtr<ID3D12GraphicsCommandList>;
        RenderViewImpl(layerViews[0], swapchainImage, swapchainFormat, [&]
        (
            const CommandListPtr& cmdList,
            const CpuDescHandle& renderTargetView,
            const CpuDescHandle& depthStencilView,
            SwapchainImageContext& swapchainContext
        )
        {
            cmdList->ClearRenderTargetView(renderTargetView, ALXR::ClearColors[ClearColorIndex(ptMode)], 0, nullptr);
            cmdList->ClearDepthStencilView(depthStencilView, D3D12_CLEAR_FLAG_DEPTH, 1.0f, 0, 0, nullptr);

            const D3D12_CPU_DESCRIPTOR_HANDLE renderTargets[] = { renderTargetView };
            cmdList->OMSetRenderTargets((UINT)std::size(renderTargets), renderTargets, true, &depthStencilView);

            ALXR::MultiViewProjectionConstantBuffer viewProjection{ {} };
            for (std::size_t viewIndex = 0; viewIndex < 2; ++viewIndex) {
                viewProjection.ViewProjection[viewIndex] = MakeViewProjMatrix(layerViews[viewIndex]);
            }

            // Set shaders and constant buffers.
            ID3D12Resource* const viewProjectionCBuffer = swapchainContext.GetViewProjectionCBuffer();
            {
                constexpr const D3D12_RANGE NoReadRange{ 0, 0 };
                void* data = nullptr;
                CHECK_HRCMD(viewProjectionCBuffer->Map(0, &NoReadRange, &data));
                assert(data != nullptr);
                std::memcpy(data, &viewProjection, sizeof(viewProjection));
                viewProjectionCBuffer->Unmap(0, nullptr);
            }
            cmdList->SetGraphicsRootConstantBufferView(RootParamIndex::ViewProjTransform, viewProjectionCBuffer->GetGPUVirtualAddress());

            RenderVisCubes(cubes, swapchainContext, cmdList);

        }, RenderPipelineType::Default);
    }

    virtual void RenderView
    (
        const std::array<XrCompositionLayerProjectionView, 2>& layerViews,
        const std::array<XrSwapchainImageBaseHeader*, 2>& swapchainImages,
        const std::int64_t swapchainFormat, const PassthroughMode ptMode,
        const std::vector<Cube>& cubes
    ) override
    {
        for (std::size_t viewID = 0; viewID < layerViews.size(); ++viewID) {
            const auto& layerView = layerViews[viewID];
            assert(layerView.subImage.imageArrayIndex == 0);
            
            using CpuDescHandle = D3D12_CPU_DESCRIPTOR_HANDLE;
            using CommandListPtr = ComPtr<ID3D12GraphicsCommandList>;
            RenderViewImpl(layerView, swapchainImages[viewID], swapchainFormat, [&]
            (
                const CommandListPtr& cmdList,
                const CpuDescHandle& renderTargetView,
                const CpuDescHandle& depthStencilView,
                SwapchainImageContext& swapchainContext
            )
            {
                // Clear swapchain and depth buffer. NOTE: This will clear the entire render target view, not just the specified view.
                cmdList->ClearRenderTargetView(renderTargetView, ALXR::ClearColors[ClearColorIndex(ptMode)], 0, nullptr);
                cmdList->ClearDepthStencilView(depthStencilView, D3D12_CLEAR_FLAG_DEPTH, 1.0f, 0, 0, nullptr);

                const D3D12_CPU_DESCRIPTOR_HANDLE renderTargets[] = { renderTargetView };
                cmdList->OMSetRenderTargets((UINT)std::size(renderTargets), renderTargets, true, &depthStencilView);

                // Set shaders and constant buffers.
                const ALXR::ViewProjectionConstantBuffer viewProjection{
                    .ViewProjection = MakeViewProjMatrix(layerView),
                    .ViewID = 0,
                };
                ID3D12Resource* const viewProjectionCBuffer = swapchainContext.GetViewProjectionCBuffer();
                {
                    constexpr const D3D12_RANGE NoReadRange{ 0, 0 };
                    void* data = nullptr;
                    CHECK_HRCMD(viewProjectionCBuffer->Map(0, &NoReadRange, &data));
                    assert(data != nullptr);
                    std::memcpy(data, &viewProjection, sizeof(viewProjection));
                    viewProjectionCBuffer->Unmap(0, nullptr);
                }
                cmdList->SetGraphicsRootConstantBufferView(RootParamIndex::ViewProjTransform, viewProjectionCBuffer->GetGPUVirtualAddress());

                RenderVisCubes(cubes, swapchainContext, cmdList);

            }, RenderPipelineType::Default);
        }
    }

    void RenderVisCubes(const std::vector<Cube>& cubes, SwapchainImageContext& swapchainContext, const ComPtr<ID3D12GraphicsCommandList>& cmdList)
    {
        // Set cube primitive data.
        if (cubes.empty())
            return;
        assert(cmdList != nullptr);

        const D3D12_VERTEX_BUFFER_VIEW vertexBufferView[] = {
            {m_cubeVertexBuffer->GetGPUVirtualAddress(), sizeof(Geometry::c_cubeVertices), sizeof(Geometry::Vertex)} };
        cmdList->IASetVertexBuffers(0, (UINT)std::size(vertexBufferView), vertexBufferView);

        const D3D12_INDEX_BUFFER_VIEW indexBufferView{ m_cubeIndexBuffer->GetGPUVirtualAddress(), sizeof(Geometry::c_cubeIndices),
                                                DXGI_FORMAT_R16_UINT };
        cmdList->IASetIndexBuffer(&indexBufferView);
        cmdList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

        constexpr const std::uint32_t cubeCBufferSize = AlignTo<D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT>(sizeof(ALXR::ModelConstantBuffer));
        swapchainContext.RequestModelCBuffer(static_cast<uint32_t>(cubeCBufferSize * cubes.size()));
        ID3D12Resource* const modelCBuffer = swapchainContext.GetModelCBuffer();

        // Render each cube
        std::uint32_t offset = 0;
        for (const Cube& cube : cubes) {
            // Compute and update the model transform.
            ALXR::ModelConstantBuffer model;
            XMStoreFloat4x4(&model.Model,
                XMMatrixTranspose(XMMatrixScaling(cube.Scale.x, cube.Scale.y, cube.Scale.z) * ALXR::LoadXrPose(cube.Pose)));
            {
                constexpr const D3D12_RANGE NoReadRange{ 0, 0 };
                std::uint8_t* data = nullptr;                
                CHECK_HRCMD(modelCBuffer->Map(0, &NoReadRange, reinterpret_cast<void**>(&data)));
                assert(data != nullptr);
                std::memcpy(data + offset, &model, sizeof(model));
                const D3D12_RANGE writeRange{ offset, offset + cubeCBufferSize };
                modelCBuffer->Unmap(0, &writeRange);
            }
            cmdList->SetGraphicsRootConstantBufferView(RootParamIndex::ModelTransform, modelCBuffer->GetGPUVirtualAddress() + offset);
            // Draw the cube.
            cmdList->DrawIndexedInstanced((UINT)std::size(Geometry::c_cubeIndices), 1, 0, 0, 0);
            
            offset += cubeCBufferSize;
        }
    }

    void SignalFence() {
        ++m_fenceValue;
        CHECK_HRCMD(m_cmdQueue->Signal(m_fence.Get(), m_fenceValue));
    }

    void CpuWaitForFence(uint64_t fenceValue) {
        if (m_fence->GetCompletedValue() < fenceValue) {
            CHECK_HRCMD(m_fence->SetEventOnCompletion(fenceValue, m_fenceEvent));
            const uint32_t retVal = WaitForSingleObjectEx(m_fenceEvent, INFINITE, FALSE);
            if (retVal != WAIT_OBJECT_0) {
                CHECK_HRCMD(E_FAIL);
            }
        }
    }

    void WaitForGpu() {
        SignalFence();
        CpuWaitForFence(m_fenceValue);
    }

    void CreateVideoTextures
    (
        const std::size_t width, const std::size_t height, const XrPixelFormat pixfmt,
        const bool createUploadBuffer,
        const D3D12_HEAP_FLAGS heap_flags = D3D12_HEAP_FLAG_NONE,
        const D3D12_RESOURCE_FLAGS res_flags = D3D12_RESOURCE_FLAG_NONE
    )
    {
        if (m_device == nullptr)
            return;

        ClearVideoTextures();

        CHECK(width % 2 == 0);

        const bool is3PlaneFmt = PlaneCount(pixfmt) > 2;

        /*constexpr*/ const DXGI_FORMAT LUMA_FORMAT     = GetLumaFormat(pixfmt);
        /*constexpr*/ const DXGI_FORMAT CHROMA_FORMAT   = GetChromaFormat(pixfmt);
        /*constexpr*/ const DXGI_FORMAT CHROMAU_FORMAT  = GetChromaUFormat(pixfmt);
        /*constexpr*/ const DXGI_FORMAT CHROMAV_FORMAT  = GetChromaVFormat(pixfmt);

        const std::uint32_t descriptorSize = m_device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);
        CD3DX12_CPU_DESCRIPTOR_HANDLE cpuHandle(m_srvHeap->GetCPUDescriptorHandleForHeapStart());
        CD3DX12_GPU_DESCRIPTOR_HANDLE gpuHandle(m_srvHeap->GetGPUDescriptorHandleForHeapStart());
        for (auto& videoTex : m_videoTextures)
        {
            if (!is3PlaneFmt)
            {
                videoTex.texture = ::CreateTexture2D(m_device.Get(), width, height, MapFormat(pixfmt), res_flags, heap_flags);
                CHECK(videoTex.texture != nullptr);
                if (createUploadBuffer) {
                    videoTex.uploadTexture = CreateTextureUploadBuffer(m_device.Get(), videoTex.texture, 0, 2);
                    CHECK(videoTex.uploadTexture != nullptr);
                }

                D3D12_SHADER_RESOURCE_VIEW_DESC srvDesc {
                    .Format = LUMA_FORMAT,
                    .ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D,
                    .Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING,
                    .Texture2D {
                        .MipLevels = 1,
                        .PlaneSlice = 0
                    }
                };
                m_device->CreateShaderResourceView(videoTex.texture.Get(), &srvDesc, cpuHandle);
                videoTex.lumaHandle = cpuHandle;
                videoTex.lumaGpuHandle = gpuHandle;
                cpuHandle.Offset(1, descriptorSize);
                gpuHandle.Offset(1, descriptorSize);

                srvDesc.Texture2D.PlaneSlice = 1;
                srvDesc.Format = CHROMA_FORMAT;
                m_device->CreateShaderResourceView(videoTex.texture.Get(), &srvDesc, cpuHandle);
                videoTex.chromaHandle = cpuHandle;
                videoTex.chromaGpuHandle = gpuHandle;
                cpuHandle.Offset(1, descriptorSize);
                gpuHandle.Offset(1, descriptorSize);

            }
            else
            {
                const std::size_t chromaWidth  = width / 2;
                const std::size_t chromaHeight = height / 2;
                videoTex.lumaTexture    = ::CreateTexture2D(m_device.Get(), width, height, LUMA_FORMAT);
                assert(CHROMAU_FORMAT == CHROMAV_FORMAT);
                videoTex.chromaTexture  = ::CreateTexture2D
                (
                    m_device.Get(), chromaWidth, chromaHeight, CHROMAU_FORMAT//,
                    //D3D12_RESOURCE_FLAG_NONE, D3D12_HEAP_FLAG_NONE, 2
                );
                videoTex.chromaVTexture = ::CreateTexture2D(m_device.Get(), chromaWidth, chromaHeight, CHROMAV_FORMAT);
                ///////////
                if (createUploadBuffer)
                {
                    videoTex.lumaStagingBuffer = CreateTextureUploadBuffer(m_device.Get(), videoTex.lumaTexture);
                    videoTex.chromaUStagingBuffer = CreateTextureUploadBuffer(m_device.Get(), videoTex.chromaTexture);// , 0, 2);
                    videoTex.chromaVStagingBuffer = CreateTextureUploadBuffer(m_device.Get(), videoTex.chromaVTexture);
                }

                //////////////
                // Describe and create a SRV for the texture.
                D3D12_SHADER_RESOURCE_VIEW_DESC srvDesc{
                    .Format = LUMA_FORMAT,
                    .ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D,
                    .Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING,
                    .Texture2D {
                        .MipLevels = 1,
                        .PlaneSlice = 0
                    }
                };
                m_device->CreateShaderResourceView(videoTex.lumaTexture.Get(), &srvDesc, cpuHandle);
                videoTex.lumaHandle = cpuHandle;
                videoTex.lumaGpuHandle = gpuHandle;
                cpuHandle.Offset(1, descriptorSize);
                gpuHandle.Offset(1, descriptorSize);

                srvDesc.Format = CHROMAU_FORMAT;
                m_device->CreateShaderResourceView(videoTex.chromaTexture.Get(), &srvDesc, cpuHandle);
                videoTex.chromaHandle = cpuHandle;
                videoTex.chromaGpuHandle = gpuHandle;
                cpuHandle.Offset(1, descriptorSize);
                gpuHandle.Offset(1, descriptorSize);

                srvDesc.Format = CHROMAV_FORMAT;
                m_device->CreateShaderResourceView(videoTex.chromaVTexture.Get(), &srvDesc, cpuHandle);
                videoTex.chromaVHandle = cpuHandle;
                videoTex.chromaVGpuHandle = gpuHandle;
                cpuHandle.Offset(1, descriptorSize);
                gpuHandle.Offset(1, descriptorSize);
            }
        }

        m_is3PlaneFormat = is3PlaneFmt;
    }

    virtual void CreateVideoTextures(const std::size_t width, const std::size_t height, const XrPixelFormat pixfmt) override
    {
        CreateVideoTextures(width, height, pixfmt, true);
    }

    ComPtr<ID3D11Device> m_d3d11Device{};
    ComPtr<ID3D11DeviceContext> m_d3d11DeviceContext{};
    ComPtr<ID3D11On12Device> m_d3d11On12Device{};

    virtual const void* GetD3D11AVDevice() const override
    {
        return m_d3d11Device.Get();
    }

    virtual void* GetD3D11AVDevice() override {
        return m_d3d11Device.Get();
    }

    virtual const void* GetD3D11VADeviceContext() const override {
        return m_d3d11DeviceContext.Get();
    }

    virtual void* GetD3D11VADeviceContext() override {
        return m_d3d11DeviceContext.Get();
    }

    void InitD3D11OnD3D12()
    {
        if (m_device == nullptr)
            return;
        UINT d3d11DeviceFlags = D3D11_CREATE_DEVICE_VIDEO_SUPPORT;// | D3D11_CREATE_DEVICE_BGRA_SUPPORT;//0;//
#ifndef NDEBUG
        d3d11DeviceFlags |= D3D11_CREATE_DEVICE_DEBUG;
#endif
        //const D3D_FEATURE_LEVEL features[] = { D3D_FEATURE_LEVEL_11_1, D3D_FEATURE_LEVEL_11_0 };
        CHECK_HRCMD(D3D11On12CreateDevice(
            m_device.Get(),
            d3d11DeviceFlags,
            nullptr,
            0,
            reinterpret_cast<IUnknown**>(m_videoTexCmdCpyQueue.GetAddressOf()),
            1,
            0,
            &m_d3d11Device,
            &m_d3d11DeviceContext,
            nullptr
        ));
        CHECK_HRCMD(m_d3d11Device.As(&m_d3d11On12Device));
        CHECK(m_d3d11Device != nullptr);
        ID3D10Multithread* pMultithread = nullptr;
        if (SUCCEEDED(m_d3d11Device->QueryInterface(__uuidof(ID3D10Multithread), (void**)&pMultithread))
            && pMultithread != nullptr) {
            pMultithread->SetMultithreadProtected(TRUE);
            pMultithread->Release();
        }
    }

    virtual void ClearVideoTextures() override
    {
        m_renderTex = std::size_t(-1);
        m_currentVideoTex = 0;
        //std::lock_guard<std::mutex> lk(m_renderMutex);
        m_texRendereComplete.WaitForGpu();
        m_videoTextures = { NV12Texture {}, NV12Texture {} };
        m_is3PlaneFormat = false;
    }

    virtual void CreateVideoTexturesD3D11VA(const std::size_t width, const std::size_t height, const XrPixelFormat pixfmt) override
    {
        if (m_d3d11Device == nullptr)
            return;

        CHECK_MSG((pixfmt != XrPixelFormat::G8_B8_R8_3PLANE_420 &&
                    pixfmt != XrPixelFormat::G10X6_B10X6_R10X6_3PLANE_420), "3-Planes formats are not supported!");
        
        CreateVideoTextures
        (
            width, height, pixfmt,
            false, D3D12_HEAP_FLAG_SHARED,
            D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET | D3D12_RESOURCE_FLAG_ALLOW_SIMULTANEOUS_ACCESS
        );

        constexpr const D3D11_RESOURCE_FLAGS d3d11Flags {
            .BindFlags = D3D11_BIND_SHADER_RESOURCE | D3D11_BIND_RENDER_TARGET,
            .MiscFlags = 0,
            .CPUAccessFlags = 0,            
            .StructureByteStride = 0,
        };
        const WindowsSecurityAttributes secAttr{};
        for (auto& vidTex : m_videoTextures)
        {
            CHECK_HRCMD(m_device->CreateSharedHandle(vidTex.texture.Get(), &secAttr, GENERIC_ALL, 0, &vidTex.wrappedD3D11SharedHandle));

            CHECK_HRCMD(m_d3d11On12Device->CreateWrappedResource
            (
                vidTex.texture.Get(),
                &d3d11Flags,
                D3D12_RESOURCE_STATE_COMMON,
                D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE,
                IID_PPV_ARGS(&vidTex.wrappedD3D11Texture)
            ));
            CHECK(vidTex.wrappedD3D11Texture != nullptr);
        }
    }

    virtual void UpdateVideoTextureD3D11VA(const YUVBuffer& yuvBuffer)
    {
        CHECK(m_device != nullptr);
        CHECK(m_videoTexCmdAllocator != nullptr);
        CHECK(yuvBuffer.frameIndex != std::uint64_t(-1));

        WaitForAvailableBuffer();

        using TextureRes = ComPtr<ID3D12Resource>;
        //TextureRes lumaUploadBuffer{}, chromaUploadBuffer{};
        const std::size_t freeIndex = m_currentVideoTex.load();
        {
            /*const*/ auto& videoTex = m_videoTextures[freeIndex];
            videoTex.frameIndex = yuvBuffer.frameIndex;
            CHECK(videoTex.wrappedD3D11Texture != nullptr);

            ComPtr<ID3D11Texture2D> new_texture = reinterpret_cast<ID3D11Texture2D*>(yuvBuffer.luma.data);
            const auto texture_index = (UINT)reinterpret_cast<std::intptr_t>(yuvBuffer.chroma.data);
            CHECK(new_texture != nullptr);

            D3D11_TEXTURE2D_DESC desc{};//, desc2{};
            videoTex.wrappedD3D11Texture->GetDesc(&desc);

            m_d3d11On12Device->AcquireWrappedResources((ID3D11Resource**)videoTex.wrappedD3D11Texture.GetAddressOf(), 1);
            
            const D3D11_BOX sourceRegion{
                .left = 0,
                .top = 0,
                .front = 0,
                .right = desc.Width,
                .bottom = desc.Height,                
                .back = 1
            };
            m_d3d11DeviceContext->CopySubresourceRegion(videoTex.wrappedD3D11Texture.Get(), 0, 0, 0, 0, new_texture.Get(), texture_index, &sourceRegion);
            
            // Release our wrapped render target resource. Releasing 
            // transitions the back buffer resource to the state specified
            // as the OutState when the wrapped resource was created.
            m_d3d11On12Device->ReleaseWrappedResources((ID3D11Resource**)videoTex.wrappedD3D11Texture.GetAddressOf(), 1);

            // Flush to submit the 11 command list to the shared command queue.
            m_d3d11DeviceContext->Flush();
        }

        m_currentVideoTex.store((freeIndex + 1) % VideoTexCount);
        //CHECK_HRCMD(m_texCopy.Signal(m_videoTexCmdCpyQueue));        
        m_renderTex.store(freeIndex);
    }

    bool WaitForAvailableBuffer()
    {
        m_texRendereComplete.WaitForGpu();
        //Log::Write(Log::Level::Info, Fmt("render idx: %d, copy idx: %d", m_renderTex.load(), m_currentVideoTex.load()));
        //CHECK_HRCMD(m_texRendereComplete.Wait(m_videoTexCmdCpyQueue));
        return true;
    }

    virtual void UpdateVideoTexture(const YUVBuffer& yuvBuffer) override
    {
        CHECK(m_device != nullptr);
        CHECK(m_videoTexCmdAllocator != nullptr);
        CHECK(yuvBuffer.frameIndex != std::uint64_t(-1));
        
        WaitForAvailableBuffer();

        ComPtr<ID3D12GraphicsCommandList> cmdList;
        CHECK_HRCMD(m_device->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT/*COPY*/, m_videoTexCmdAllocator.Get(), nullptr,
            __uuidof(ID3D12GraphicsCommandList),
            reinterpret_cast<void**>(cmdList.ReleaseAndGetAddressOf())));

        using TextureRes = ComPtr<ID3D12Resource>;
        //TextureRes lumaUploadBuffer{}, chromaUploadBuffer{};
        const std::size_t freeIndex = m_currentVideoTex.load();
        {
            auto& videoTex = m_videoTextures[freeIndex];
            videoTex.frameIndex = yuvBuffer.frameIndex;

            const bool is3PlaneFmt = yuvBuffer.chroma2.data != nullptr;
            if (!is3PlaneFmt)
            {
                CHECK(!m_is3PlaneFormat.load());
                CHECK(videoTex.texture != nullptr);
                CHECK(videoTex.uploadTexture != nullptr);
                
                const std::array<const D3D12_SUBRESOURCE_DATA,2> textureData
                {
                    D3D12_SUBRESOURCE_DATA {
                        .pData = yuvBuffer.luma.data,
                        .RowPitch = static_cast<LONG_PTR>(yuvBuffer.luma.pitch),
                        .SlicePitch = static_cast<LONG_PTR>(yuvBuffer.luma.pitch * yuvBuffer.luma.height)
                    },
                    D3D12_SUBRESOURCE_DATA {
                        .pData = yuvBuffer.chroma.data,
                        .RowPitch = static_cast<LONG_PTR>(yuvBuffer.chroma.pitch),
                        .SlicePitch = static_cast<LONG_PTR>(yuvBuffer.chroma.pitch * yuvBuffer.chroma.height),
                    }
                };
                CD3DX12_RESOURCE_BARRIER resourceBarrier =
                    CD3DX12_RESOURCE_BARRIER::Transition(videoTex.texture.Get(), D3D12_RESOURCE_STATE_COMMON, D3D12_RESOURCE_STATE_COPY_DEST);
                cmdList->ResourceBarrier(1, &resourceBarrier);

                UpdateSubresources
                (
                    cmdList.Get(), videoTex.texture.Get(), videoTex.uploadTexture.Get(),
                    0, 0, (UINT)textureData.size(), textureData.data()
                );

                resourceBarrier =
                    CD3DX12_RESOURCE_BARRIER::Transition(videoTex.texture.Get(), D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_COMMON);
                cmdList->ResourceBarrier(1, &resourceBarrier);
            }
            else
            {
                CHECK(m_is3PlaneFormat.load());
                CHECK(videoTex.lumaTexture != nullptr);
                CHECK(videoTex.chromaTexture != nullptr);
                CHECK(videoTex.chromaVTexture != nullptr);

                const auto uploadData = [&cmdList](const TextureRes& tex, const TextureRes& uploadBuff, const Buffer& buf)
                {
                    const auto texDesc = tex->GetDesc();
                    CHECK(buf.height <= texDesc.Height);
                    const D3D12_SUBRESOURCE_DATA textureData {
                        .pData = buf.data,
                        .RowPitch = static_cast<LONG_PTR>(buf.pitch),
                        .SlicePitch = static_cast<LONG_PTR>(buf.pitch * buf.height)
                    };
                    UpdateSubresources<1>(cmdList.Get(), tex.Get(), uploadBuff.Get(), 0, 0, 1, &textureData);
                };

                std::array<CD3DX12_RESOURCE_BARRIER, 3> resourceBarriers = {
                    CD3DX12_RESOURCE_BARRIER::Transition(videoTex.lumaTexture.Get(),    D3D12_RESOURCE_STATE_COMMON, D3D12_RESOURCE_STATE_COPY_DEST),
                    CD3DX12_RESOURCE_BARRIER::Transition(videoTex.chromaTexture.Get(),  D3D12_RESOURCE_STATE_COMMON, D3D12_RESOURCE_STATE_COPY_DEST),
                    CD3DX12_RESOURCE_BARRIER::Transition(videoTex.chromaVTexture.Get(), D3D12_RESOURCE_STATE_COMMON, D3D12_RESOURCE_STATE_COPY_DEST),
                };
                cmdList->ResourceBarrier((UINT)resourceBarriers.size(), resourceBarriers.data());

                uploadData(videoTex.lumaTexture,    videoTex.lumaStagingBuffer,    yuvBuffer.luma);
                uploadData(videoTex.chromaTexture,  videoTex.chromaUStagingBuffer, yuvBuffer.chroma);
                uploadData(videoTex.chromaVTexture, videoTex.chromaVStagingBuffer, yuvBuffer.chroma2);;

                resourceBarriers = {
                    CD3DX12_RESOURCE_BARRIER::Transition(videoTex.lumaTexture.Get(),    D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_COMMON),
                    CD3DX12_RESOURCE_BARRIER::Transition(videoTex.chromaTexture.Get(),  D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_COMMON),
                    CD3DX12_RESOURCE_BARRIER::Transition(videoTex.chromaVTexture.Get(), D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_COMMON),
                };
                cmdList->ResourceBarrier((UINT)resourceBarriers.size(), resourceBarriers.data());
            }
        }
        // END CMDS //////////////////////////////////////////////////////////////////////
        CHECK_HRCMD(cmdList->Close());
        ID3D12CommandList* cmdLists[] = { cmdList.Get() };
        m_videoTexCmdCpyQueue->ExecuteCommandLists((UINT)std::size(cmdLists), cmdLists);
        
        m_currentVideoTex.store((freeIndex + 1) % VideoTexCount);
        CHECK_HRCMD(m_texCopy.Signal(m_videoTexCmdCpyQueue));
        m_renderTex.store(freeIndex);
    }

    std::size_t currentTextureIdx = std::size_t(-1);

    virtual void BeginVideoView() override
    {
#if 0
#ifdef XR_ENABLE_CUDA_INTEROP
        const cudaExternalSemaphoreWaitParams externalSemaphoreWaitParams {
            .params{.fence{.value = m_texCopy.fenceValue.load()}},
            .flags = 0
        };
        if (cudaWaitExternalSemaphoresAsync(&m_m_texCopyExtSemaphore, &externalSemaphoreWaitParams, 1, videoBufferStream) != cudaSuccess)
        {
            Log::Write(Log::Level::Error, "cudaWaitExternalSemaphoresAsync failed.");
            CHECK(false);
        }
#endif
#else
        CHECK_HRCMD(m_texCopy.Wait(m_cmdQueue));
#endif
        currentTextureIdx = m_renderTex.load();
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
        if (cudaSignalExternalSemaphoresAsync
            (&m_m_texRenderExtSemaphore, &externalSemaphoreSignalParams, 1, videoBufferStream) != cudaSuccess)
        {
            Log::Write(Log::Level::Error, "m_texRendereComplete cudaSignalExternalSemaphoresAsync failed.");
            CHECK(false);
        }
        m_texRendereComplete.fenceValue.store(nextVal);
#endif
#else
        CHECK_HRCMD(m_texRendereComplete.Signal(m_cmdQueue));
#endif
    }

    virtual std::uint64_t GetVideoFrameIndex() const override {
        return currentTextureIdx == std::uint64_t(-1) ?
            currentTextureIdx :
            m_videoTextures[currentTextureIdx].frameIndex;
    }

    virtual void RenderVideoMultiView
    (
        const std::array<XrCompositionLayerProjectionView, 2>& layerViews, const XrSwapchainImageBaseHeader* swapchainImage,
        const std::int64_t swapchainFormat, const PassthroughMode newMode /*= PassthroughMode::None*/
    ) override
    {
        CHECK(m_isMultiViewSupported);
        using CpuDescHandle = D3D12_CPU_DESCRIPTOR_HANDLE;
        using CommandListPtr = ComPtr<ID3D12GraphicsCommandList>;
        RenderViewImpl(layerViews[0], swapchainImage, swapchainFormat, [&]
        (
            const CommandListPtr& cmdList,
            const CpuDescHandle& renderTargetView,
            const CpuDescHandle& depthStencilView,
            SwapchainImageContext& swapchainContext
        )
        {
            if (currentTextureIdx == std::size_t(-1))
                return;
            const auto& videoTex = m_videoTextures[currentTextureIdx];
            
            ID3D12DescriptorHeap* const ppHeaps[] = { m_srvHeap.Get() };
            cmdList->SetDescriptorHeaps((UINT)std::size(ppHeaps), ppHeaps);
            cmdList->SetGraphicsRootDescriptorTable(RootParamIndex::LumaTexture, videoTex.lumaGpuHandle); // Second texture will be (texture1+1)
            cmdList->SetGraphicsRootDescriptorTable(RootParamIndex::ChromaTexture, videoTex.chromaGpuHandle);
            if (m_is3PlaneFormat)
                cmdList->SetGraphicsRootDescriptorTable(RootParamIndex::ChromaVTexture, videoTex.chromaVGpuHandle);
            if (m_fovDecodeParams)
                cmdList->SetGraphicsRootConstantBufferView(RootParamIndex::FoveatedDecodeParams, swapchainContext.GetFoveationParamCBuffer()->GetGPUVirtualAddress());
            
            cmdList->ClearRenderTargetView(renderTargetView, ALXR::VideoClearColors[ClearColorIndex(newMode)], 0, nullptr);
            cmdList->ClearDepthStencilView(depthStencilView, D3D12_CLEAR_FLAG_DEPTH, 1.0f, 0, 0, nullptr);

            const D3D12_CPU_DESCRIPTOR_HANDLE renderTargets[] = { renderTargetView };
            cmdList->OMSetRenderTargets((UINT)std::size(renderTargets), renderTargets, true, &depthStencilView);

            cmdList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

            // Draw Video Quad
            cmdList->DrawInstanced(3, 1, 0, 0);
        }, RenderPipelineType::Video, newMode);
    }

    virtual void RenderVideoView(
        const std::array<XrCompositionLayerProjectionView, 2>& layerViews,
        const std::array<XrSwapchainImageBaseHeader*, 2>& swapchainImages,
        const std::int64_t swapchainFormat, const PassthroughMode newMode /*= PassthroughMode::None*/
    ) override
    {
        for (std::uint32_t viewID = 0; viewID < layerViews.size(); ++viewID) {
            const auto& layerView = layerViews[viewID];
            CHECK(layerView.subImage.imageArrayIndex == 0);
            using CpuDescHandle = D3D12_CPU_DESCRIPTOR_HANDLE;
            using CommandListPtr = ComPtr<ID3D12GraphicsCommandList>;
            RenderViewImpl(layerView, swapchainImages[viewID], swapchainFormat, [&]
            (
                const CommandListPtr& cmdList,
                const CpuDescHandle& renderTargetView,
                const CpuDescHandle& depthStencilView,
                SwapchainImageContext& swapchainContext
            )
            {
                if (currentTextureIdx == std::size_t(-1))
                    return;
                const auto& videoTex = m_videoTextures[currentTextureIdx];

                ID3D12DescriptorHeap* const ppHeaps[] = { m_srvHeap.Get() };
                cmdList->SetDescriptorHeaps((UINT)std::size(ppHeaps), ppHeaps);
                cmdList->SetGraphicsRootDescriptorTable(RootParamIndex::LumaTexture, videoTex.lumaGpuHandle); // Second texture will be (texture1+1)
                cmdList->SetGraphicsRootDescriptorTable(RootParamIndex::ChromaTexture, videoTex.chromaGpuHandle);
                if (m_is3PlaneFormat)
                    cmdList->SetGraphicsRootDescriptorTable(RootParamIndex::ChromaVTexture, videoTex.chromaVGpuHandle);
                if (m_fovDecodeParams)
                    cmdList->SetGraphicsRootConstantBufferView(RootParamIndex::FoveatedDecodeParams, swapchainContext.GetFoveationParamCBuffer()->GetGPUVirtualAddress());

                // Set shaders and constant buffers.
                ID3D12Resource* const viewProjectionCBuffer = swapchainContext.GetViewProjectionCBuffer();
                {
                    const ALXR::ViewProjectionConstantBuffer viewProjection{ .ViewID = viewID };
                    constexpr const D3D12_RANGE NoReadRange{ 0, 0 };
                    void* data = nullptr;
                    CHECK_HRCMD(viewProjectionCBuffer->Map(0, &NoReadRange, &data));
                    assert(data != nullptr);
                    std::memcpy(data, &viewProjection, sizeof(viewProjection));
                    viewProjectionCBuffer->Unmap(0, nullptr);
                }
                cmdList->SetGraphicsRootConstantBufferView(RootParamIndex::ViewProjTransform, viewProjectionCBuffer->GetGPUVirtualAddress());

                // Clear swapchain and depth buffer. NOTE: This will clear the entire render target view, not just the specified view.
                cmdList->ClearRenderTargetView(renderTargetView, ALXR::VideoClearColors[ClearColorIndex(newMode)], 0, nullptr);
                cmdList->ClearDepthStencilView(depthStencilView, D3D12_CLEAR_FLAG_DEPTH, 1.0f, 0, 0, nullptr);

                const D3D12_CPU_DESCRIPTOR_HANDLE renderTargets[] = { renderTargetView };
                cmdList->OMSetRenderTargets((UINT)std::size(renderTargets), renderTargets, true, &depthStencilView);

                cmdList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

                // Draw Video Quad
                cmdList->DrawInstanced(3, 1, 0, 0);
            }, RenderPipelineType::Video, newMode);
        }
    }

    virtual inline void SetEnvironmentBlendMode(const XrEnvironmentBlendMode newMode) override {
        static_assert(XR_ENVIRONMENT_BLEND_MODE_OPAQUE == 1);
        static_assert(ALXR::ClearColors.size() >= 3);
        static_assert(ALXR::VideoClearColors.size() >= 3);
        assert(newMode > 0 && newMode < 4);
        m_clearColorIndex = (newMode - 1);
    }

    virtual void SetFoveatedDecode(const ALXR::FoveatedDecodeParams* newFovDecParm) override {
        const auto fovDecodeParams = m_fovDecodeParams;
        const bool changePipelines = (fovDecodeParams == nullptr && newFovDecParm != nullptr) ||
                                     (fovDecodeParams != nullptr && newFovDecParm == nullptr);        
        if (changePipelines) {
            m_VideoPipelineStates.clear();
        }
        if (newFovDecParm) {
            for (auto& swapchainCtx : m_swapchainImageContexts)
                swapchainCtx.SetFoveationDecodeData(*newFovDecParm);
        }
        m_fovDecodeParams = newFovDecParm ?
            std::make_shared<ALXR::FoveatedDecodeParams>(*newFovDecParm) : nullptr;
    }

    virtual inline bool IsMultiViewEnabled() const override {
        return m_isMultiViewSupported;
    }

    using ID3D12CommandQueuePtr = ComPtr<ID3D12CommandQueue>;

#include "cuda/d3d12cuda_interop.inl"

   private:
    CoreShaders m_coreShaders{};
    ComPtr<ID3D12Device> m_device;
    LUID             m_dx12deviceluid{};
    ComPtr<ID3D12CommandQueue> m_cmdQueue;
    ComPtr<ID3D12Fence> m_fence;
    uint64_t m_fenceValue = 0;
    HANDLE m_fenceEvent = INVALID_HANDLE_VALUE;
    std::list<SwapchainImageContext> m_swapchainImageContexts;
    std::map<const XrSwapchainImageBaseHeader*, SwapchainImageContext*> m_swapchainImageContextMap;
    XrGraphicsBindingD3D12KHR m_graphicsBinding{
        .type = XR_TYPE_GRAPHICS_BINDING_D3D12_KHR,
        .next = nullptr
    };
    ComPtr<ID3D12RootSignature> m_rootSignature;
    std::map<DXGI_FORMAT, ComPtr<ID3D12PipelineState>> m_pipelineStates;
    ComPtr<ID3D12Resource> m_cubeVertexBuffer;
    ComPtr<ID3D12Resource> m_cubeIndexBuffer;

    static_assert(XR_ENVIRONMENT_BLEND_MODE_OPAQUE == 1);
    std::size_t m_clearColorIndex{ (XR_ENVIRONMENT_BLEND_MODE_OPAQUE - 1) };
    ////////////////////////////////////////////////////////////////////////

    D3D12FenceEvent                 m_texRendereComplete {};
    D3D12FenceEvent                 m_texCopy {};
    constexpr static const std::size_t VideoTexCount = 2;
    struct NV12Texture {

        CD3DX12_CPU_DESCRIPTOR_HANDLE lumaHandle{};
        CD3DX12_CPU_DESCRIPTOR_HANDLE chromaHandle{};
        CD3DX12_CPU_DESCRIPTOR_HANDLE chromaVHandle{};

        CD3DX12_GPU_DESCRIPTOR_HANDLE lumaGpuHandle{};
        CD3DX12_GPU_DESCRIPTOR_HANDLE chromaGpuHandle{};
        CD3DX12_GPU_DESCRIPTOR_HANDLE chromaVGpuHandle{};

        // NV12
        ComPtr<ID3D12Resource>  texture{};
        ComPtr<ID3D12Resource>  uploadTexture{};
        ComPtr<ID3D11Texture2D> wrappedD3D11Texture{};
        HANDLE                  wrappedD3D11SharedHandle = INVALID_HANDLE_VALUE;
        HANDLE                  d3d11TextureSharedHandle = INVALID_HANDLE_VALUE;

        // P010LE / CUDA / 3-Plane Fmts
        ComPtr<ID3D12Resource> lumaTexture{};
        ComPtr<ID3D12Resource> chromaTexture{};
        ComPtr<ID3D12Resource> chromaVTexture{};

        ComPtr<ID3D12Resource> lumaStagingBuffer{};
        ComPtr<ID3D12Resource> chromaUStagingBuffer {};
        ComPtr<ID3D12Resource> chromaVStagingBuffer {};

        std::uint64_t          frameIndex = std::uint64_t(-1);
    };
    std::array<NV12Texture, VideoTexCount> m_videoTextures{};
    std::atomic<std::size_t>       m_currentVideoTex{ (std::size_t)0 }, m_renderTex{ (std::size_t)-1 };
    std::atomic<bool>              m_is3PlaneFormat{ false };
    NV12Texture                    m_videoTexUploadBuffers{};
    //std::mutex                     m_renderMutex{};

    ComPtr<ID3D12CommandAllocator> m_videoTexCmdAllocator{};
    ComPtr<ID3D12CommandQueue>     m_videoTexCmdCpyQueue{};

    using ID3D12PipelineStatePtr = ComPtr<ID3D12PipelineState>;
    using VidePipelineStateList = std::array<ID3D12PipelineStatePtr, VideoPShader::TypeCount>;
    using PipelineStateMap = std::unordered_map<DXGI_FORMAT, VidePipelineStateList>;
    PipelineStateMap m_VideoPipelineStates;

    ComPtr<ID3D12DescriptorHeap> m_srvHeap{};
    ////////////////////////////////////////////////////////////////////////
    ComPtr<ID3D12DescriptorHeap> m_rtvHeap;
    ComPtr<ID3D12DescriptorHeap> m_dsvHeap;
    
    using FoveatedDecodeParamsPtr = SwapchainImageContext::FoveatedDecodeParamsPtr;
    FoveatedDecodeParamsPtr m_fovDecodeParams{};

    bool m_isMultiViewSupported = false;
};

}  // namespace

std::shared_ptr<IGraphicsPlugin> CreateGraphicsPlugin_D3D12(const std::shared_ptr<Options>& options,
                                                            std::shared_ptr<IPlatformPlugin> platformPlugin) {
    return std::make_shared<D3D12GraphicsPlugin>(options, platformPlugin);
}

#endif
