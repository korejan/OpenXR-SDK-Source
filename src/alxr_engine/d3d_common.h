// Copyright (c) 2017-2022, The Khronos Group Inc.
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#if defined(XR_USE_GRAPHICS_API_D3D11) || defined(XR_USE_GRAPHICS_API_D3D12)

#include <array>
#include <vector>
#include <string>
#include <filesystem>
#include <DirectXMath.h>

struct alignas(16) ModelConstantBuffer {
    DirectX::XMFLOAT4X4 Model;
};
struct alignas(16) ViewProjectionConstantBuffer {
    DirectX::XMFLOAT4X4 ViewProjection;
    std::uint32_t ViewID;
};

struct alignas(16) MultiViewProjectionConstantBuffer {
    DirectX::XMFLOAT4X4 ViewProjection[2];
};

// Separate entrypoints for the vertex and pixel shader functions.
constexpr inline char MultiViewShaderHlsl[] = R"_(
    struct PSVertex {
        float4 Pos : SV_POSITION;
        float3 Color : COLOR0;
        uint ViewId : SV_RenderTargetArrayIndex;
    };
    struct Vertex {
        float3 Pos : POSITION;
        float3 Color : COLOR0;
        uint InstId : SV_InstanceID;
    };
    cbuffer ModelConstantBuffer : register(b0) {
        float4x4 Model;
    };
    cbuffer ViewProjectionConstantBuffer : register(b1) {
        float4x4 ViewProjection[2];
    };

    PSVertex MainVS(Vertex input) {
        PSVertex output;
        output.Pos = mul(mul(float4(input.Pos, 1), Model), ViewProjection[input.InstId]);
        output.Color = input.Color;
        output.ViewId = input.InstId;
        return output;
    }

    float4 MainPS(PSVertex input) : SV_TARGET {
        return float4(input.Color, 1);
    }
    )_";

constexpr inline char ShaderHlsl[] = R"_(
    struct PSVertex {
        float4 Pos : SV_POSITION;
        float3 Color : COLOR0;
    };
    struct Vertex {
        float3 Pos : POSITION;
        float3 Color : COLOR0;
    };
    cbuffer ModelConstantBuffer : register(b0) {
        float4x4 Model;
    };
    cbuffer ViewProjectionConstantBuffer : register(b1) {
        float4x4 ViewProjection;
    };

    PSVertex MainVS(Vertex input) {
       PSVertex output;
       output.Pos = mul(mul(float4(input.Pos, 1), Model), ViewProjection);
       output.Color = input.Color;
       return output;
    }

    float4 MainPS(PSVertex input) : SV_TARGET {
        return float4(input.Color, 1);
    }
    )_";

DirectX::XMMATRIX XM_CALLCONV LoadXrPose(const XrPosef& pose);
DirectX::XMMATRIX XM_CALLCONV LoadXrMatrix(const XrMatrix4x4f& matrix);

std::vector<std::uint8_t> LoadCompiledShaderObject(const std::filesystem::path& csoFile);
Microsoft::WRL::ComPtr<ID3DBlob> CompileShader(const char* hlsl, const char* entrypoint, const char* shaderTarget);
Microsoft::WRL::ComPtr<IDXGIAdapter1> GetAdapter(LUID adapterId);

constexpr inline DXGI_FORMAT GetLumaFormat(const DXGI_FORMAT yuvFmt) {
    switch (yuvFmt) {
    case DXGI_FORMAT::DXGI_FORMAT_NV12: return DXGI_FORMAT_R8_UNORM;
    case DXGI_FORMAT::DXGI_FORMAT_P010: return DXGI_FORMAT_R16_UNORM;
    }
    return DXGI_FORMAT::DXGI_FORMAT_UNKNOWN;
}

constexpr inline DXGI_FORMAT GetChromaFormat(const DXGI_FORMAT yuvFmt) {
    switch (yuvFmt) {
    case DXGI_FORMAT::DXGI_FORMAT_NV12: return DXGI_FORMAT_R8G8_UNORM;
    case DXGI_FORMAT::DXGI_FORMAT_P010: return DXGI_FORMAT_R16G16_UNORM;
    }
    return DXGI_FORMAT::DXGI_FORMAT_UNKNOWN;
}

constexpr inline DXGI_FORMAT GetChromaUFormat(const DXGI_FORMAT yuvFmt) {
    switch (yuvFmt) {
    case DXGI_FORMAT::DXGI_FORMAT_R8G8_UNORM: return DXGI_FORMAT_R8_UNORM;
    case DXGI_FORMAT::DXGI_FORMAT_R16G16_UNORM: return DXGI_FORMAT_R16_UNORM;
    }
    return DXGI_FORMAT::DXGI_FORMAT_UNKNOWN;
}

constexpr inline DXGI_FORMAT GetChromaVFormat(const DXGI_FORMAT yuvFmt) {
    switch (yuvFmt) {
    case DXGI_FORMAT::DXGI_FORMAT_R8G8_UNORM: return DXGI_FORMAT_R8_UNORM;
    case DXGI_FORMAT::DXGI_FORMAT_R16G16_UNORM: return DXGI_FORMAT_R16_UNORM;
    }
    return DXGI_FORMAT::DXGI_FORMAT_UNKNOWN;
}

constexpr inline const char VideoVShaderHlsl[] = R"_(
    struct PSVertex {
        float4 Pos : SV_POSITION;
        float2 uv : TEXCOORD;
    };
    struct Vertex {
        float3 Pos : POSITION;
        float2 uv : TEXCOORD;
    };
    cbuffer ModelConstantBuffer : register(b0) {
        float4x4 Model;
    };
    cbuffer ViewProjectionConstantBuffer : register(b1) {
        float4x4 ViewProjection;
        uint ViewID;
    };

    PSVertex MainVS(Vertex input) {
        PSVertex output;
        output.Pos = mul(mul(float4(input.Pos, 1), Model), ViewProjection);
        output.uv = input.uv;
        if (ViewID > 0) {
            output.uv.x += 0.5f;
        }
        return output;
    }
    )_";

constexpr inline const char VideoMultiViewVShaderHlsl[] = R"_(
    struct PSVertex {
        float4 Pos : SV_POSITION;
        float2 uv : TEXCOORD;
        uint ViewId : SV_RenderTargetArrayIndex;
    };
    struct Vertex {
        float3 Pos : POSITION;
        float2 uv : TEXCOORD;
        uint InstId : SV_InstanceID;
    };
    cbuffer ModelConstantBuffer : register(b0) {
        float4x4 Model;
    };
    cbuffer ViewProjectionConstantBuffer : register(b1) {
        float4x4 ViewProjection[2];
    };

    PSVertex MainVS(Vertex input) {
        PSVertex output;
        output.Pos = mul(mul(float4(input.Pos, 1), Model), ViewProjection[input.InstId]);
        output.ViewId = input.InstId;
        output.uv = input.uv;
        if (input.InstId > 0) {
            output.uv.x += 0.5f;
        }
        return output;
    }
    )_";

constexpr inline const char VideoPShaderHlsl[] = R"_(
    Texture2D<float>  tex_y : register(t0);
    Texture2D<float2> tex_uv : register(t1);
    Texture2D<float>  tex_v : register(t2);
    
    SamplerState y_sampler : register(s0);
    SamplerState uv_sampler : register(s1);

    // Derived from https://msdn.microsoft.com/en-us/library/windows/desktop/dd206750(v=vs.85).aspx
    // Section: Converting 8-bit YUV to RGB888
    static const float3x3 YUVtoRGBCoeffMatrix = 
    {
        1.164383f,  1.164383f, 1.164383f,
        0.000000f, -0.391762f, 2.017232f,
        1.596027f, -0.812968f, 0.000000f
    };

    float3 ConvertYUVtoRGB(float3 yuv)
    {
        // Derived from https://msdn.microsoft.com/en-us/library/windows/desktop/dd206750(v=vs.85).aspx
        // Section: Converting 8-bit YUV to RGB888

        // These values are calculated from (16 / 255) and (128 / 255)
        yuv -= float3(0.062745f, 0.501960f, 0.501960f);
        yuv = mul(yuv, YUVtoRGBCoeffMatrix);

        return saturate(yuv);
    }

    float sRGBToLinearRGBScalar(float x)
    {
        static const float delta = 1.0 / 12.92;
        static const float alpha = 1.0 / 1.055;
        return (x < 0.04045) ?
            (x * delta) : pow(((x + 0.055) * alpha), 2.4);
    }

    // conversion based on: https://www.khronos.org/registry/DataFormat/specs/1.3/dataformat.1.3.html#TRANSFER_SRGB
    float4 sRGBToLinearRGB(float4 lrgb)
    {
        const float r = sRGBToLinearRGBScalar(lrgb.r);
        const float g = sRGBToLinearRGBScalar(lrgb.g);
        const float b = sRGBToLinearRGBScalar(lrgb.b);
        return float4(r,g,b,lrgb.a);
    }

    float4 MainPS(PSVertex input) : SV_TARGET {
        float y = tex_y.Sample(y_sampler, input.uv);
        float2 uv = tex_uv.Sample(y_sampler, input.uv);
        float3 rgb = ConvertYUVtoRGB(float3(y, uv));
        return sRGBToLinearRGB(float4(rgb,1.0f));
    }

    float4 MainBlendPS(PSVertex input) : SV_TARGET {
        float y = tex_y.Sample(y_sampler, input.uv);
        float2 uv = tex_uv.Sample(y_sampler, input.uv);
        float3 rgb = ConvertYUVtoRGB(float3(y, uv));
        return sRGBToLinearRGB(float4(rgb,0.6f));
    }

    static const float3 MaskKeyColor = float3(0.01, 0.01, 0.01);

    float4 MainMaskPS(PSVertex input) : SV_TARGET {
        float y = tex_y.Sample(y_sampler, input.uv);
        float2 uv = tex_uv.Sample(y_sampler, input.uv);
        float3 rgb = ConvertYUVtoRGB(float3(y, uv));
        
        float alpha = all(rgb < MaskKeyColor) ? 0.3f : 1.0f;
        
        return sRGBToLinearRGB(float4(rgb,alpha));
    }

    float4 Main3PlaneFmtPS(PSVertex input) : SV_TARGET {
        float y = tex_y.Sample(y_sampler, input.uv);
        float u = tex_uv.Sample(y_sampler, input.uv).r;
        float v = tex_v.Sample(y_sampler, input.uv);
        float3 rgb = ConvertYUVtoRGB(float3(y,u,v));
        return sRGBToLinearRGB(float4(rgb,1.0f));
    }

    float4 MainBlend3PlaneFmtPS(PSVertex input) : SV_TARGET {
        float y = tex_y.Sample(y_sampler, input.uv);
        float u = tex_uv.Sample(y_sampler, input.uv).r;
        float v = tex_v.Sample(y_sampler, input.uv);
        float3 rgb = ConvertYUVtoRGB(float3(y,u,v));
        return sRGBToLinearRGB(float4(rgb,1.0f));
    }

    float4 MainMask3PlaneFmtPS(PSVertex input) : SV_TARGET {
        float y = tex_y.Sample(y_sampler, input.uv);
        float u = tex_uv.Sample(y_sampler, input.uv).r;
        float v = tex_v.Sample(y_sampler, input.uv);
        float3 rgb = ConvertYUVtoRGB(float3(y,u,v));
        
        float alpha = all(rgb < MaskKeyColor) ? 0.3f : 1.0f;

        return sRGBToLinearRGB(float4(rgb,alpha));
    }
    )_";

constexpr inline std::string MakeVideoShaderHlslStr(const bool withMultiVew) {
    const auto& videoVertexShader = withMultiVew ? VideoMultiViewVShaderHlsl : VideoVShaderHlsl;
    std::string videoShaderStr = videoVertexShader;
    videoShaderStr += '\n';
    videoShaderStr += VideoPShaderHlsl;
    return videoShaderStr;
}

namespace ALXR {
    using CColorType = DirectX::XMVECTORF32;
    
    constexpr inline const std::array<const float, 3> DarkSlateGray { 0.184313729f, 0.309803933f, 0.309803933f };
    constexpr inline const std::array<const float, 3> CClear { 0.0f, 0.0f, 0.0f };
    
    constexpr inline const std::array<const CColorType, 4> ClearColors { CColorType
        // OpaqueClear - DirectX::Colors::DarkSlateGray
        {DarkSlateGray[0], DarkSlateGray[1], DarkSlateGray[2], 1.0f},
        // AdditiveClear
        { CClear[0], CClear[1], CClear[2], 0.0f },
        // AlphaBlendClear
        { CClear[0], CClear[1], CClear[2], 0.5f },
        // OpaqueClear - for XR_FB_passthrough / Passthrough Modes.
        {DarkSlateGray[0], DarkSlateGray[1], DarkSlateGray[2], 0.2f},
    };
    constexpr inline const std::array<const CColorType, 4> VideoClearColors { CColorType
        // OpaqueClear
        { CClear[0], CClear[1], CClear[2], 1.0f},
        // AdditiveClear
        { CClear[0], CClear[1], CClear[2], 0.0f },
        // AlphaBlendClear
        { CClear[0], CClear[1], CClear[2], 0.5f },
        // OpaqueClear - for XR_FB_passthrough / Passthrough Modes.
        { CClear[0], CClear[1], CClear[2], 0.2f },
    };
}
#endif
