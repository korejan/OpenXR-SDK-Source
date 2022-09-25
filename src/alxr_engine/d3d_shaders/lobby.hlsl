#if defined(ENABLE_SM5_MULTI_VIEW) || defined(ENABLE_SM6_MULTI_VIEW)
    #define MULTI_VIEW_ENABLED
#endif

struct PSVertex {
    float4 Pos : SV_POSITION;
    float3 Color : COLOR0;
#ifdef ENABLE_SM5_MULTI_VIEW
    uint ViewId : SV_RenderTargetArrayIndex;
#endif
};
struct Vertex {
    float3 Pos : POSITION;
    float3 Color : COLOR0;

#ifdef ENABLE_SM6_MULTI_VIEW
    uint ViewId : SV_ViewID;
#elif defined(ENABLE_SM5_MULTI_VIEW)
    uint ViewId : SV_InstanceID;
#endif
};

cbuffer ModelConstantBuffer : register(b0) {
    float4x4 Model;
};
cbuffer ViewProjectionConstantBuffer : register(b1) {
#ifdef MULTI_VIEW_ENABLED
    float4x4 ViewProjection[2];
#else
    float4x4 ViewProjection;
#endif
};

PSVertex MainVS(Vertex input) {
    PSVertex output;

    output.Pos = mul(mul(float4(input.Pos, 1), Model),
#ifdef MULTI_VIEW_ENABLED
        ViewProjection[input.ViewId]);
#else
        ViewProjection);
#endif
    output.Color = input.Color;

#ifdef ENABLE_SM5_MULTI_VIEW
    output.ViewId = input.ViewId;
#endif
    return output;
}

float4 MainPS(PSVertex input) : SV_TARGET {
    return float4(input.Color, 1);
}
