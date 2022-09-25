#if defined(ENABLE_SM5_MULTI_VIEW) || defined(ENABLE_SM6_MULTI_VIEW)
    #define MULTI_VIEW_ENABLED
#endif

struct PSVertex {
    float4 Pos : SV_POSITION;
    float2 uv : TEXCOORD;
#ifdef ENABLE_SM5_MULTI_VIEW
    uint ViewId : SV_RenderTargetArrayIndex;
#endif
};

struct Vertex {
    float3 Pos : POSITION;
    float2 uv : TEXCOORD;

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
    uint ViewID;
#endif
};

#ifdef MULTI_VIEW_ENABLED
    #define VS_GET_VIEW_INDEX(input) input.ViewId
    #define VS_GET_VIEW_PROJ(input) ViewProjection[input.ViewId]
#else
    #define VS_GET_VIEW_INDEX(input) ViewID
    #define VS_GET_VIEW_PROJ(input) ViewProjection
#endif

PSVertex MainVS(Vertex input) {
    PSVertex output;
    output.Pos = mul(mul(float4(input.Pos, 1), Model), VS_GET_VIEW_PROJ(input));
    
    output.uv = input.uv;    
    if (VS_GET_VIEW_INDEX(input) > 0) {
        output.uv.x += 0.5f;
    }

#ifdef ENABLE_SM5_MULTI_VIEW
    output.ViewId = input.ViewId;
#endif
    return output;
}
