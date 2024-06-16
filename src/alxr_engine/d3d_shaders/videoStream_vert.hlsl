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


#ifndef MULTI_VIEW_ENABLED
cbuffer ViewIDConstantBuffer : register(b0) {
    float4x4 ViewProjection;
    uint ViewId;
};
    #define IMAGE_SCALE_CB_REG b1
#else
    #define IMAGE_SCALE_CB_REG b0
#endif

cbuffer ImageScaleCB : register(IMAGE_SCALE_CB_REG) {
    float2 ImageScale;
};

static const float2 TriPositions[3] =
{
    float2(-1.0f, -1.0f),
    float2(-1.0f, 3.0f),
    float2(3.0f, -1.0f)
};
static const float2 UVs[2][3] =
{
    {
        float2(0.0f, 1.0f),
        float2(0.0f, -1.0f),
        float2(1.0f, 1.0f)
    },
    {
        float2(0.5f, 1.0f),
        float2(0.5f, -1.0f),
        float2(1.5f, 1.0f)
    }
};

PSVertex MainVS(uint VertexID : SV_VertexID
#ifdef ENABLE_SM6_MULTI_VIEW
    , uint ViewId : SV_ViewID
#elif defined(ENABLE_SM5_MULTI_VIEW)
    , uint ViewId : SV_InstanceID
#endif
)
{
    PSVertex output;
    output.Pos = float4(TriPositions[VertexID], 0.0f, 1.0f);
    output.uv = UVs[ViewId][VertexID] * ImageScale;
#ifdef ENABLE_SM5_MULTI_VIEW
    output.ViewId = ViewId;
#endif
    return output;
}
