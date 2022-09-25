#include "common/baseVideoFrag.hlsl"

static const float3 MaskKeyColor = float3(0.01, 0.01, 0.01);

float4 MainPS
(
    PSVertex input
#if (defined (ENABLE_FOVEATION_DECODE) && defined(ENABLE_SM6_MULTI_VIEW))
    , in uint ViewId : SV_ViewID
#endif
) : SV_TARGET
{
    const float3 rgb = PS_SAMPLE_VIDEO_TEXTURE(input);
    const float alpha = all(rgb < MaskKeyColor) ? 0.3f : 1.0f;
    return float4(rgb, alpha);
}
