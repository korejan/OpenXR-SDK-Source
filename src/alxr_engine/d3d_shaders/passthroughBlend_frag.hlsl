#include "common/baseVideoFrag.hlsl"

float4 MainPS
(
    PSVertex input
#if (defined (ENABLE_FOVEATION_DECODE) && defined(ENABLE_SM6_MULTI_VIEW))
    , in uint ViewId : SV_ViewID
#endif
) : SV_TARGET
{
    return float4(PS_SAMPLE_VIDEO_TEXTURE(input), 0.6f);
}
