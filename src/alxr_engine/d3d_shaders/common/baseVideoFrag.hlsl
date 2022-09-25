#include "../videoStream_vert.hlsl"

#ifdef ENABLE_3PLANE_FMT_SAMPLER
    #include "yuv3PlaneSampler.hlsl"
#else
    #include "yuvSampler.hlsl"
#endif

#ifdef ENABLE_FOVEATION_DECODE
    #include "decodeFoveation.hlsl"
#endif

#ifdef ENABLE_SM6_MULTI_VIEW
    #define PS_GET_VIEW_INDEX(input) ViewId
#elif defined(ENABLE_SM5_MULTI_VIEW)
    #define PS_GET_VIEW_INDEX(input) input.ViewId
#else
    #define PS_GET_VIEW_INDEX(input) uint(input.uv.x > 0.5f)
#endif

#ifdef ENABLE_FOVEATION_DECODE
    #define PS_SAMPLE_VIDEO_TEXTURE(input)\
        SampleVideoTexture(input.uv, PS_GET_VIEW_INDEX(input))
#else
    #define PS_SAMPLE_VIDEO_TEXTURE(input)\
        SampleVideoTexture(input.uv)
#endif

float3 SampleVideoTexture
(
    const float2 uv
#ifdef ENABLE_FOVEATION_DECODE
    , float rightEye
#endif
)
{
    return sRGBToLinearRGB(SampleYUV
    (
#ifdef ENABLE_FOVEATION_DECODE
        DecodeFoveationUV(uv, rightEye)
#else
        uv
#endif
    ));
}
