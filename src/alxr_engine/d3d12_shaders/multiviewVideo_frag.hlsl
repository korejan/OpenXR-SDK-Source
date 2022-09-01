#include "multiviewVideo_vert.hlsl"
#include "common/sRGBLinearize.hlsl"

Texture2D<float>  tex_y : register(t0);
Texture2D<float2> tex_uv : register(t1);

SamplerState y_sampler : register(s0);
SamplerState uv_sampler : register(s1);

float4 MainPS(PSVertex input) : SV_TARGET {
    float y = tex_y.Sample(y_sampler, input.uv);
    float2 uv = tex_uv.Sample(y_sampler, input.uv);
    float3 rgb = ConvertYUVtoRGB(float3(y, uv));
    return sRGBToLinearRGB(float4(rgb,1.0f));
}
