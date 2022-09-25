#include "sRGBLinearize.hlsl"

Texture2D<float>  tex_y : register(t0);
Texture2D<float2> tex_uv : register(t1);

SamplerState y_sampler : register(s0);
SamplerState uv_sampler : register(s1);

float3 SampleYUV(float2 texuv) {
	const float y = tex_y.Sample(y_sampler, texuv);
	const float2 uv = tex_uv.Sample(y_sampler, texuv);
	return ConvertYUVtoRGB(float3(y, uv));
}
