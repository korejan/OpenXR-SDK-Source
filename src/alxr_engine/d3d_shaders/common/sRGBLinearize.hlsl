
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

// conversion based on: https://www.khronos.org/registry/DataFormat/specs/1.3/dataformat.1.3.html#TRANSFER_SRGB
float3 sRGBToLinearRGB(float3 srgb)
{
    static const float delta = 1.0 / 12.92;
    static const float alpha = 1.0 / 1.055;
    static const float3 delta3  = float3(delta, delta, delta);
    static const float3 alpha3  = float3(alpha, alpha, alpha);
    static const float3 theta3  = float3(0.04045, 0.04045, 0.04045);
    static const float3 offset3 = float3(0.055, 0.055, 0.055);
    static const float3 gamma3  = float3(2.4, 2.4, 2.4);
    const float3 lower = srgb * delta3;
    const float3 upper = pow((srgb + offset3) * alpha3, gamma3);
    return lerp(upper, lower, srgb < theta3);
}
