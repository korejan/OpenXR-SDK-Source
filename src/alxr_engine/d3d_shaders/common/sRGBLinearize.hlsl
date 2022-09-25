
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
float3 sRGBToLinearRGB(float3 lrgb)
{
    const float r = sRGBToLinearRGBScalar(lrgb.r);
    const float g = sRGBToLinearRGBScalar(lrgb.g);
    const float b = sRGBToLinearRGBScalar(lrgb.b);
    return float3(r, g, b);
}
