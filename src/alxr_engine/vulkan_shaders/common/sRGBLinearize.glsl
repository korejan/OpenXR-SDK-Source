const float delta = 1.0 / 12.92;
const float alpha = 1.0 / 1.055;

float sRGBToLinearRGBScalar(float x)
{
    return (x < 0.04045) ?
        (x * delta) : pow((x + 0.055) * alpha, 2.4);
}

// conversion based on: https://www.khronos.org/registry/DataFormat/specs/1.3/dataformat.1.3.html#TRANSFER_SRGB
vec4 sRGBToLinearRGB(vec4 lrgb)
{
    const float r = sRGBToLinearRGBScalar(lrgb.r);
    const float g = sRGBToLinearRGBScalar(lrgb.g);
    const float b = sRGBToLinearRGBScalar(lrgb.b);
    return vec4(r, g, b, lrgb.a);
}
