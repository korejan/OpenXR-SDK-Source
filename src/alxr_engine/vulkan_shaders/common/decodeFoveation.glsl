precision highp float;

layout(constant_id = 0) const float EyeSizeRatioX = 0.978516;
layout(constant_id = 1) const float EyeSizeRatioY = 0.978516;
layout(constant_id = 2) const float CenterSizeX   = 0.399123;
layout(constant_id = 3) const float CenterSizeY   = 0.399123;
layout(constant_id = 4) const float CenterShiftX  = 0.401460;
layout(constant_id = 5) const float CenterShiftY  = 0.401460;
layout(constant_id = 6) const float EdgeRatioX    = 4.000000;
layout(constant_id = 7) const float EdgeRatioY    = 4.000000;

const vec2 EyeSizeRatio = vec2(EyeSizeRatioX, EyeSizeRatioY);
const vec2 CenterSize   = vec2(CenterSizeX,   CenterSizeY);
const vec2 CenterShift  = vec2(CenterShiftX,  CenterShiftY);
const vec2 EdgeRatio    = vec2(EdgeRatioX,    EdgeRatioY);

vec2 TextureToEyeUV(const vec2 textureUV, const float isRightEye) {
    // flip distortion horizontally for right eye
    // left: x * 2; right: (1 - x) * 2
    return vec2((textureUV.x + isRightEye * (1. - 2. * textureUV.x)) * 2., textureUV.y);
}

vec2 EyeToTextureUV(const vec2 eyeUV, const float isRightEye) {
    // left: x / 2; right 1 - (x / 2)
    return vec2(eyeUV.x * 0.5 + isRightEye * (1. - eyeUV.x), eyeUV.y);
}

vec2 ComputeRightEdge
(
    const vec2 alignedUV, const vec2 er1, const vec2 hiBoundC,
    const vec2 c0, const vec2 c1, const vec2 c2
)
{
    const vec2 d1 = 1. - hiBoundC;
    const vec2 d2 = EdgeRatio * d1;
    const vec2 d3 = c2 * EdgeRatio - c2;
    const vec2 d4 = c2 - EdgeRatio * c1 - 2. * EdgeRatio * c2 + c2 * d2 + EdgeRatio;
    const vec2 d5 = d4 / d2;
    const vec2 d6 = d5 * d5 - 4. * (d3 * (c1 - hiBoundC + hiBoundC * c2) / (d2 * d1) - alignedUV * d3 / d2);
    return (sqrt(abs(d6)) - d5) / (2. * c2 * er1) * d2;
}

vec2 ComputeLeftEdge
(
    const vec2 alignedUV, const vec2 loBoundC,
    const vec2 c0, const vec2 c1, const vec2 c2
)
{
    const vec2 d1 = c1 + c2 * loBoundC;
    const vec2 d2 = d1 / loBoundC;
    const vec2 d3 = 1. - EdgeRatio;
    const vec2 d4 = EdgeRatio * loBoundC;
    const vec2 d5 = c2 * d3;
    const vec2 d6 = d2 * d2 + 4. * d5 / d4 * alignedUV;
    return (sqrt(abs(d6)) - d2) / (2. * d5) * d4;
}

vec2 DecodeFoveationUV(const vec2 uv, const float isRightEye) {
    const vec2 alignedUV = TextureToEyeUV(uv, isRightEye);

    const vec2 er1 = EdgeRatio - 1.;

    const vec2 c0 = (1. - CenterSize) * 0.5;
    const vec2 loBound = c0 * (CenterShift + 1.);

    const vec2 c1 = er1 * loBound / EdgeRatio;
    const vec2 c2 = er1 * CenterSize + 1.;

    const vec2 hiBoundA = c0 * (CenterShift - 1.);
    const vec2 hiBound = hiBoundA + 1.;

    const vec2 underBound = vec2(lessThan(alignedUV, loBound));
    const vec2 overBound = vec2(greaterThan(alignedUV, hiBound));
    const vec2 inBound = vec2(loBound.x < alignedUV.x&& alignedUV.x < hiBound.x,
        loBound.y < alignedUV.y&& alignedUV.y < hiBound.y);

    const vec2 c2Inv = 1.0 / c2;
    const vec2 center = (alignedUV - c1) * EdgeRatio * c2Inv;

    const vec2 loBoundC = loBound * c2Inv;
    const vec2 leftEdge = ComputeLeftEdge(alignedUV, loBoundC, c0, c1, c2);

    const vec2 hiBoundC = hiBoundA * c2Inv + 1.;
    const vec2 rightEdge = ComputeRightEdge(alignedUV, er1, hiBoundC, c0, c1, c2);

    const vec2 uncompressedUV = underBound * leftEdge + inBound * center + overBound * rightEdge;

    return EyeToTextureUV(uncompressedUV * EyeSizeRatio, isRightEye);
}
