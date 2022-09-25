cbuffer FoveationParams : register(b2) {
    float2 EyeSizeRatio;
    float2 CenterSize;
    float2 CenterShift;
    float2 EdgeRatio;
};

float2 TextureToEyeUV(const float2 textureUV, const float isRightEye) {
    // flip distortion horizontally for right eye
    // left: x * 2; right: (1 - x) * 2
    return float2((textureUV.x + isRightEye * (1. - 2. * textureUV.x)) * 2., textureUV.y);
}

float2 EyeToTextureUV(const float2 eyeUV, const float isRightEye) {
    // left: x / 2; right 1 - (x / 2)
    return float2(eyeUV.x * 0.5 + isRightEye * (1. - eyeUV.x), eyeUV.y);
}

float2 ComputeRightEdge
(
    const float2 alignedUV, const float2 er1, const float2 hiBoundC,
    const float2 c0, const float2 c1, const float2 c2
)
{
    const float2 d1 = 1. - hiBoundC;
    const float2 d2 = EdgeRatio * d1;
    const float2 d3 = c2 * EdgeRatio - c2;
    const float2 d4 = c2 - EdgeRatio * c1 - 2. * EdgeRatio * c2 + c2 * d2 + EdgeRatio;
    const float2 d5 = d4 / d2;
    const float2 d6 = d5 * d5 - 4. * (d3 * (c1 - hiBoundC + hiBoundC * c2) / (d2 * d1) - alignedUV * d3 / d2);
    return (sqrt(abs(d6)) - d5) / (2. * c2 * er1) * d2;
}

float2 ComputeLeftEdge
(
    const float2 alignedUV, const float2 loBoundC,
    const float2 c0, const float2 c1, const float2 c2
)
{
    const float2 d1 = c1 + c2 * loBoundC;
    const float2 d2 = d1 / loBoundC;
    const float2 d3 = 1. - EdgeRatio;
    const float2 d4 = EdgeRatio * loBoundC;
    const float2 d5 = c2 * d3;
    const float2 d6 = d2 * d2 + 4. * d5 / d4 * alignedUV;
    return (sqrt(abs(d6)) - d2) / (2. * d5) * d4;
}

float2 DecodeFoveationUV(const float2 uv, const float isRightEye) {
    const float2 alignedUV = TextureToEyeUV(uv, isRightEye);

    const float2 er1 = EdgeRatio - 1.;

    const float2 c0 = (1. - CenterSize) * 0.5;
    const float2 loBound = c0 * (CenterShift + 1.);

    const float2 c1 = er1 * loBound / EdgeRatio;
    const float2 c2 = er1 * CenterSize + 1.;

    const float2 hiBoundA = c0 * (CenterShift - 1.);
    const float2 hiBound = hiBoundA + 1.;

    const float2 underBound = float2(alignedUV < loBound);
    const float2 overBound = float2(alignedUV > hiBound);
    const float2 inBound = float2(loBound.x < alignedUV.x&& alignedUV.x < hiBound.x,
                                  loBound.y < alignedUV.y&& alignedUV.y < hiBound.y);

    const float2 c2Inv = 1.0 / c2;
    const float2 center = (alignedUV - c1) * EdgeRatio * c2Inv;

    const float2 loBoundC = loBound * c2Inv;
    const float2 leftEdge = ComputeLeftEdge(alignedUV, loBoundC, c0, c1, c2);

    const float2 hiBoundC = hiBoundA * c2Inv + 1.;
    const float2 rightEdge = ComputeRightEdge(alignedUV, er1, hiBoundC, c0, c1, c2);

    const float2 uncompressedUV = underBound * leftEdge + inBound * center + overBound * rightEdge;

    return EyeToTextureUV(uncompressedUV * EyeSizeRatio, isRightEye);
}
