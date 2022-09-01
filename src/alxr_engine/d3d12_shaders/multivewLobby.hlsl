struct PSVertex {
    float4 Pos : SV_POSITION;
    float3 Color : COLOR0;
};
struct Vertex {
    float3 Pos : POSITION;
    float3 Color : COLOR0;
};

cbuffer ModelConstantBuffer : register(b0) {
    float4x4 Model;
};
cbuffer ViewProjectionConstantBuffer : register(b1) {
    float4x4 ViewProjection[2];
};

PSVertex MainVS(Vertex input, in uint ViewId : SV_ViewID) {
    PSVertex output;
    output.Pos = mul(mul(float4(input.Pos, 1), Model), ViewProjection[ViewId]);
    output.Color = input.Color;
    return output;
}

float4 MainPS(PSVertex input) : SV_TARGET {
    return float4(input.Color, 1);
}
