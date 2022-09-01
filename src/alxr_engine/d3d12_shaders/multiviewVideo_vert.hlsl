struct PSVertex {
    float4 Pos : SV_POSITION;
    float2 uv : TEXCOORD;
};
struct Vertex {
    float3 Pos : POSITION;
    float2 uv : TEXCOORD;
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
    output.uv = input.uv;
    if (ViewId > 0) {
        output.uv.x += 0.5f;
    }
    return output;
}
