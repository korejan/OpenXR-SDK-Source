#version 460
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable
#ifdef ENABLE_MULTIVEW_EXT
    #extension GL_EXT_multiview : enable
#endif

#pragma vertex

layout (std140, push_constant) uniform buf
{
#ifdef ENABLE_MULTIVEW_EXT
    mat4 mvp[2];
#else
    mat4 mvp;
    uint ViewID;
#endif
} ubuf;

layout (location = 0) in vec3 Position;
layout (location = 1) in vec2 UV;
            
layout (location = 0) out vec2 oUV;            
out gl_PerVertex
{
    vec4 gl_Position;
};

void main()
{
    vec2 ouv = UV;

    const bool isRightView =
#ifdef ENABLE_MULTIVEW_EXT
        gl_ViewIndex > 0;
#else
        ubuf.ViewID > 0;
#endif
    if (isRightView) {
        ouv.x += 0.5f;
    }
    oUV = ouv;

    gl_Position =
#ifdef ENABLE_MULTIVEW_EXT
        ubuf.mvp[gl_ViewIndex] * vec4(Position, 1.0);
#else
        ubuf.mvp * vec4(Position, 1.0);
#endif
    gl_Position.y = -gl_Position.y;
}
