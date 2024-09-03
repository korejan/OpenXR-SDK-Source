#version 460
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable
#pragma vertex

layout(std140, push_constant) uniform buf
{
    mat4 mvp;
} ubuf;

layout(location = 0) in vec2 Position;

out gl_PerVertex
{
    vec4 gl_Position;
};

void main()
{
    gl_Position = ubuf.mvp * vec4(Position, -1.0, 1.0);
}
