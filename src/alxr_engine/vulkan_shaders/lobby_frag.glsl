#version 460
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable
#pragma fragment

layout (location = 0) in vec4 oColor;
layout (location = 0) out vec4 FragColor;

layout(early_fragment_tests) in;

void main()
{
    FragColor = oColor;
}
