#version 460
#ifdef ENABLE_ARB_INCLUDE_EXT
    #extension GL_ARB_shading_language_include : require
#else
    // required by glslangValidator
    #extension GL_GOOGLE_include_directive : require
#endif
#pragma fragment

#include "common/baseVideoFrag.glsl"

layout(constant_id = 9) const float AlphaValue = 0.6f;

layout(location = 0) out vec4 FragColor;

void main()
{
    FragColor = vec4(SampleVideoTexture().rgb, AlphaValue);
}
