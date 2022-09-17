#version 460
#ifdef ENABLE_ARB_INCLUDE_EXT
    #extension GL_ARB_shading_language_include : require
#else
    // required by glslangValidator
    #extension GL_GOOGLE_include_directive : require
#endif

#include "common/baseVideoFrag.glsl"

layout(location = 0) out vec4 FragColor;

#pragma fragment

void main()
{
    FragColor = vec4(SampleVideoTexture().rgb, 0.6f);
}
