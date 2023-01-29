#version 460
#ifdef ENABLE_ARB_INCLUDE_EXT
    #extension GL_ARB_shading_language_include : require
#else
    // required by glslangValidator
    #extension GL_GOOGLE_include_directive : require
#endif
#pragma fragment

#include "common/baseVideoFrag.glsl"

layout(constant_id = 9) const float AlphaValue = 0.3f;


const vec3 key_color = vec3(0.47f, 0.01f, 0.47f);
const vec3 key_color2 = vec3(0.53f, 0.10f, 0.53f);

layout(location = 0) out vec4 FragColor;

void main()
{
    vec4 color = SampleVideoTexture();
    color.a = (all(greaterThan(color.rgb, key_color)) && all(lessThan(color.rgb, key_color2))) ? 0.01f : 1.0f; 
    FragColor = color;
}