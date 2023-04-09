#pragma once
#ifndef ALXR_FACIAL_TRACKING_PACKET_H
#define ALXR_FACIAL_TRACKING_PACKET_H
#include "alxr_ctypes.h"

#ifdef XR_USE_PLATFORM_WIN32
#ifdef ENGINE_DLL_EXPORTS
    /*Enabled as "export" while compiling the dll project*/
    #define DLLEXPORT __declspec(dllexport)  
#else
    /*Enabled as "import" in the Client side for using already created dll file*/
    #define DLLEXPORT __declspec(dllimport)  
#endif
#else
#define DLLEXPORT
#endif

#ifdef __cplusplus
extern "C" {;
#endif

constexpr static const std::size_t MaxEyeCount = 2;
constexpr static const std::size_t MaxExpressionCount = 63;
static_assert((XR_FACIAL_EXPRESSION_LIP_COUNT_HTC + XR_FACIAL_EXPRESSION_EYE_COUNT_HTC) <= MaxExpressionCount);
#ifdef XR_USE_OXR_OCULUS
    static_assert(XR_FACE_EXPRESSION_COUNT_FB <= MaxExpressionCount);
#endif 

#pragma pack(push, 1)
struct ALXRFacialEyePacket {
    ALXRFacialExpressionType expressionType;
    ALXREyeTrackingType      eyeTrackerType;
    std::uint8_t             isEyeFollowingBlendshapesValid;
    std::uint8_t             isEyeGazePoseValid[MaxEyeCount];
    float                    expressionWeights[MaxExpressionCount];
    XrPosef                  eyeGazePoses[MaxEyeCount];
};
#pragma pack(pop)

struct ALXRProcessFrameResult {
    ALXRFacialEyePacket* newPacket;
    bool                 exitRenderLoop;
    bool                 requestRestart;
};
DLLEXPORT void alxr_process_frame2(ALXRProcessFrameResult* result);

#ifdef __cplusplus
}
#endif
#endif
