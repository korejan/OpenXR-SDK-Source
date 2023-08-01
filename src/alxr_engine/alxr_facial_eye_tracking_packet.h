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

inline constexpr const std::size_t MaxEyeCount = 2;
inline constexpr const std::size_t MaxExpressionCount = 63;
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

enum ALXRSpaceLocationFlags : uint64_t {
    ALXR_SPACE_LOCATION_ORIENTATION_VALID_BIT   = 0x00000001,
    ALXR_SPACE_LOCATION_POSITION_VALID_BIT      = 0x00000002,
    ALXR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT = 0x00000004,
    ALXR_SPACE_LOCATION_POSITION_TRACKED_BIT    = 0x00000008
};

enum ALXRSpaceVelocityFlags : uint64_t {
    ALXR_SPACE_VELOCITY_LINEAR_VALID_BIT  = 0x00000001,
    ALXR_SPACE_VELOCITY_ANGULAR_VALID_BIT = 0x00000002
};

typedef struct ALXRHandJointLocation {
    ALXRSpaceLocationFlags locationFlags;
    ALXRPosef              pose;
    float                  radius;
} ALXRHandJointLocation;

typedef struct ALXRHandJointVelocity {
    ALXRSpaceVelocityFlags  velocityFlags;
    ALXRVector3f            linearVelocity;
    ALXRVector3f            angularVelocity;
} ALXRHandJointVelocity;

inline constexpr const std::size_t MaxHandJointCount = 26;

typedef struct ALXRHandJointLocations {
    ALXRHandJointLocation jointLocations[MaxHandJointCount];
    ALXRHandJointVelocity jointVelocities[MaxHandJointCount];
    bool                  isActive;
} ALXRHandJointLocations;

inline constexpr const std::size_t MaxHandCount = 2;

typedef struct ALXRHandTracking {
    ALXRHandJointLocations hands[MaxHandCount];
} ALXRHandTracking;

struct ALXRProcessFrameResult {
    ALXRHandTracking    handTracking;
    ALXRFacialEyePacket facialEyeTracking;
    bool                exitRenderLoop;
    bool                requestRestart;
};
DLLEXPORT void alxr_process_frame2(ALXRProcessFrameResult* result);

#ifdef __cplusplus
}
#endif
#endif
