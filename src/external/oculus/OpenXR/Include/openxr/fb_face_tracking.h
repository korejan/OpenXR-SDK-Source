// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

/************************************************************************************
Filename    :   fb_face_tracking.h
Content     :   Face tracking APIs.
Language    :   C99
*************************************************************************************/

#pragma once

#include <openxr/openxr.h>
#include <openxr/openxr_extension_helpers.h>

/*
  202 XR_FB_face_tracking
*/

#if defined(__cplusplus)
extern "C" {
#endif

#ifndef XR_FB_face_tracking
#define XR_FB_face_tracking 1
XR_DEFINE_HANDLE(XrFaceTrackerFB)

#define XR_FB_face_tracking_SPEC_VERSION 1
#define XR_FB_FACE_TRACKING_EXTENSION_NAME "XR_FB_face_tracking"
XR_STRUCT_ENUM(XR_TYPE_FACE_TRACKER_CREATE_INFO_FB, 1000201005);
XR_STRUCT_ENUM(XR_TYPE_FACE_EXPRESSION_INFO_FB, 1000201002);
XR_STRUCT_ENUM(XR_TYPE_FACE_EXPRESSION_WEIGHTS_FB, 1000201006);

XR_STRUCT_ENUM(XR_TYPE_SYSTEM_FACE_TRACKING_PROPERTIES_FB, 1000201004);

typedef enum XrFaceExpressionFB {
    XR_FACE_EXPRESSION_BROW_LOWERER_L_FB = 0,
    XR_FACE_EXPRESSION_BROW_LOWERER_R_FB = 1,

    XR_FACE_EXPRESSION_CHEEK_PUFF_L_FB = 2,
    XR_FACE_EXPRESSION_CHEEK_PUFF_R_FB = 3,
    XR_FACE_EXPRESSION_CHEEK_RAISER_L_FB = 4,
    XR_FACE_EXPRESSION_CHEEK_RAISER_R_FB = 5,
    XR_FACE_EXPRESSION_CHEEK_SUCK_L_FB = 6,
    XR_FACE_EXPRESSION_CHEEK_SUCK_R_FB = 7,

    XR_FACE_EXPRESSION_CHIN_RAISER_B_FB = 8,
    XR_FACE_EXPRESSION_CHIN_RAISER_T_FB = 9,

    XR_FACE_EXPRESSION_DIMPLER_L_FB = 10,
    XR_FACE_EXPRESSION_DIMPLER_R_FB = 11,

    XR_FACE_EXPRESSION_EYES_CLOSED_L_FB = 12,
    XR_FACE_EXPRESSION_EYES_CLOSED_R_FB = 13,
    XR_FACE_EXPRESSION_EYES_LOOK_DOWN_L_FB = 14,
    XR_FACE_EXPRESSION_EYES_LOOK_DOWN_R_FB = 15,
    XR_FACE_EXPRESSION_EYES_LOOK_LEFT_L_FB = 16,
    XR_FACE_EXPRESSION_EYES_LOOK_LEFT_R_FB = 17,
    XR_FACE_EXPRESSION_EYES_LOOK_RIGHT_L_FB = 18,
    XR_FACE_EXPRESSION_EYES_LOOK_RIGHT_R_FB = 19,
    XR_FACE_EXPRESSION_EYES_LOOK_UP_L_FB = 20,
    XR_FACE_EXPRESSION_EYES_LOOK_UP_R_FB = 21,

    XR_FACE_EXPRESSION_INNER_BROW_RAISER_L_FB = 22,
    XR_FACE_EXPRESSION_INNER_BROW_RAISER_R_FB = 23,

    XR_FACE_EXPRESSION_JAW_DROP_FB = 24,
    XR_FACE_EXPRESSION_JAW_SIDEWAYS_LEFT_FB = 25,
    XR_FACE_EXPRESSION_JAW_SIDEWAYS_RIGHT_FB = 26,
    XR_FACE_EXPRESSION_JAW_THRUST_FB = 27,

    XR_FACE_EXPRESSION_LID_TIGHTENER_L_FB = 28,
    XR_FACE_EXPRESSION_LID_TIGHTENER_R_FB = 29,

    XR_FACE_EXPRESSION_LIP_CORNER_DEPRESSOR_L_FB = 30,
    XR_FACE_EXPRESSION_LIP_CORNER_DEPRESSOR_R_FB = 31,
    XR_FACE_EXPRESSION_LIP_CORNER_PULLER_L_FB = 32,
    XR_FACE_EXPRESSION_LIP_CORNER_PULLER_R_FB = 33,
    XR_FACE_EXPRESSION_LIP_FUNNELER_LB_FB = 34,
    XR_FACE_EXPRESSION_LIP_FUNNELER_LT_FB = 35,
    XR_FACE_EXPRESSION_LIP_FUNNELER_RB_FB = 36,
    XR_FACE_EXPRESSION_LIP_FUNNELER_RT_FB = 37,
    XR_FACE_EXPRESSION_LIP_PRESSOR_L_FB = 38,
    XR_FACE_EXPRESSION_LIP_PRESSOR_R_FB = 39,
    XR_FACE_EXPRESSION_LIP_PUCKER_L_FB = 40,
    XR_FACE_EXPRESSION_LIP_PUCKER_R_FB = 41,
    XR_FACE_EXPRESSION_LIP_STRETCHER_L_FB = 42,
    XR_FACE_EXPRESSION_LIP_STRETCHER_R_FB = 43,
    XR_FACE_EXPRESSION_LIP_SUCK_LB_FB = 44,
    XR_FACE_EXPRESSION_LIP_SUCK_LT_FB = 45,
    XR_FACE_EXPRESSION_LIP_SUCK_RB_FB = 46,
    XR_FACE_EXPRESSION_LIP_SUCK_RT_FB = 47,
    XR_FACE_EXPRESSION_LIP_TIGHTENER_L_FB = 48,
    XR_FACE_EXPRESSION_LIP_TIGHTENER_R_FB = 49,
    XR_FACE_EXPRESSION_LIPS_TOWARD_FB = 50,
    XR_FACE_EXPRESSION_LOWER_LIP_DEPRESSOR_L_FB = 51,
    XR_FACE_EXPRESSION_LOWER_LIP_DEPRESSOR_R_FB = 52,

    XR_FACE_EXPRESSION_MOUTH_LEFT_FB = 53,
    XR_FACE_EXPRESSION_MOUTH_RIGHT_FB = 54,

    XR_FACE_EXPRESSION_NOSE_WRINKLER_L_FB = 55,
    XR_FACE_EXPRESSION_NOSE_WRINKLER_R_FB = 56,

    XR_FACE_EXPRESSION_OUTER_BROW_RAISER_L_FB = 57,
    XR_FACE_EXPRESSION_OUTER_BROW_RAISER_R_FB = 58,

    XR_FACE_EXPRESSION_UPPER_LID_RAISER_L_FB = 59,
    XR_FACE_EXPRESSION_UPPER_LID_RAISER_R_FB = 60,
    XR_FACE_EXPRESSION_UPPER_LIP_RAISER_L_FB = 61,
    XR_FACE_EXPRESSION_UPPER_LIP_RAISER_R_FB = 62,

    XR_FACE_EXPRESSION_COUNT_FB = 63,
    XR_FACE_EXPRESSION_MAX_ENUM_FB = 0x7FFFFFFF
} XrFaceExpressionFB;

typedef enum XrFaceExpressionSetFB {
    XR_FACE_EXPRESSION_SET_DEFAULT_FB = 0,
    XR_FACE_EXPRESSION_SET_MAX_ENUM_FB = 0x7FFFFFFF
} XrFaceExpressionSetFB;
#define XR_FACE_EXPRESSSION_SET_DEFAULT_FB XR_FACE_EXPRESSION_SET_DEFAULT_FB

typedef enum XrFaceConfidenceFB {
    XR_FACE_CONFIDENCE_LOWER_FACE_FB = 0,
    XR_FACE_CONFIDENCE_UPPER_FACE_FB = 1,

    XR_FACE_CONFIDENCE_COUNT_FB = 2,
    XR_FACE_CONFIDENCE_MAX_ENUM_FB = 0x7FFFFFFF
} XrFaceConfidenceFB;

typedef struct XrSystemFaceTrackingPropertiesFB {
    XrStructureType type;
    void* XR_MAY_ALIAS next;
    XrBool32 supportsFaceTracking;
} XrSystemFaceTrackingPropertiesFB;

typedef struct XrFaceTrackerCreateInfoFB {
    XrStructureType type;
    const void* XR_MAY_ALIAS next;
    XrFaceExpressionSetFB faceExpressionSet;
} XrFaceTrackerCreateInfoFB;

typedef struct XrFaceExpressionInfoFB {
    XrStructureType type;
    const void* XR_MAY_ALIAS next;
    XrTime time;
} XrFaceExpressionInfoFB;

typedef struct XrFaceExpressionStatusFB {
    XrBool32 isValid;
    XrBool32 isEyeFollowingBlendshapesValid;
} XrFaceExpressionStatusFB;

typedef struct XrFaceExpressionWeightsFB {
    XrStructureType type;
    void* XR_MAY_ALIAS next;
    uint32_t weightCount;
    float* weights;
    uint32_t confidenceCount;
    float* confidences;
    XrFaceExpressionStatusFB status;
    XrTime time;
} XrFaceExpressionWeightsFB;

typedef XrResult(XRAPI_PTR* PFN_xrCreateFaceTrackerFB)(
    XrSession session,
    const XrFaceTrackerCreateInfoFB* createInfo,
    XrFaceTrackerFB* faceTracker);
typedef XrResult(XRAPI_PTR* PFN_xrDestroyFaceTrackerFB)(XrFaceTrackerFB faceTracker);
typedef XrResult(XRAPI_PTR* PFN_xrGetFaceExpressionWeightsFB)(
    XrFaceTrackerFB faceTracker,
    const XrFaceExpressionInfoFB* expressionInfo,
    XrFaceExpressionWeightsFB* expressionWeights);

#endif // XR_FB_face_tracking


#ifdef __cplusplus
}
#endif
