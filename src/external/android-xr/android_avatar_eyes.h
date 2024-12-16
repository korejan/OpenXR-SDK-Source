#ifndef ANDROID_AVATAR_EYES_H_
#define ANDROID_AVATAR_EYES_H_ 1

/*
** Copyright 2017-2024, The Khronos Group Inc.
**
** SPDX-License-Identifier: Apache-2.0 OR MIT
*/

/*
** This header is generated from the Khronos OpenXR XML API Registry.
**
*/


#ifdef __cplusplus
extern "C" {
#endif


#ifndef XR_ANDROID_avatar_eyes

// XR_ANDROID_avatar_eyes is a preprocessor guard. Do not pass it to API calls.
#define XR_ANDROID_avatar_eyes 1

#define XR_EYE_MAX_ANDROID 2

XR_DEFINE_HANDLE(XrEyeTrackerANDROID)
#define XR_ANDROID_avatar_eyes_SPEC_VERSION 1
#define XR_ANDROID_AVATAR_EYES_EXTENSION_NAME "XR_ANDROID_avatar_eyes"
// XrEyeTrackerANDROID
#define XR_OBJECT_TYPE_EYE_TRACKER_ANDROID ((XrObjectType) 1000456000U)
#define XR_TYPE_EYES_ANDROID              ((XrStructureType) 1000456000U)
#define XR_TYPE_EYE_TRACKER_CREATE_INFO_ANDROID ((XrStructureType) 1000456001U)
#define XR_TYPE_EYES_GET_INFO_ANDROID     ((XrStructureType) 1000456002U)
#define XR_TYPE_SYSTEM_AVATAR_EYES_PROPERTIES_ANDROID ((XrStructureType) 1000456003U)

typedef enum XrEyeTrackingModeANDROID {
    XR_EYE_TRACKING_MODE_NOT_TRACKING_ANDROID = 0,
    XR_EYE_TRACKING_MODE_RIGHT_ANDROID = 1,
    XR_EYE_TRACKING_MODE_LEFT_ANDROID = 2,
    XR_EYE_TRACKING_MODE_BOTH_ANDROID = 3,
    XR_EYE_TRACKING_MODE_MAX_ENUM_ANDROID = 0x7FFFFFFF
} XrEyeTrackingModeANDROID;

typedef enum XrEyeStateANDROID {
    XR_EYE_STATE_INVALID_ANDROID = 0,
    XR_EYE_STATE_GAZING_ANDROID = 1,
    XR_EYE_STATE_SHUT_ANDROID = 2,
    XR_EYE_STATE_MAX_ENUM_ANDROID = 0x7FFFFFFF
} XrEyeStateANDROID;

typedef enum XrEyeIndexANDROID {
    XR_EYE_INDEX_LEFT_ANDROID = 0,
    XR_EYE_INDEX_RIGHT_ANDROID = 1,
    XR_EYE_INDEX_MAX_ENUM_ANDROID = 0x7FFFFFFF
} XrEyeIndexANDROID;
typedef struct XrSystemAvatarEyesPropertiesANDROID {
    XrStructureType       type;
    void* XR_MAY_ALIAS    next;
    XrBool32              supportsAvatarEyes;
} XrSystemAvatarEyesPropertiesANDROID;

typedef struct XrEyeTrackerCreateInfoANDROID {
    XrStructureType             type;
    const void* XR_MAY_ALIAS    next;
} XrEyeTrackerCreateInfoANDROID;

typedef struct XrEyesGetInfoANDROID {
    XrStructureType             type;
    const void* XR_MAY_ALIAS    next;
    XrSpace                     baseSpace;
    XrTime                      time;
} XrEyesGetInfoANDROID;

typedef struct XrEyeANDROID {
    XrEyeStateANDROID    eyeState;
    XrPosef              eyePose;
} XrEyeANDROID;

typedef struct XrEyesANDROID {
    XrStructureType             type;
    void* XR_MAY_ALIAS          next;
    XrEyeANDROID                eyes[XR_EYE_MAX_ANDROID];
    XrEyeTrackingModeANDROID    mode;
} XrEyesANDROID;

typedef XrResult (XRAPI_PTR *PFN_xrCreateEyeTrackerANDROID)(XrSession session, const XrEyeTrackerCreateInfoANDROID* createInfo, XrEyeTrackerANDROID* eyeTracker);
typedef XrResult (XRAPI_PTR *PFN_xrDestroyEyeTrackerANDROID)(XrEyeTrackerANDROID eyeTracker);
typedef XrResult (XRAPI_PTR *PFN_xrGetEyesInfoANDROID)(XrEyeTrackerANDROID eyeTracker, const XrEyesGetInfoANDROID* getInfo, XrEyesANDROID* infoOutput);

#ifndef XR_NO_PROTOTYPES
#ifdef XR_EXTENSION_PROTOTYPES
XRAPI_ATTR XrResult XRAPI_CALL xrCreateEyeTrackerANDROID(
    XrSession                                   session,
    const XrEyeTrackerCreateInfoANDROID*        createInfo,
    XrEyeTrackerANDROID*                        eyeTracker);

XRAPI_ATTR XrResult XRAPI_CALL xrDestroyEyeTrackerANDROID(
    XrEyeTrackerANDROID                         eyeTracker);

XRAPI_ATTR XrResult XRAPI_CALL xrGetEyesInfoANDROID(
    XrEyeTrackerANDROID                         eyeTracker,
    const XrEyesGetInfoANDROID*                 getInfo,
    XrEyesANDROID*                              infoOutput);
#endif /* XR_EXTENSION_PROTOTYPES */
#endif /* !XR_NO_PROTOTYPES */
#endif /* XR_ANDROID_avatar_eyes */

#ifdef __cplusplus
}
#endif

#endif
