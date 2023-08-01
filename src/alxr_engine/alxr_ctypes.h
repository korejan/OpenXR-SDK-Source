#pragma once
#ifndef ALXR_ENGINE_CTYPES_H
#define ALXR_ENGINE_CTYPES_H

#ifdef __cplusplus
extern "C" {;
#endif

#include <stdint.h>

#ifndef ALXR_CLIENT
#define ALXR_CLIENT
#endif
#include "bindings.h"

enum ALXRGraphicsApi : uint32_t
{
    Auto,
    Vulkan2,
    Vulkan,
    D3D12,
    D3D11,
    OpenGLES,
    OpenGL,
    ApiCount = OpenGL
};

enum class ALXRDecoderType : uint32_t
{
    D311VA,
    NVDEC,
    CUVID,
    VAAPI,
    CPU
};

enum class ALXRFacialExpressionType : uint8_t {
    None = 0, // Not Support or Disabled
    FB,
    HTC,
    Pico,
    Auto,
    TypeCount
};

enum class ALXREyeTrackingType : uint8_t {
    None = 0, // Not Support or Disabled
    FBEyeTrackingSocial,
    ExtEyeGazeInteraction,
    Auto,
    TypeCount
};

enum class ALXRTrackingSpace : uint32_t
{
    LocalRefSpace,
    StageRefSpace,
    ViewRefSpace
};

enum class ALXRCodecType : uint32_t
{
    H264_CODEC,
    HEVC_CODEC
};

// replicates https://registry.khronos.org/OpenXR/specs/1.0/html/xrspec.html#XR_FB_color_space
enum class ALXRColorSpace : int32_t  {
    Unmanaged = 0,
    Rec2020 = 1,
    Rec709 = 2,
    RiftCV1 = 3,
    RiftS = 4,
    Quest = 5,
    P3 = 6,
    AdobeRgb = 7,
    Default = Quest,
    MaxEnum = 0x7fffffff
};

enum ALXRTrackingEnabledFlags : uint64_t {
    ALXR_TRACKING_ENABLED_HANDS = (1u << 0),
    ALXR_TRACKING_ENABLED_EYES  = (1u << 1),
    ALXR_TRACKING_ENABLED_FACE  = (1u << 2),    
    ALXR_TRACKING_ENABLED_ALL = ALXR_TRACKING_ENABLED_HANDS | ALXR_TRACKING_ENABLED_EYES | ALXR_TRACKING_ENABLED_FACE
};

struct ALXRSystemProperties
{
    char         systemName[256];
    float        currentRefreshRate;
    const float* refreshRates;
    uint32_t refreshRatesCount;
    uint32_t recommendedEyeWidth;
    uint32_t recommendedEyeHeight;
    uint64_t enabledTrackingSystemsFlags;
};

struct ALXREyeInfo
{
    ALXREyeFov eyeFov[2];
    float ipd;
};

struct ALXRVersion {
    uint32_t major;
    uint32_t minor;
    uint32_t patch;
};

typedef struct ALXRRustCtx
{
    void (*inputSend)(const TrackingInfo* data);
    void (*viewsConfigSend)(const ALXREyeInfo* eyeInfo);
    uint64_t (*pathStringToHash)(const char* path);
    void (*timeSyncSend)(const TimeSync* data);
    void (*videoErrorReportSend)();
    void (*batterySend)(uint64_t device_path, float gauge_value, bool is_plugged);
    void (*setWaitingNextIDR)(const bool);
    void (*requestIDR)();

    ALXRVersion     firmwareVersion;
    ALXRGraphicsApi graphicsApi;
    ALXRDecoderType decoderType;
    ALXRColorSpace  displayColorSpace;

    ALXRFacialExpressionType facialTracking;
    ALXREyeTrackingType eyeTracking;

    uint16_t trackingServerPortNo;

    bool verbose;
    bool disableLinearizeSrgb;
    bool noSuggestedBindings;
    bool noServerFramerateLock;
    bool noFrameSkip;
    bool disableLocalDimming;
    bool headlessSession;
    bool noFTServer;
    bool noPassthrough;
    bool noHandTracking;

#ifdef XR_USE_PLATFORM_ANDROID
    void* applicationVM;
    void* applicationActivity;
#endif
} ALXRClientCtx;

struct ALXRGuardianData {
    float areaWidth;
    float areaHeight;
    bool shouldSync;
};

struct ALXRRenderConfig
{
    uint32_t eyeWidth;
    uint32_t eyeHeight;
    float refreshRate;
    float foveationCenterSizeX;
    float foveationCenterSizeY;
    float foveationCenterShiftX;
    float foveationCenterShiftY;
    float foveationEdgeRatioX;
    float foveationEdgeRatioY;
    bool enableFoveation;
};

struct ALXRDecoderConfig
{
    ALXRCodecType codecType;
    uint32_t      cpuThreadCount; // only used for software decoding.
    bool          enableFEC;
    bool          realtimePriority;
};

struct ALXRStreamConfig {
    ALXRTrackingSpace   trackingSpaceType;
    ALXRRenderConfig    renderConfig;
    ALXRDecoderConfig   decoderConfig;
};

enum ALXRLogOptions : uint32_t {
    ALXR_LOG_OPTION_NONE = 0,
    ALXR_LOG_OPTION_TIMESTAMP = (1u << 0),
    ALXR_LOG_OPTION_LEVEL_TAG = (1u << 1)
};
enum class ALXRLogLevel : uint32_t { Verbose, Info, Warning, Error };
typedef void (*ALXRLogOutputFn)(ALXRLogLevel level, const char* output, uint32_t len);

#ifdef __cplusplus
}
#endif
#endif
