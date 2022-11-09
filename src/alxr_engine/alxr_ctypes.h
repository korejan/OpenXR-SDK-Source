#pragma once
#ifndef ALXR_ENGINE_CTYPES_H
#define ALXR_ENGINE_CTYPES_H

#ifndef ALXR_CLIENT
#define ALXR_CLIENT
#endif
#include "bindings.h"

#ifdef __cplusplus
extern "C" {;
#endif

enum ALXRGraphicsApi
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

enum ALXRDecoderType
{
    D311VA,
    NVDEC,
    CUVID,
    VAAPI,
    CPU
};

enum ALXRTrackingSpace
{
    LocalRefSpace,
    StageRefSpace,
    ViewRefSpace
};

enum ALXRCodecType
{
    H264_CODEC,
    HEVC_CODEC
};

// replicates https://registry.khronos.org/OpenXR/specs/1.0/html/xrspec.html#XR_FB_color_space
enum ALXRColorSpace {
    Unmanaged = 0,
    Rec2020 = 1,
    Rec709 = 2,
    RiftCV1 = 3,
    RiftS = 4,
    Quest = 5,
    P3 = 6,
    AdobeRgb = 7,
    MaxEnum = 0x7fffffff
};

struct ALXRSystemProperties
{
    char         systemName[256];
    float        currentRefreshRate;
    const float* refreshRates;
    unsigned int refreshRatesCount;
    unsigned int recommendedEyeWidth;
    unsigned int recommendedEyeHeight;
};

struct ALXREyeInfo
{
    EyeFov eyeFov[2];
    float ipd;
};

struct ALXRRustCtx
{
    void (*inputSend)(const TrackingInfo* data);
    void (*viewsConfigSend)(const ALXREyeInfo* eyeInfo);
    unsigned long long (*pathStringToHash)(const char* path);
    void (*timeSyncSend)(const TimeSync* data);
    void (*videoErrorReportSend)();
    void (*batterySend)(unsigned long long device_path, float gauge_value, bool is_plugged);
    void (*setWaitingNextIDR)(const bool);
    void (*requestIDR)();

    ALXRGraphicsApi graphicsApi;
    ALXRDecoderType decoderType;
    ALXRColorSpace  displayColorSpace;

    bool verbose;
    bool disableLinearizeSrgb;
    bool noSuggestedBindings;
    bool noServerFramerateLock;
    bool noFrameSkip;
    bool disableLocalDimming;
#ifdef XR_USE_PLATFORM_ANDROID
    void* applicationVM;
    void* applicationActivity;
#endif
};

struct ALXRGuardianData {
    bool shouldSync;
    float areaWidth;
    float areaHeight;
};

struct ALXRRenderConfig
{
    unsigned int eyeWidth;
    unsigned int eyeHeight;
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
    bool          enableFEC;
    bool          realtimePriority;
    unsigned int  cpuThreadCount; // only used for software decoding.
};

struct ALXRStreamConfig {
    ALXRTrackingSpace   trackingSpaceType;
    ALXRRenderConfig    renderConfig;
    ALXRDecoderConfig   decoderConfig;
};

#ifdef __cplusplus
}
#endif
#endif
