// Copyright (c) 2017-2021, The Khronos Group Inc.
//
// SPDX-License-Identifier: Apache-2.0

#define ENGINE_DLL_EXPORTS

#include "pch.h"
#include "common.h"
#include "options.h"
#include "platformdata.h"
#include "platformplugin.h"
#include "graphicsplugin.h"
#include "openxr_program.h"
#include <common/xr_linear.h>
#include <array>
#include <cmath>
#include <cstddef>
#include <cassert>
#include <cstring>
#include <ctime>
#include <tuple>
#include <numeric>
#include <span>
#include <unordered_map>
#include <map>
#include <string_view>
#include <string>
#include <ratio>
#include <chrono>
#include <algorithm>
#include <mutex>
#include <shared_mutex>
#ifdef XR_USE_PLATFORM_ANDROID
    #include <unistd.h>
#endif

#include "xr_utils.h"
#include "concurrent_queue.h"
//#include "alxr_engine.h"
#include "alxr_ctypes.h"
#include "ALVR-common/packet_types.h"
#include "timing.h"
#include "latency_manager.h"
#include "interaction_profiles.h"
#include "interaction_manager.h"

#ifdef XR_USE_PLATFORM_ANDROID
#ifndef ALXR_ENGINE_DISABLE_QUIT_ACTION
#define ALXR_ENGINE_DISABLE_QUIT_ACTION
#endif
#endif

//#define ALXR_ENGINE_ENABLE_VIZ_SPACES

namespace {
#if !defined(XR_USE_PLATFORM_WIN32)
#define strcpy_s(dest, source) strncpy((dest), (source), sizeof(dest))
#endif

inline std::string GetXrVersionString(XrVersion ver) {
    return Fmt("%d.%d.%d", XR_VERSION_MAJOR(ver), XR_VERSION_MINOR(ver), XR_VERSION_PATCH(ver));
}

inline XrFormFactor GetXrFormFactor(const std::string& formFactorStr) {
    if (EqualsIgnoreCase(formFactorStr, "Hmd")) {
        return XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
    }
    if (EqualsIgnoreCase(formFactorStr, "Handheld")) {
        return XR_FORM_FACTOR_HANDHELD_DISPLAY;
    }
    throw std::invalid_argument(Fmt("Unknown form factor '%s'", formFactorStr.c_str()));
}

inline XrViewConfigurationType GetXrViewConfigurationType(const std::string& viewConfigurationStr) {
    if (EqualsIgnoreCase(viewConfigurationStr, "Mono")) {
        return XR_VIEW_CONFIGURATION_TYPE_PRIMARY_MONO;
    }
    if (EqualsIgnoreCase(viewConfigurationStr, "Stereo")) {
        return XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
    }
    throw std::invalid_argument(Fmt("Unknown view configuration '%s'", viewConfigurationStr.c_str()));
}

inline XrEnvironmentBlendMode GetXrEnvironmentBlendMode(const std::string& environmentBlendModeStr) {
    if (EqualsIgnoreCase(environmentBlendModeStr, "Opaque")) {
        return XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
    }
    if (EqualsIgnoreCase(environmentBlendModeStr, "Additive")) {
        return XR_ENVIRONMENT_BLEND_MODE_ADDITIVE;
    }
    if (EqualsIgnoreCase(environmentBlendModeStr, "AlphaBlend")) {
        return XR_ENVIRONMENT_BLEND_MODE_ALPHA_BLEND;
    }
    throw std::invalid_argument(Fmt("Unknown environment blend mode '%s'", environmentBlendModeStr.c_str()));
}

namespace Math {
template < typename RealT >
constexpr inline RealT ToDegrees(const RealT radians)
{
    return static_cast<RealT>(radians * (180.0 / 3.14159265358979323846));
}

inline void XrMatrix4x4f_CreateFromPose(XrMatrix4x4f& m, const XrPosef& pose)
{
    constexpr const XrVector3f scale{ 1.0f,1.0f,1.0f };
    XrMatrix4x4f_CreateTranslationRotationScale(&m, &pose.position, &pose.orientation, &scale);
}

inline XrMatrix4x4f XrMatrix4x4f_CreateFromPose(const XrPosef& pose)
{
    XrMatrix4x4f ret;
    XrMatrix4x4f_CreateFromPose(ret, pose);
    return ret;
}

namespace Pose {
constexpr inline XrPosef Identity() {
    XrPosef t{};
    t.orientation.w = 1;
    return t;
}

constexpr inline XrPosef Translation(const XrVector3f& translation) {
    XrPosef t = Identity();
    t.position = translation;
    return t;
}

inline XrPosef RotateCCWAboutYAxis(float radians, XrVector3f translation) {
    XrPosef t = Identity();
    t.orientation.x = 0.f;
    t.orientation.y = std::sin(radians * 0.5f);
    t.orientation.z = 0.f;
    t.orientation.w = std::cos(radians * 0.5f);
    t.position = translation;
    return t;
}

constexpr bool IsPoseValid(XrSpaceLocationFlags locationFlags) {
    constexpr XrSpaceLocationFlags PoseValidFlags = XR_SPACE_LOCATION_POSITION_VALID_BIT | XR_SPACE_LOCATION_ORIENTATION_VALID_BIT;
    return (locationFlags & PoseValidFlags) == PoseValidFlags;
}

constexpr bool IsPoseTracked(XrSpaceLocationFlags locationFlags) {
    constexpr XrSpaceLocationFlags PoseTrackedFlags =
        XR_SPACE_LOCATION_POSITION_TRACKED_BIT | XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT;
    return (locationFlags & PoseTrackedFlags) == PoseTrackedFlags;
}

constexpr bool IsPoseValid(const XrSpaceLocation& spaceLocation) {
    return IsPoseValid(spaceLocation.locationFlags);
}

constexpr bool IsPoseTracked(const XrSpaceLocation& spaceLocation) {
    return IsPoseTracked(spaceLocation.locationFlags);
}

constexpr bool IsPoseValid(const XrHandJointLocationEXT& jointLocation) {
    return IsPoseValid(jointLocation.locationFlags);
}

constexpr bool IsPoseTracked(const XrHandJointLocationEXT& jointLocation) {
    return IsPoseTracked(jointLocation.locationFlags);
}
}  // namespace Pose
}  // namespace Math

constexpr inline auto ToTrackingSpaceName(const ALXRTrackingSpace ts)
{
    switch (ts)
    {
    case ALXRTrackingSpace::LocalRefSpace: return "Local";
    case ALXRTrackingSpace::ViewRefSpace: return "View";
    }
    return "Stage";
}

/*constexpr*/ inline ALXRTrackingSpace ToTrackingSpace(const std::string_view& tsname)
{
    if (EqualsIgnoreCase(tsname, "Local"))
        return ALXRTrackingSpace::LocalRefSpace;
    if (EqualsIgnoreCase(tsname, "View"))
        return ALXRTrackingSpace::ViewRefSpace;
    return ALXRTrackingSpace::StageRefSpace;
}

constexpr inline ALXRTrackingSpace ToTrackingSpace(const XrReferenceSpaceType xrreftype)
{
    switch (xrreftype) {
    case XR_REFERENCE_SPACE_TYPE_VIEW: return ALXRTrackingSpace::ViewRefSpace;
    case XR_REFERENCE_SPACE_TYPE_LOCAL: return ALXRTrackingSpace::LocalRefSpace;
    }
    return ALXRTrackingSpace::StageRefSpace;
}

constexpr inline XrReferenceSpaceType ToXrReferenceSpaceType(const ALXRTrackingSpace xrreftype)
{
    switch (xrreftype) {
    case ALXRTrackingSpace::ViewRefSpace: return XR_REFERENCE_SPACE_TYPE_VIEW;
    case ALXRTrackingSpace::LocalRefSpace: return XR_REFERENCE_SPACE_TYPE_LOCAL;
    }
    return XR_REFERENCE_SPACE_TYPE_STAGE;
}

inline XrReferenceSpaceCreateInfo GetXrReferenceSpaceCreateInfo(const std::string_view& referenceSpaceTypeStr) {
    XrReferenceSpaceCreateInfo referenceSpaceCreateInfo{
        .type = XR_TYPE_REFERENCE_SPACE_CREATE_INFO,
        .next = nullptr,
        .poseInReferenceSpace = ALXR::IdentityPose
    };
    if (EqualsIgnoreCase(referenceSpaceTypeStr, "View")) {
        referenceSpaceCreateInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_VIEW;
    } else if (EqualsIgnoreCase(referenceSpaceTypeStr, "ViewFront")) {
        // Render head-locked 2m in front of device.
        referenceSpaceCreateInfo.poseInReferenceSpace = Math::Pose::Translation({0.f, 0.f, -2.f}),
        referenceSpaceCreateInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_VIEW;
    } else if (EqualsIgnoreCase(referenceSpaceTypeStr, "Local")) {
        referenceSpaceCreateInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL;
    } else if (EqualsIgnoreCase(referenceSpaceTypeStr, "Stage")) {
        referenceSpaceCreateInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_STAGE;
    } else if (EqualsIgnoreCase(referenceSpaceTypeStr, "StageLeft")) {
        referenceSpaceCreateInfo.poseInReferenceSpace = Math::Pose::RotateCCWAboutYAxis(0.f, {-2.f, 0.f, -2.f});
        referenceSpaceCreateInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_STAGE;
    } else if (EqualsIgnoreCase(referenceSpaceTypeStr, "StageRight")) {
        referenceSpaceCreateInfo.poseInReferenceSpace = Math::Pose::RotateCCWAboutYAxis(0.f, {2.f, 0.f, -2.f});
        referenceSpaceCreateInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_STAGE;
    } else if (EqualsIgnoreCase(referenceSpaceTypeStr, "StageLeftRotated")) {
        referenceSpaceCreateInfo.poseInReferenceSpace = Math::Pose::RotateCCWAboutYAxis(3.14f / 3.f, {-2.f, 0.5f, -2.f});
        referenceSpaceCreateInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_STAGE;
    } else if (EqualsIgnoreCase(referenceSpaceTypeStr, "StageRightRotated")) {
        referenceSpaceCreateInfo.poseInReferenceSpace = Math::Pose::RotateCCWAboutYAxis(-3.14f / 3.f, {2.f, 0.5f, -2.f});
        referenceSpaceCreateInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_STAGE;
    } else if (EqualsIgnoreCase(referenceSpaceTypeStr, "UboundedMSFT")) {
        referenceSpaceCreateInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_UNBOUNDED_MSFT;
    } else {
        throw std::invalid_argument(Fmt("Unknown reference space type '%s'", referenceSpaceTypeStr.data()));
    }
    return referenceSpaceCreateInfo;
}

inline XrReferenceSpaceCreateInfo GetXrReferenceSpaceCreateInfo(const ALXRTrackingSpace ts) {
    return GetXrReferenceSpaceCreateInfo(ToTrackingSpaceName(ts));
}

constexpr inline TrackingVector3 ToTrackingVector3(const XrVector3f& v)
{
    return { v.x, v.y, v.z };
}

constexpr inline TrackingQuat ToTrackingQuat(const XrQuaternionf& v)
{
    return { v.x, v.y, v.z, v.w };
}

constexpr inline const XrView IdentityView {
    .type = XR_TYPE_VIEW,
    .next = nullptr,
    .pose = ALXR::IdentityPose,
    .fov = { 0,0,0,0 }
};

constexpr inline XrHandJointEXT GetJointParent(const XrHandJointEXT h)
{
    switch (h)
    {
    case XR_HAND_JOINT_PALM_EXT: return XR_HAND_JOINT_PALM_EXT;
    case XR_HAND_JOINT_WRIST_EXT: return XR_HAND_JOINT_PALM_EXT;
    case XR_HAND_JOINT_THUMB_METACARPAL_EXT: return XR_HAND_JOINT_WRIST_EXT;
    case XR_HAND_JOINT_THUMB_PROXIMAL_EXT: return XR_HAND_JOINT_THUMB_METACARPAL_EXT;
    case XR_HAND_JOINT_THUMB_DISTAL_EXT: return XR_HAND_JOINT_THUMB_PROXIMAL_EXT;
    case XR_HAND_JOINT_THUMB_TIP_EXT: return XR_HAND_JOINT_THUMB_DISTAL_EXT;
    case XR_HAND_JOINT_INDEX_METACARPAL_EXT: return XR_HAND_JOINT_WRIST_EXT;
    case XR_HAND_JOINT_INDEX_PROXIMAL_EXT: return XR_HAND_JOINT_INDEX_METACARPAL_EXT;
    case XR_HAND_JOINT_INDEX_INTERMEDIATE_EXT: return XR_HAND_JOINT_INDEX_PROXIMAL_EXT;
    case XR_HAND_JOINT_INDEX_DISTAL_EXT: return XR_HAND_JOINT_INDEX_INTERMEDIATE_EXT;
    case XR_HAND_JOINT_INDEX_TIP_EXT: return XR_HAND_JOINT_INDEX_DISTAL_EXT;
    case XR_HAND_JOINT_MIDDLE_METACARPAL_EXT: return XR_HAND_JOINT_WRIST_EXT;
    case XR_HAND_JOINT_MIDDLE_PROXIMAL_EXT: return XR_HAND_JOINT_MIDDLE_METACARPAL_EXT;
    case XR_HAND_JOINT_MIDDLE_INTERMEDIATE_EXT: return XR_HAND_JOINT_MIDDLE_PROXIMAL_EXT;
    case XR_HAND_JOINT_MIDDLE_DISTAL_EXT: return XR_HAND_JOINT_MIDDLE_INTERMEDIATE_EXT;
    case XR_HAND_JOINT_MIDDLE_TIP_EXT: return XR_HAND_JOINT_MIDDLE_DISTAL_EXT;
    case XR_HAND_JOINT_RING_METACARPAL_EXT: return XR_HAND_JOINT_WRIST_EXT;
    case XR_HAND_JOINT_RING_PROXIMAL_EXT: return XR_HAND_JOINT_RING_METACARPAL_EXT;
    case XR_HAND_JOINT_RING_INTERMEDIATE_EXT: return XR_HAND_JOINT_RING_PROXIMAL_EXT;
    case XR_HAND_JOINT_RING_DISTAL_EXT: return XR_HAND_JOINT_RING_INTERMEDIATE_EXT;
    case XR_HAND_JOINT_RING_TIP_EXT: return XR_HAND_JOINT_RING_DISTAL_EXT;
    case XR_HAND_JOINT_LITTLE_METACARPAL_EXT: return XR_HAND_JOINT_WRIST_EXT;
    case XR_HAND_JOINT_LITTLE_PROXIMAL_EXT: return XR_HAND_JOINT_LITTLE_METACARPAL_EXT;
    case XR_HAND_JOINT_LITTLE_INTERMEDIATE_EXT: return XR_HAND_JOINT_LITTLE_PROXIMAL_EXT;
    case XR_HAND_JOINT_LITTLE_DISTAL_EXT: return XR_HAND_JOINT_LITTLE_INTERMEDIATE_EXT;
    case XR_HAND_JOINT_LITTLE_TIP_EXT: return XR_HAND_JOINT_LITTLE_DISTAL_EXT;
    }
    return h;
}

constexpr inline XrHandJointEXT ToXRHandJointType(const ALVR_HAND h)
{
    switch (h)
    {
    case ALVR_HAND::alvrHandBone_WristRoot: return XR_HAND_JOINT_WRIST_EXT;
    case ALVR_HAND::alvrHandBone_Thumb0: return XR_HAND_JOINT_THUMB_METACARPAL_EXT;
    case ALVR_HAND::alvrHandBone_Thumb1: return XR_HAND_JOINT_THUMB_PROXIMAL_EXT;
    case ALVR_HAND::alvrHandBone_Thumb2: return XR_HAND_JOINT_THUMB_DISTAL_EXT;
    case ALVR_HAND::alvrHandBone_Thumb3: return XR_HAND_JOINT_THUMB_TIP_EXT;
    case ALVR_HAND::alvrHandBone_Index1: return XR_HAND_JOINT_INDEX_PROXIMAL_EXT;
    case ALVR_HAND::alvrHandBone_Index2: return XR_HAND_JOINT_INDEX_INTERMEDIATE_EXT;
    case ALVR_HAND::alvrHandBone_Index3: return XR_HAND_JOINT_INDEX_DISTAL_EXT;
    case ALVR_HAND::alvrHandBone_Middle1: return XR_HAND_JOINT_MIDDLE_PROXIMAL_EXT;
    case ALVR_HAND::alvrHandBone_Middle2: return XR_HAND_JOINT_MIDDLE_INTERMEDIATE_EXT;
    case ALVR_HAND::alvrHandBone_Middle3: return XR_HAND_JOINT_MIDDLE_DISTAL_EXT;
    case ALVR_HAND::alvrHandBone_Ring1: return XR_HAND_JOINT_RING_PROXIMAL_EXT;
    case ALVR_HAND::alvrHandBone_Ring2: return XR_HAND_JOINT_RING_INTERMEDIATE_EXT;
    case ALVR_HAND::alvrHandBone_Ring3: return XR_HAND_JOINT_RING_DISTAL_EXT;
    case ALVR_HAND::alvrHandBone_Pinky0: return XR_HAND_JOINT_LITTLE_METACARPAL_EXT;
    case ALVR_HAND::alvrHandBone_Pinky1: return XR_HAND_JOINT_LITTLE_PROXIMAL_EXT;
    case ALVR_HAND::alvrHandBone_Pinky2: return XR_HAND_JOINT_LITTLE_INTERMEDIATE_EXT;
    case ALVR_HAND::alvrHandBone_Pinky3: return XR_HAND_JOINT_LITTLE_DISTAL_EXT;
    default: return XR_HAND_JOINT_MAX_ENUM_EXT;
    }
}

struct OpenXrProgram final : IOpenXrProgram {
    OpenXrProgram(const std::shared_ptr<Options>& options, const std::shared_ptr<IPlatformPlugin>& platformPlugin,
        const std::shared_ptr<IGraphicsPlugin>& graphicsPlugin)
        : m_options(options), m_platformPlugin(platformPlugin), m_graphicsPlugin(graphicsPlugin)
    {
        LogLayersAndExtensions();
    }

    OpenXrProgram(const std::shared_ptr<Options>& options, const std::shared_ptr<IPlatformPlugin>& platformPlugin)
        : m_options(options), m_platformPlugin(platformPlugin), m_graphicsPlugin{ nullptr }
    {
        LogLayersAndExtensions();
        auto& graphicsApi = options->GraphicsPlugin;
        if (graphicsApi.empty() || graphicsApi == "auto")
        {
            Log::Write(Log::Level::Info, "Running auto graphics api selection.");
            constexpr const auto to_graphics_api_str = [](const ALXRGraphicsApi gapi) -> std::tuple<std::string_view, std::string_view>
            {
                using namespace std::string_view_literals;
                switch (gapi)
                {
                case ALXRGraphicsApi::Vulkan2:  return std::make_tuple("XR_KHR_vulkan_enable2"sv, "Vulkan2"sv);
                case ALXRGraphicsApi::Vulkan:   return std::make_tuple("XR_KHR_vulkan_enable"sv, "Vulkan"sv);
                case ALXRGraphicsApi::D3D12:    return std::make_tuple("XR_KHR_D3D12_enable"sv, "D3D12"sv);
                case ALXRGraphicsApi::D3D11:    return std::make_tuple("XR_KHR_D3D11_enable"sv, "D3D11"sv);
                case ALXRGraphicsApi::OpenGLES: return std::make_tuple("XR_KHR_opengl_es_enable"sv, "OpenGLES"sv);
                default: return std::make_tuple("XR_KHR_opengl_enable"sv, "OpenGL"sv);
                }
            };
            for (size_t apiIndex = ALXRGraphicsApi::Vulkan2; apiIndex < size_t(ALXRGraphicsApi::ApiCount); ++apiIndex) {
                const auto& [ext_name, gapi] = to_graphics_api_str(static_cast<ALXRGraphicsApi>(apiIndex));
                auto itr = m_supportedGraphicsContexts.find(ext_name);
                if (itr != m_supportedGraphicsContexts.end() && itr->second) {
                    graphicsApi = gapi;
                    break;
                }
            }
        }
        m_graphicsPlugin = CreateGraphicsPlugin(options, platformPlugin);
        Log::Write(Log::Level::Info, Fmt("Selected Graphics API: %s", graphicsApi.c_str()));
    }

    virtual ~OpenXrProgram() override {
        Log::Write(Log::Level::Verbose, "Destroying OpenXrProgram");
                
        if (IsSessionRunning()) {
            CHECK_XRCMD(xrEndSession(m_session));
            m_sessionRunning.store(false);
        }

        if (m_ptLayerData.reconPassthroughLayer != XR_NULL_HANDLE) {
            Log::Write(Log::Level::Verbose, "Destroying PassthroughLayer");
            assert(m_pfnDestroyPassthroughLayerFB);
            m_pfnDestroyPassthroughLayerFB(m_ptLayerData.reconPassthroughLayer);
            m_ptLayerData.reconPassthroughLayer = XR_NULL_HANDLE;
        }

        if (m_ptLayerData.passthrough != XR_NULL_HANDLE) {
            Log::Write(Log::Level::Verbose, "Destroying Passthrough");
            assert(m_pfnDestroyPassthroughFB);
            m_pfnDestroyPassthroughFB(m_ptLayerData.passthrough);
            m_ptLayerData.passthrough = XR_NULL_HANDLE;
        }

        if (m_pfnDestroyHandTrackerEXT != nullptr)
        {
            Log::Write(Log::Level::Verbose, "Destroying HandTrackers");
            assert(m_pfnCreateHandTrackerEXT != nullptr);
            for (auto& handTracker : m_input.handerTrackers) {
                if (handTracker.tracker != XR_NULL_HANDLE) {
                    m_pfnDestroyHandTrackerEXT(handTracker.tracker);
                    handTracker.tracker = XR_NULL_HANDLE;
                }
            }
        }

        m_interactionManager.reset();

        if (m_visualizedSpaces.size() > 0) {
            Log::Write(Log::Level::Verbose, "Destroying Visualized XrSpaces");
        }
        for (XrSpace visualizedSpace : m_visualizedSpaces) {
            xrDestroySpace(visualizedSpace);
        }
        m_visualizedSpaces.clear();

        if (m_viewSpace != XR_NULL_HANDLE) {
            Log::Write(Log::Level::Verbose, "Destroying View XrSpaces");
            xrDestroySpace(m_viewSpace);
            m_viewSpace = XR_NULL_HANDLE;
        }

        if (m_boundingStageSpace != XR_NULL_HANDLE) {
            Log::Write(Log::Level::Verbose, "Destroying BoundingStage XrSpaces");
            xrDestroySpace(m_boundingStageSpace);
            m_boundingStageSpace = XR_NULL_HANDLE;
        }

        if (m_appSpace != XR_NULL_HANDLE) {
            Log::Write(Log::Level::Verbose, "Destroying App XrSpaces");
            xrDestroySpace(m_appSpace);
            m_appSpace = XR_NULL_HANDLE;
        }

        Log::Write(Log::Level::Verbose, "Destroying XrSwapChains");
        ClearSwapchains();

        if (m_session != XR_NULL_HANDLE) {
            Log::Write(Log::Level::Verbose, "Destroying XrSession");
            xrDestroySession(m_session);
            m_session = XR_NULL_HANDLE;
        }

        if (m_instance != XR_NULL_HANDLE) {
            Log::Write(Log::Level::Verbose, "Destroying XrInstance");
            xrDestroyInstance(m_instance);
            m_instance = XR_NULL_HANDLE;
        }

        Log::Write(Log::Level::Verbose, "Destroying GraphicsPlugin");
        m_graphicsPlugin.reset();
        Log::Write(Log::Level::Verbose, "Destroying PlatformPlugin");
        m_platformPlugin.reset();

        m_systemId = XR_NULL_SYSTEM_ID;

        Log::Write(Log::Level::Verbose, "OpenXrProgram Destroyed.");
    }

    using ExtensionMap = std::unordered_map<std::string_view, bool>;
    ExtensionMap m_availableSupportedExtMap = {
#ifdef XR_USE_PLATFORM_UWP
#pragma message ("UWP Extensions Enabled.")
        // Require XR_EXT_win32_appcontainer_compatible extension when building in UWP context.
        { XR_EXT_WIN32_APPCONTAINER_COMPATIBLE_EXTENSION_NAME, false },
#endif
#ifdef XR_USE_PLATFORM_ANDROID
        { XR_KHR_ANDROID_THREAD_SETTINGS_EXTENSION_NAME, false },
#endif
        { XR_EXT_PERFORMANCE_SETTINGS_EXTENSION_NAME, false },
        { XR_MSFT_UNBOUNDED_REFERENCE_SPACE_EXTENSION_NAME, false },
        { XR_MSFT_HAND_INTERACTION_EXTENSION_NAME, false },
        { XR_HTC_VIVE_COSMOS_CONTROLLER_INTERACTION_EXTENSION_NAME, false },
        { XR_HTC_VIVE_FOCUS3_CONTROLLER_INTERACTION_EXTENSION_NAME, false },
        { XR_HTC_HAND_INTERACTION_EXTENSION_NAME, false },
        { XR_KHR_CONVERT_TIMESPEC_TIME_EXTENSION_NAME, false },
#ifdef XR_USE_PLATFORM_WIN32
        { XR_KHR_WIN32_CONVERT_PERFORMANCE_COUNTER_TIME_EXTENSION_NAME, false },
#endif
        { XR_EXT_HAND_TRACKING_EXTENSION_NAME, false },
        { XR_FB_DISPLAY_REFRESH_RATE_EXTENSION_NAME, false },
        { XR_FB_COLOR_SPACE_EXTENSION_NAME, false },
        { XR_FB_PASSTHROUGH_EXTENSION_NAME, false },
#ifdef XR_USE_OXR_PICO
#pragma message ("Pico Neo 3 OXR Extensions Enabled.")
        { XR_PICO_PERFORMANCE_SETTINGS_EXTENSION_NAME, false },
        { XR_PICO_VIEW_STATE_EXT_ENABLE_EXTENSION_NAME, false },
        { XR_PICO_FRAME_END_INFO_EXT_EXTENSION_NAME, false },
        { XR_PICO_ANDROID_CONTROLLER_FUNCTION_EXT_ENABLE_EXTENSION_NAME, false },
        { XR_PICO_CONFIGS_EXT_EXTENSION_NAME, false },
        { XR_PICO_RESET_SENSOR_EXTENSION_NAME, false },
        { XR_PICO_SESSION_BEGIN_INFO_EXT_ENABLE_EXTENSION_NAME, false },
        { XR_PICO_SINGLEPASS_ENABLE_EXTENSION_NAME, false }
#endif
    };
    ExtensionMap m_supportedGraphicsContexts = {
#ifdef XR_USE_GRAPHICS_API_VULKAN
        { XR_KHR_VULKAN_ENABLE2_EXTENSION_NAME,   false },
        { XR_KHR_VULKAN_ENABLE_EXTENSION_NAME,    false },
#endif
#ifdef XR_USE_GRAPHICS_API_D3D12
        { XR_KHR_D3D12_ENABLE_EXTENSION_NAME,     false },
#endif
#ifdef XR_USE_GRAPHICS_API_D3D11
        { XR_KHR_D3D11_ENABLE_EXTENSION_NAME,     false },
#endif                                            
#ifdef XR_USE_GRAPHICS_API_OPENGL
        { XR_KHR_OPENGL_ENABLE_EXTENSION_NAME,    false },
#endif
#ifdef XR_USE_GRAPHICS_API_OPENGL_ES
        { XR_KHR_OPENGL_ES_ENABLE_EXTENSION_NAME, false }
#endif
    };

    void LogLayersAndExtensions() {
        // Write out extension properties for a given layer.
        const auto logExtensions = [this](const char* layerName, int indent = 0) {
            uint32_t instanceExtensionCount = 0;
            CHECK_XRCMD(xrEnumerateInstanceExtensionProperties(layerName, 0, &instanceExtensionCount, nullptr));

            std::vector<XrExtensionProperties> extensions(instanceExtensionCount, {
                .type = XR_TYPE_EXTENSION_PROPERTIES,
                .next = nullptr
            });
            CHECK_XRCMD(xrEnumerateInstanceExtensionProperties(layerName, (uint32_t)extensions.size(), &instanceExtensionCount,
                extensions.data()));

            constexpr const auto SetExtensionMap = [](auto& extMap, const std::string_view extName)
            {
                const auto itr = extMap.find(extName);
                if (itr == extMap.end())
                    return;
                itr->second = true;
            };
            const std::string indentStr(indent, ' ');
            Log::Write(Log::Level::Verbose, Fmt("%sAvailable Extensions: (%d)", indentStr.c_str(), instanceExtensionCount));
            for (const XrExtensionProperties& extension : extensions) {

                SetExtensionMap(m_availableSupportedExtMap,  extension.extensionName);
                SetExtensionMap(m_supportedGraphicsContexts, extension.extensionName);
                Log::Write(Log::Level::Verbose, Fmt("%s  Name=%s SpecVersion=%d", indentStr.c_str(), extension.extensionName,
                    extension.extensionVersion));
            }
        };

        // Log non-layer extensions (layerName==nullptr).
        logExtensions(nullptr);

        // Log layers and any of their extensions.
        {
            uint32_t layerCount = 0;
            CHECK_XRCMD(xrEnumerateApiLayerProperties(0, &layerCount, nullptr));
            std::vector<XrApiLayerProperties> layers(layerCount, {
                .type = XR_TYPE_API_LAYER_PROPERTIES,
                .next = nullptr
            });
            CHECK_XRCMD(xrEnumerateApiLayerProperties((uint32_t)layers.size(), &layerCount, layers.data()));

            Log::Write(Log::Level::Info, Fmt("Available Layers: (%d)", layerCount));
            for (const XrApiLayerProperties& layer : layers) {
                Log::Write(Log::Level::Verbose,
                    Fmt("  Name=%s SpecVersion=%s LayerVersion=%d Description=%s", layer.layerName,
                        GetXrVersionString(layer.specVersion).c_str(), layer.layerVersion, layer.description));
                logExtensions(layer.layerName, 4);
            }
        }
    }

    inline bool IsRuntime(const OxrRuntimeType rtType) const {
        return rtType == m_runtimeType;
    }

    void LogInstanceInfo() {
        CHECK(m_instance != XR_NULL_HANDLE && m_graphicsPlugin != nullptr);

        XrInstanceProperties instanceProperties{
            .type = XR_TYPE_INSTANCE_PROPERTIES,
            .next = nullptr
        };
        CHECK_XRCMD(xrGetInstanceProperties(m_instance, &instanceProperties));

        Log::Write(Log::Level::Info, Fmt("Instance RuntimeName=%s RuntimeVersion=%s", instanceProperties.runtimeName,
            GetXrVersionString(instanceProperties.runtimeVersion).c_str()));

        m_runtimeType = FromString(instanceProperties.runtimeName);
#ifdef XR_USE_OXR_PICO
        m_graphicsPlugin->SetEnableLinearizeRGB(false);
#else
        m_graphicsPlugin->SetEnableLinearizeRGB(!(m_options->DisableLinearizeSrgb || IsRuntime(OxrRuntimeType::HTCWave)));
#endif
    }

    void CreateInstanceInternal() {
        CHECK(m_instance == XR_NULL_HANDLE);

        // Create union of extensions required by platform and graphics plugins.
        std::vector<const char*> extensions;

        // Transform platform and graphics extension std::strings to C strings.
        const std::vector<std::string> platformExtensions = m_platformPlugin->GetInstanceExtensions();
        std::transform(platformExtensions.begin(), platformExtensions.end(), std::back_inserter(extensions),
            [](const std::string& ext) { return ext.c_str(); });
        const std::vector<std::string> graphicsExtensions = m_graphicsPlugin->GetInstanceExtensions();
        std::transform(graphicsExtensions.begin(), graphicsExtensions.end(), std::back_inserter(extensions),
            [](const std::string& ext) { return ext.c_str(); });

        for (const auto& [extName, extAvaileble] : m_availableSupportedExtMap) {
            if (extAvaileble) {
                extensions.push_back(extName.data());
            }
        }

        Log::Write(Log::Level::Info, "Selected extensions to enable:");
        for (const auto& extName : extensions) {
            Log::Write(Log::Level::Info, Fmt("\t%s", extName));
        }

        XrInstanceCreateInfo createInfo {
            .type = XR_TYPE_INSTANCE_CREATE_INFO,
            .next = m_platformPlugin->GetInstanceCreateExtension(),
            .applicationInfo {
                .applicationVersion = 1,
                .engineVersion = 1,
                .apiVersion = XR_CURRENT_API_VERSION
            },
            .enabledApiLayerCount = 0,
            .enabledApiLayerNames = nullptr,
            .enabledExtensionCount = (uint32_t)extensions.size(),
            .enabledExtensionNames = extensions.data()
        };
        auto& appInfo = createInfo.applicationInfo;
        std::strcpy(appInfo.applicationName, "alxr-client");
        std::strcpy(appInfo.engineName, "alxr-engine");
        CHECK_XRCMD(xrCreateInstance(&createInfo, &m_instance));
    }

    void CreateInstance() override {
        //LogLayersAndExtensions();

        CreateInstanceInternal();

        LogInstanceInfo();
    }

    void LogViewConfigurations() {
        CHECK(m_instance != XR_NULL_HANDLE);
        CHECK(m_systemId != XR_NULL_SYSTEM_ID);

        uint32_t viewConfigTypeCount = 0;
        CHECK_XRCMD(xrEnumerateViewConfigurations(m_instance, m_systemId, 0, &viewConfigTypeCount, nullptr));
        std::vector<XrViewConfigurationType> viewConfigTypes(viewConfigTypeCount, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO);
        CHECK_XRCMD(xrEnumerateViewConfigurations(m_instance, m_systemId, viewConfigTypeCount, &viewConfigTypeCount,
            viewConfigTypes.data()));
        CHECK((uint32_t)viewConfigTypes.size() == viewConfigTypeCount);

        Log::Write(Log::Level::Info, Fmt("Available View Configuration Types: (%d)", viewConfigTypeCount));
        for (XrViewConfigurationType viewConfigType : viewConfigTypes) {
            Log::Write(Log::Level::Verbose, Fmt("  View Configuration Type: %s %s", to_string(viewConfigType),
                viewConfigType == m_viewConfigType ? "(Selected)" : ""));

            XrViewConfigurationProperties viewConfigProperties{ .type=XR_TYPE_VIEW_CONFIGURATION_PROPERTIES, .next=nullptr };
            CHECK_XRCMD(xrGetViewConfigurationProperties(m_instance, m_systemId, viewConfigType, &viewConfigProperties));

            Log::Write(Log::Level::Verbose,
                Fmt("  View configuration FovMutable=%s", viewConfigProperties.fovMutable == XR_TRUE ? "True" : "False"));

            uint32_t viewCount = 0;
            CHECK_XRCMD(xrEnumerateViewConfigurationViews(m_instance, m_systemId, viewConfigType, 0, &viewCount, nullptr));
            if (viewCount > 0) {
                std::vector<XrViewConfigurationView> views(viewCount, { .type=XR_TYPE_VIEW_CONFIGURATION_VIEW, .next=nullptr });
                CHECK_XRCMD(
                    xrEnumerateViewConfigurationViews(m_instance, m_systemId, viewConfigType, viewCount, &viewCount, views.data()));

                for (uint32_t i = 0; i < views.size(); ++i) {
                    const XrViewConfigurationView& view = views[i];

                    Log::Write(Log::Level::Verbose, Fmt("    View [%d]: Recommended Width=%d Height=%d SampleCount=%d", i,
                        view.recommendedImageRectWidth, view.recommendedImageRectHeight,
                        view.recommendedSwapchainSampleCount));
                    Log::Write(Log::Level::Verbose,
                        Fmt("    View [%d]:     Maximum Width=%d Height=%d SampleCount=%d", i, view.maxImageRectWidth,
                            view.maxImageRectHeight, view.maxSwapchainSampleCount));
                }
            }
            else {
                Log::Write(Log::Level::Error, Fmt("Empty view configuration type"));
            }
            LogEnvironmentBlendMode(viewConfigType);
        }
    }

    using XrEnvironmentBlendModeList = std::vector<XrEnvironmentBlendMode>;
    XrEnvironmentBlendModeList GetEnvironmentBlendModes(const XrViewConfigurationType type) const
    {
        uint32_t count = 0;
        CHECK_XRCMD(xrEnumerateEnvironmentBlendModes(m_instance, m_systemId, type, 0, &count, nullptr));
        if (count == 0)
            return {};
        std::vector<XrEnvironmentBlendMode> blendModes(count, XR_ENVIRONMENT_BLEND_MODE_OPAQUE);
        CHECK_XRCMD(xrEnumerateEnvironmentBlendModes(m_instance, m_systemId, type, count, &count, blendModes.data()));
        return blendModes;
    }

    void LogEnvironmentBlendMode(XrViewConfigurationType type) {
        CHECK(m_instance != XR_NULL_HANDLE);
        CHECK(m_systemId != 0);

        const auto blendModes = GetEnvironmentBlendModes(type);
        Log::Write(Log::Level::Info, Fmt("Available Environment Blend Mode count : (%zu)", blendModes.size()));
        
        for (XrEnvironmentBlendMode mode : blendModes) {
            const bool blendModeMatch = (mode == m_environmentBlendMode);
            Log::Write(Log::Level::Info,
                Fmt("Environment Blend Mode (%s) : %s", to_string(mode), blendModeMatch ? "(Selected)" : ""));
        }
    }

    void InitializeSystem(const ALXR::ALXRPaths& alxrPaths) override {

        CHECK(m_instance != XR_NULL_HANDLE);
        CHECK(m_systemId == XR_NULL_SYSTEM_ID);

        m_alxrPaths = alxrPaths;
        CHECK(m_alxrPaths != ALXR::ALXR_NULL_PATHS);

        m_formFactor = GetXrFormFactor(m_options->FormFactor);
        m_viewConfigType = GetXrViewConfigurationType(m_options->ViewConfiguration);
        m_environmentBlendMode = GetXrEnvironmentBlendMode(m_options->EnvironmentBlendMode);

        const XrSystemGetInfo systemInfo{
            .type = XR_TYPE_SYSTEM_GET_INFO,
            .next = nullptr,
            .formFactor = m_formFactor
        };        
        CHECK_XRCMD(xrGetSystem(m_instance, &systemInfo, &m_systemId));

        Log::Write(Log::Level::Verbose, Fmt("Using system %d for form factor %s", m_systemId, to_string(m_formFactor)));
        CHECK(m_instance != XR_NULL_HANDLE);
        CHECK(m_systemId != XR_NULL_SYSTEM_ID);

        LogViewConfigurations();

        const auto blendModes = GetEnvironmentBlendModes(m_viewConfigType);
        if (std::find(blendModes.begin(), blendModes.end(), m_environmentBlendMode) == blendModes.end() && !blendModes.empty()) {
            Log::Write(Log::Level::Info, Fmt
            (
                "Requested environment blend mode (%s) is not available, using first available mode (%s)",
                to_string(m_environmentBlendMode),
                to_string(blendModes[0])
            ));
            m_environmentBlendMode = blendModes[0];
        }

        // The graphics API can initialize the graphics device now that the systemId and instance
        // handle are available.
        m_graphicsPlugin->InitializeDevice(m_instance, m_systemId, m_environmentBlendMode);
        m_isMultiViewEnabled = m_graphicsPlugin->IsMultiViewEnabled();
        
        Log::Write(Log::Level::Info, m_isMultiViewEnabled ?
            "Multi-view rendering enabled." :
            "Multi-view rendering not supported.");
    }

    inline std::vector<XrReferenceSpaceType> GetAvailableReferenceSpaces() const
    {
        CHECK(m_session != XR_NULL_HANDLE);
        uint32_t spaceCount = 0;
        CHECK_XRCMD(xrEnumerateReferenceSpaces(m_session, 0, &spaceCount, nullptr));
        assert(spaceCount > 0);
        std::vector<XrReferenceSpaceType> spaces(spaceCount);
        CHECK_XRCMD(xrEnumerateReferenceSpaces(m_session, spaceCount, &spaceCount, spaces.data()));
        return spaces;
    }

    inline XrReferenceSpaceCreateInfo GetAppReferenceSpaceCreateInfo() const {

        const auto appReferenceSpaceType = [this]() -> std::string_view
        {
            constexpr const auto refSpaceName = [](const XrReferenceSpaceType refType) {
                switch (refType) {
                case XR_REFERENCE_SPACE_TYPE_VIEW: return "View";
                case XR_REFERENCE_SPACE_TYPE_LOCAL: return "Local";
                case XR_REFERENCE_SPACE_TYPE_STAGE: return "Stage";
                case XR_REFERENCE_SPACE_TYPE_UNBOUNDED_MSFT: return "UboundedMSFT";
                //case XR_REFERENCE_SPACE_TYPE_COMBINED_EYE_VARJO:
                };
                assert(false); // "Uknown HMD reference space type"
                return "Stage";
            };
            const auto availSpaces = GetAvailableReferenceSpaces();
            assert(availSpaces.size() > 0);
            // iterate through order of preference/priority, STAGE is the most preferred if available.
            for (const auto spaceType : {   XR_REFERENCE_SPACE_TYPE_STAGE,
                                            XR_REFERENCE_SPACE_TYPE_UNBOUNDED_MSFT,
                                            XR_REFERENCE_SPACE_TYPE_LOCAL,
                                            XR_REFERENCE_SPACE_TYPE_VIEW })
            {
                if (std::find(availSpaces.begin(), availSpaces.end(), spaceType) != availSpaces.end())
                    return refSpaceName(spaceType);
            }
            // should never reach this point.
            return refSpaceName(availSpaces[0]);
        }();
        return GetXrReferenceSpaceCreateInfo(appReferenceSpaceType);
    }

#ifdef XR_USE_PLATFORM_WIN32
    static inline std::uint64_t ToTimeUs(const LARGE_INTEGER& ctr)
    {
        const std::int64_t freq = _Query_perf_frequency(); // doesn't change after system boot
        const std::int64_t whole = (ctr.QuadPart / freq) * std::micro::den;
        const std::int64_t part = (ctr.QuadPart % freq) * std::micro::den / freq;
        return static_cast<std::uint64_t>(whole + part);
    }
#else
    static inline std::uint64_t ToTimeUs(const struct timespec& ts)
    {
        return (ts.tv_sec * 1000000) + (ts.tv_nsec / 1000);
    }
#endif

    inline std::uint64_t FromXrTimeUs(const XrTime xrt, const std::uint64_t defaultVal = std::uint64_t(-1)) const
    {
#ifdef XR_USE_PLATFORM_WIN32
        if (m_pfnConvertTimeToWin32PerformanceCounterKHR == nullptr)
            return defaultVal;
        LARGE_INTEGER ctr;
        if (m_pfnConvertTimeToWin32PerformanceCounterKHR(m_instance, xrt, &ctr) == XR_ERROR_TIME_INVALID)
            return defaultVal;
        return ToTimeUs(ctr);
#else
        if (m_pfnConvertTimeToTimespecTimeKHR == nullptr)
            return defaultVal;
        struct timespec ts;
        if (m_pfnConvertTimeToTimespecTimeKHR(m_instance, xrt, &ts) == XR_ERROR_TIME_INVALID)
            return defaultVal;
        return ToTimeUs(ts);
#endif
    }

    virtual inline std::tuple<XrTime, std::uint64_t> XrTimeNow() const override
    {
#ifdef XR_USE_PLATFORM_WIN32
        if (m_pfnConvertWin32PerformanceCounterToTimeKHR == nullptr)
            return { -1, std::uint64_t(-1) };
        LARGE_INTEGER ctr;
        QueryPerformanceCounter(&ctr);
        XrTime xrTimeNow;
        if (m_pfnConvertWin32PerformanceCounterToTimeKHR(m_instance, &ctr, &xrTimeNow) == XR_ERROR_TIME_INVALID)
            return { -1, std::uint64_t(-1) };
        return { xrTimeNow, ToTimeUs(ctr) };
#else
        if (m_pfnConvertTimespecTimeToTimeKHR == nullptr)
            return { -1, std::uint64_t(-1) };
        struct timespec ts;
        if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0)
            return { -1, std::uint64_t(-1) };
        XrTime xrTimeNow;
        if (m_pfnConvertTimespecTimeToTimeKHR(m_instance, &ts, &xrTimeNow) == XR_ERROR_TIME_INVALID)
            return { -1, std::uint64_t(-1) };
        return { xrTimeNow, ToTimeUs(ts) };
#endif
    }

    void LogReferenceSpaces() {
        CHECK(m_session != XR_NULL_HANDLE);

        const auto spaces = GetAvailableReferenceSpaces();
        Log::Write(Log::Level::Info, Fmt("Available reference spaces: %d", spaces.size()));
        for (const XrReferenceSpaceType space : spaces) {
            Log::Write(Log::Level::Verbose, Fmt("  Name: %s", to_string(space)));
        }
    }

    void InitializeActions() {
        CHECK(m_instance != XR_NULL_HANDLE);

        const auto IsProfileSupported = [this](const auto& profile)
        {
            if (IsRuntime(OxrRuntimeType::HTCWave)) {
                if (profile.IsCore())
                    return false;
                constexpr const std::array<const std::string_view, 2> HTCFilterList{
                    XR_HTC_VIVE_FOCUS3_CONTROLLER_INTERACTION_EXTENSION_NAME,
                    XR_HTC_HAND_INTERACTION_EXTENSION_NAME
                };
                for (const auto& htcProfile : HTCFilterList) {
                    if (htcProfile == profile.extensionName)
                        return IsExtEnabled(profile.extensionName);
                }
                return false;
            }
            return profile.IsCore() || IsExtEnabled(profile.extensionName);
        };
        m_interactionManager = std::make_unique<ALXR::InteractionManager>
        (
            m_instance,
            m_session,
            m_alxrPaths,
            IsPassthroughSupported() ?
                [this](const ALXR::PassthroughMode newMode) { TogglePassthroughMode(newMode); } : ALXR::TogglePTModeFn {},
#ifdef XR_USE_OXR_PICO
            m_pfnXrVibrateControllerPico,
#endif            
            IsProfileSupported
        );
    }

    inline bool IsExtEnabled(const std::string_view& extName) const
    {
        auto ext_itr = m_availableSupportedExtMap.find(extName);
        return ext_itr != m_availableSupportedExtMap.end() && ext_itr->second;
    }

    bool InitializeExtensions()
    {
        CHECK(m_instance != XR_NULL_HANDLE);
        CHECK(m_session != XR_NULL_HANDLE);

#ifdef XR_USE_PLATFORM_WIN32
        if (IsExtEnabled(XR_KHR_WIN32_CONVERT_PERFORMANCE_COUNTER_TIME_EXTENSION_NAME))
        {
            Log::Write(Log::Level::Info, Fmt("%s enabled.", XR_KHR_WIN32_CONVERT_PERFORMANCE_COUNTER_TIME_EXTENSION_NAME));
            CHECK_XRCMD(xrGetInstanceProcAddr(m_instance, "xrConvertTimeToWin32PerformanceCounterKHR",
                reinterpret_cast<PFN_xrVoidFunction*>(&m_pfnConvertTimeToWin32PerformanceCounterKHR)));
            CHECK_XRCMD(xrGetInstanceProcAddr(m_instance, "xrConvertWin32PerformanceCounterToTimeKHR",
                reinterpret_cast<PFN_xrVoidFunction*>(&m_pfnConvertWin32PerformanceCounterToTimeKHR)));
        }
#endif
        if (IsExtEnabled(XR_KHR_CONVERT_TIMESPEC_TIME_EXTENSION_NAME))
        {
            Log::Write(Log::Level::Info, Fmt("%s enabled.", XR_KHR_CONVERT_TIMESPEC_TIME_EXTENSION_NAME));
            CHECK_XRCMD(xrGetInstanceProcAddr(m_instance, "xrConvertTimespecTimeToTimeKHR",
                reinterpret_cast<PFN_xrVoidFunction*>(&m_pfnConvertTimespecTimeToTimeKHR)));
            CHECK_XRCMD(xrGetInstanceProcAddr(m_instance, "xrConvertTimeToTimespecTimeKHR",
                reinterpret_cast<PFN_xrVoidFunction*>(&m_pfnConvertTimeToTimespecTimeKHR)));
        }

        if (IsExtEnabled(XR_FB_COLOR_SPACE_EXTENSION_NAME))
        {
            Log::Write(Log::Level::Info, Fmt("%s enabled.", XR_FB_COLOR_SPACE_EXTENSION_NAME));
            CHECK_XRCMD(xrGetInstanceProcAddr(m_instance, "xrEnumerateColorSpacesFB",
                reinterpret_cast<PFN_xrVoidFunction*>(&m_pfnEnumerateColorSpacesFB)));
            CHECK_XRCMD(xrGetInstanceProcAddr(m_instance, "xrSetColorSpaceFB",
                reinterpret_cast<PFN_xrVoidFunction*>(&m_pfnSetColorSpaceFB)));
        }

        if (IsExtEnabled(XR_FB_DISPLAY_REFRESH_RATE_EXTENSION_NAME))
        {
            Log::Write(Log::Level::Info, Fmt("%s enabled.", XR_FB_DISPLAY_REFRESH_RATE_EXTENSION_NAME));
            CHECK_XRCMD(xrGetInstanceProcAddr(m_instance, "xrEnumerateDisplayRefreshRatesFB",
                reinterpret_cast<PFN_xrVoidFunction*>(&m_pfnEnumerateDisplayRefreshRatesFB)));
            CHECK_XRCMD(xrGetInstanceProcAddr(m_instance, "xrGetDisplayRefreshRateFB",
                reinterpret_cast<PFN_xrVoidFunction*>(&m_pfnGetDisplayRefreshRateFB)));
            CHECK_XRCMD(xrGetInstanceProcAddr(m_instance, "xrRequestDisplayRefreshRateFB",
                reinterpret_cast<PFN_xrVoidFunction*>(&m_pfnRequestDisplayRefreshRateFB)));
        }

#ifdef XR_USE_OXR_PICO
        const auto GetPicoInstanceProcAddr = [this](const char* const name, auto& fn)
        {
            const XrResult result = xrGetInstanceProcAddr(m_instance, name, reinterpret_cast<PFN_xrVoidFunction*>(&fn));
            if (result != XR_SUCCESS) {
                Log::Write(Log::Level::Warning, Fmt("Unable to load xr-extension function: %s, error-code: %d", name, result));
            }
        };

        if (IsExtEnabled(XR_PICO_ANDROID_CONTROLLER_FUNCTION_EXT_ENABLE_EXTENSION_NAME))
        {
            Log::Write(Log::Level::Info, Fmt("%s enabled.", XR_PICO_ANDROID_CONTROLLER_FUNCTION_EXT_ENABLE_EXTENSION_NAME));
            GetPicoInstanceProcAddr("xrGetControllerConnectionStatePico", m_pfnGetControllerConnectionStatePico);
            GetPicoInstanceProcAddr("xrSetEngineVersionPico", m_pfnSetEngineVersionPico);
            GetPicoInstanceProcAddr("xrStartCVControllerThreadPico", m_pfnStartCVControllerThreadPico);
            GetPicoInstanceProcAddr("xrStopCVControllerThreadPico", m_pfnStopCVControllerThreadPico);
            GetPicoInstanceProcAddr("xrVibrateControllerPico", m_pfnXrVibrateControllerPico);
        }

        if (IsExtEnabled(XR_PICO_CONFIGS_EXT_EXTENSION_NAME))
        {
            Log::Write(Log::Level::Info, Fmt("%s enabled.", XR_PICO_CONFIGS_EXT_EXTENSION_NAME));
            GetPicoInstanceProcAddr("xrGetConfigPICO", m_pfnGetConfigPICO);
            GetPicoInstanceProcAddr("xrSetConfigPICO", m_pfnSetConfigPICO);
        }

        if (IsExtEnabled(XR_PICO_RESET_SENSOR_EXTENSION_NAME))
        {
            Log::Write(Log::Level::Info, Fmt("%s enabled.", XR_PICO_RESET_SENSOR_EXTENSION_NAME));
            GetPicoInstanceProcAddr("xrResetSensorPICO", m_pfnResetSensorPICO);
        }

        if (IsExtEnabled(XR_PICO_SESSION_BEGIN_INFO_EXT_ENABLE_EXTENSION_NAME)) {
            Log::Write(Log::Level::Info, Fmt("%s enabled.", XR_PICO_SESSION_BEGIN_INFO_EXT_ENABLE_EXTENSION_NAME));
        }

        if (IsExtEnabled(XR_PICO_SINGLEPASS_ENABLE_EXTENSION_NAME)) {
            Log::Write(Log::Level::Info, Fmt("%s enabled.", XR_PICO_SINGLEPASS_ENABLE_EXTENSION_NAME));
        }
        
        if (m_pfnSetConfigPICO) {
            // const auto picoPlatformStr = std::to_string(Platform::NATIVE);
            // m_pfnSetConfigPICO(m_session, ConfigsSetEXT::PLATFORM, const_cast<char*>(picoPlatformStr.c_str()));
            // m_pfnSetConfigPICO(m_session, ConfigsSetEXT::ENABLE_SIX_DOF, "1");

            const auto trackingOriginStr = std::to_string(TrackingOrigin::STAGELEVEL);
            Log::Write(Log::Level::Info, Fmt("Setting Pico Tracking Origin: %s", trackingOriginStr.c_str()));
            m_pfnSetConfigPICO(m_session, ConfigsSetEXT::TRACKING_ORIGIN, const_cast<char*>(trackingOriginStr.c_str()));
        }
#endif

        SetDeviceColorSpace();
        UpdateSupportedDisplayRefreshRates();
        InitializePassthroughAPI();
        return InitializeHandTrackers();
    }

    bool SetDeviceColorSpace()
    {
        if (m_pfnSetColorSpaceFB == nullptr)
            return false;

        //constexpr const auto to_string = [](const XrColorSpaceFB csType)
        //{
        //    switch (csType)
        //    {
        //    case XR_COLOR_SPACE_UNMANAGED_FB: return "UNMANAGED";
        //    case XR_COLOR_SPACE_REC2020_FB: return "REC2020";
        //    case XR_COLOR_SPACE_REC709_FB: return "REC709";
        //    case XR_COLOR_SPACE_RIFT_CV1_FB: return "RIFT_CV1";
        //    case XR_COLOR_SPACE_RIFT_S_FB: return "RIFT_S";
        //    case XR_COLOR_SPACE_QUEST_FB: return "QUEST";
        //    case XR_COLOR_SPACE_P3_FB: return "P3";
        //    case XR_COLOR_SPACE_ADOBE_RGB_FB: return "ADOBE_RGB";
        //    }
        //    return "unknown-color-space-type";
        //};

        //uint32_t colorSpaceCount = 0;
        //CHECK_XRCMD(m_pfnEnumerateColorSpacesFB(m_session, 0, &colorSpaceCount, nullptr));

        //std::vector<XrColorSpaceFB> colorSpaceTypes{ colorSpaceCount, XR_COLOR_SPACE_UNMANAGED_FB };
        //CHECK_XRCMD(m_pfnEnumerateColorSpacesFB(m_session, colorSpaceCount, &colorSpaceCount, colorSpaceTypes.data()));

        CHECK_XRCMD(m_pfnSetColorSpaceFB(m_session, XR_COLOR_SPACE_REC2020_FB));
        Log::Write(Log::Level::Info, "Color space set.");
        return true;
    }

    bool InitializeHandTrackers()
    {
        //if (m_instance != XR_NULL_HANDLE && m_systemId != XR_NULL_SYSTEM_ID)
        //    return false;

        // Inspect hand tracking system properties
        XrSystemHandTrackingPropertiesEXT handTrackingSystemProperties{ .type=XR_TYPE_SYSTEM_HAND_TRACKING_PROPERTIES_EXT, .next=nullptr };
        XrSystemProperties systemProperties{ .type=XR_TYPE_SYSTEM_PROPERTIES, .next = &handTrackingSystemProperties };
        CHECK_XRCMD(xrGetSystemProperties(m_instance, m_systemId, &systemProperties));
        if (!handTrackingSystemProperties.supportsHandTracking) {
            Log::Write(Log::Level::Info, Fmt("%s is not supported.", XR_EXT_HAND_TRACKING_EXTENSION_NAME));
            // The system does not support hand tracking
            return false;
        }

        // Get function pointer for xrCreateHandTrackerEXT
        CHECK_XRCMD(xrGetInstanceProcAddr(m_instance, "xrCreateHandTrackerEXT",
            reinterpret_cast<PFN_xrVoidFunction*>(&m_pfnCreateHandTrackerEXT)));

        // Get function pointer for xrLocateHandJointsEXT
        CHECK_XRCMD(xrGetInstanceProcAddr(m_instance, "xrLocateHandJointsEXT",
            reinterpret_cast<PFN_xrVoidFunction*>(&m_pfnLocateHandJointsEXT)));

        // Get function pointer for xrLocateHandJointsEXT
        CHECK_XRCMD(xrGetInstanceProcAddr(m_instance, "xrDestroyHandTrackerEXT",
            reinterpret_cast<PFN_xrVoidFunction*>(&m_pfnDestroyHandTrackerEXT)));

        if (m_pfnCreateHandTrackerEXT == nullptr ||
            m_pfnLocateHandJointsEXT == nullptr  ||
            m_pfnDestroyHandTrackerEXT == nullptr)
            return false;
        Log::Write(Log::Level::Info, Fmt("%s is enabled.", XR_EXT_HAND_TRACKING_EXTENSION_NAME));

        // Create a hand tracker for left hand that tracks default set of hand joints.
        const auto createHandTracker = [&](auto& handerTracker, const XrHandEXT hand)
        {
            const XrHandTrackerCreateInfoEXT createInfo{
                .type = XR_TYPE_HAND_TRACKER_CREATE_INFO_EXT,
                .next = nullptr,
                .hand = hand,
                .handJointSet = XR_HAND_JOINT_SET_DEFAULT_EXT
            };
            CHECK_XRCMD(m_pfnCreateHandTrackerEXT(m_session, &createInfo, &handerTracker.tracker));
        };
        createHandTracker(m_input.handerTrackers[0], XR_HAND_LEFT_EXT);
        createHandTracker(m_input.handerTrackers[1], XR_HAND_RIGHT_EXT);

        auto& leftHandBaseOrientation = m_input.handerTrackers[0].baseOrientation;
        auto& rightHandBaseOrientation = m_input.handerTrackers[1].baseOrientation;   
        XrMatrix4x4f zRot;
        XrMatrix4x4f& yRot = rightHandBaseOrientation;
        XrMatrix4x4f_CreateRotation(&yRot, 0.0, -90.0f, 0.0f);
        XrMatrix4x4f_CreateRotation(&zRot, 0.0, 0.0f, 180.0f);
        XrMatrix4x4f_Multiply(&leftHandBaseOrientation, &yRot, &zRot);
        return true;
    }

    void InitializePassthroughAPI()
    {
        if (m_instance == XR_NULL_HANDLE ||
            m_systemId == XR_NULL_SYSTEM_ID ||
            !IsExtEnabled(XR_FB_PASSTHROUGH_EXTENSION_NAME)) {
            return;
        }

        XrSystemPassthroughPropertiesFB passthroughSystemProperties{
            .type = XR_TYPE_SYSTEM_PASSTHROUGH_PROPERTIES_FB,
            .next = nullptr
        };
        XrSystemProperties systemProperties{
            .type = XR_TYPE_SYSTEM_PROPERTIES,
            .next = &passthroughSystemProperties
        };
        CHECK_XRCMD(xrGetSystemProperties(m_instance, m_systemId, &systemProperties));
        if (passthroughSystemProperties.supportsPassthrough == XR_FALSE) {
            Log::Write(Log::Level::Info, Fmt("%s is not supported.", XR_FB_PASSTHROUGH_EXTENSION_NAME));
            return;
        }
        Log::Write(Log::Level::Info, Fmt("%s enabled.", XR_FB_PASSTHROUGH_EXTENSION_NAME));

#define CAT(x,y) x ## y
#define INIT_PFN(ExtName)\
    CHECK_XRCMD(xrGetInstanceProcAddr(m_instance, "xr"#ExtName, reinterpret_cast<PFN_xrVoidFunction*>(&CAT(m_pfn,ExtName))));

        INIT_PFN(CreatePassthroughFB);
        INIT_PFN(DestroyPassthroughFB);
        INIT_PFN(PassthroughStartFB);
        INIT_PFN(PassthroughPauseFB);
        INIT_PFN(CreatePassthroughLayerFB);
        INIT_PFN(DestroyPassthroughLayerFB);
        INIT_PFN(PassthroughLayerSetStyleFB);
        INIT_PFN(PassthroughLayerPauseFB);
        INIT_PFN(PassthroughLayerResumeFB);

#undef INIT_PFN
#undef CAT

        constexpr const XrPassthroughCreateInfoFB ptci {
            .type = XR_TYPE_PASSTHROUGH_CREATE_INFO_FB,
            .next = nullptr            
        };
        if (XR_FAILED(m_pfnCreatePassthroughFB(m_session, &ptci, &m_ptLayerData.passthrough))) {
            Log::Write(Log::Level::Error, "Failed to create passthrough object!");
            m_ptLayerData = {};
            return;
        }

        const XrPassthroughLayerCreateInfoFB plci {
            .type = XR_TYPE_PASSTHROUGH_LAYER_CREATE_INFO_FB,
            .next = nullptr,
            .passthrough = m_ptLayerData.passthrough,
            .purpose = XR_PASSTHROUGH_LAYER_PURPOSE_RECONSTRUCTION_FB
        };
        if (XR_FAILED(m_pfnCreatePassthroughLayerFB(m_session, &plci, &m_ptLayerData.reconPassthroughLayer))) {
            Log::Write(Log::Level::Error, "Failed to create passthrough layer!");
            m_ptLayerData = {};
            return;
        }
        CHECK(m_ptLayerData.reconPassthroughLayer != XR_NULL_HANDLE);

        Log::Write(Log::Level::Info, "Passthrough API is initialized.");
    }

    inline bool IsPassthroughSupported() const {
        return m_ptLayerData.reconPassthroughLayer != XR_NULL_HANDLE;
    }

    std::atomic<ALXR::PassthroughMode> m_currentPTMode{ ALXR::PassthroughMode::None };
    inline bool IsPassthroughModeEnabled() const {
        return m_currentPTMode.load() != ALXR::PassthroughMode::None;
    }

    void StartPassthroughMode() {
        if (!IsPassthroughSupported())
            return;
        CHECK_XRCMD(m_pfnPassthroughStartFB(m_ptLayerData.passthrough));
        CHECK_XRCMD(m_pfnPassthroughLayerResumeFB(m_ptLayerData.reconPassthroughLayer));
        Log::Write(Log::Level::Info, "Passthrough (Layer) is started/resumed.");

        constexpr const XrPassthroughStyleFB style {
            .type = XR_TYPE_PASSTHROUGH_STYLE_FB,
            .next = nullptr,
            .textureOpacityFactor = 0.5f,
            .edgeColor = { 0.0f, 0.0f, 0.0f, 0.0f },
        };
        CHECK_XRCMD(m_pfnPassthroughLayerSetStyleFB(m_ptLayerData.reconPassthroughLayer, &style));
    }

    void StopPassthroughMode() {
        if (!IsPassthroughModeEnabled())
            return;
        assert(IsPassthroughSupported());
        m_currentPTMode.store(ALXR::PassthroughMode::None);
    /////////////////////////////////////////////////
        Log::Write(Log::Level::Info, "Passthrough (Layer) is stopped/paused.");
        CHECK_XRCMD(m_pfnPassthroughLayerPauseFB(m_ptLayerData.reconPassthroughLayer));
        CHECK_XRCMD(m_pfnPassthroughPauseFB(m_ptLayerData.passthrough));
    }

    void TogglePassthroughMode(const ALXR::PassthroughMode newMode) {
        assert(IsPassthroughSupported());
        const auto lastMode = m_currentPTMode.load();
        if (newMode == lastMode) {
            StopPassthroughMode();
            return;
        }
        if (lastMode == ALXR::PassthroughMode::None) {
            StartPassthroughMode();
        }
        /////////////////////////////////////////////////
        m_currentPTMode.store(newMode);
    }

    void SetPerformanceLevels() {
        if (!IsSessionRunning())
            return;

        if (IsExtEnabled(XR_EXT_PERFORMANCE_SETTINGS_EXTENSION_NAME)) {
            PFN_xrPerfSettingsSetPerformanceLevelEXT setPerfLevel = nullptr;
            CHECK_XRCMD(xrGetInstanceProcAddr
            (
                m_instance,
                "xrPerfSettingsSetPerformanceLevelEXT",
                reinterpret_cast<PFN_xrVoidFunction*>(&setPerfLevel)
            ));
            CHECK(setPerfLevel != nullptr);
            CHECK_XRCMD(setPerfLevel(m_session, XR_PERF_SETTINGS_DOMAIN_CPU_EXT, XR_PERF_SETTINGS_LEVEL_SUSTAINED_HIGH_EXT));
            CHECK_XRCMD(setPerfLevel(m_session, XR_PERF_SETTINGS_DOMAIN_GPU_EXT, XR_PERF_SETTINGS_LEVEL_BOOST_EXT));
        }
#ifdef XR_USE_OXR_PICO
        if (IsExtEnabled(XR_PICO_PERFORMANCE_SETTINGS_EXTENSION_NAME)) {
            PFN_xrSetPerformanceLevelPICO setPerfLevel = nullptr;
            const XrResult result = xrGetInstanceProcAddr
            (
                m_instance,
                "xrSetPerformanceLevelPICO",
                reinterpret_cast<PFN_xrVoidFunction*>(&setPerfLevel)
            );
            if (XR_FAILED(result) || setPerfLevel == nullptr) {
                Log::Write(Log::Level::Warning, Fmt("Unable to load xr-extension function: %s, error-code: %d", "xrSetPerformanceLevelPICO", result));
                return;
            }
            if (XR_FAILED(setPerfLevel(m_session, XR_PERF_SETTINGS_DOMAIN_CPU_EXT, XR_PERF_SETTINGS_LEVEL_SUSTAINED_HIGH_EXT)))
                Log::Write(Log::Level::Warning, "Failed to set CPU performance level");
            if (XR_FAILED(setPerfLevel(m_session, XR_PERF_SETTINGS_DOMAIN_GPU_EXT, XR_PERF_SETTINGS_LEVEL_BOOST_EXT)))
                Log::Write(Log::Level::Warning, "Failed to set GPU performance level");
        }
#endif
    }

#ifdef XR_USE_PLATFORM_ANDROID
    inline bool SetAndroidAppThread(const pid_t threadId, const AndroidThreadType threadType) {
        if (!IsSessionRunning() ||
            !IsExtEnabled(XR_KHR_ANDROID_THREAD_SETTINGS_EXTENSION_NAME))
            return false;
        
        PFN_xrSetAndroidApplicationThreadKHR setAndroidAppThreadFn = nullptr;
        CHECK_XRCMD(xrGetInstanceProcAddr
        (
            m_instance,
            "xrSetAndroidApplicationThreadKHR",
            reinterpret_cast<PFN_xrVoidFunction*>(&setAndroidAppThreadFn)
        ));
        Log::Write(Log::Level::Info, Fmt("Setting android app thread, type: %lu, thread-id: %d", threadType, threadId));
        const auto result = setAndroidAppThreadFn(m_session, static_cast<XrAndroidThreadTypeKHR>(threadType), threadId);
        if (XR_FAILED(result)) {
            Log::Write(Log::Level::Info, Fmt("Failed setting android app thread, type: %lu, thread-id: %d, result-id: %lu", threadType, threadId, result));
        }
        return XR_SUCCEEDED(result);
    }

    virtual inline bool SetAndroidAppThread(const AndroidThreadType threadType) override {
        return SetAndroidAppThread(gettid(), threadType);
    }
#endif

    void CreateVisualizedSpaces() {
        CHECK(m_session != XR_NULL_HANDLE);
#ifdef ALXR_ENGINE_ENABLE_VIZ_SPACES
        constexpr const std::string_view visualizedSpaces[] = { "ViewFront", "Local", "Stage", "StageLeft", "StageRight", "StageLeftRotated", "StageRightRotated" };

        for (const auto& visualizedSpace : visualizedSpaces) {
            XrReferenceSpaceCreateInfo referenceSpaceCreateInfo = GetXrReferenceSpaceCreateInfo(visualizedSpace);
            XrSpace space;
            XrResult res = xrCreateReferenceSpace(m_session, &referenceSpaceCreateInfo, &space);
            if (XR_SUCCEEDED(res)) {
                m_visualizedSpaces.push_back(space);
                Log::Write(Log::Level::Info, Fmt("visualized-space %s added", visualizedSpace.data()));
            } else {
                Log::Write(Log::Level::Warning,
                           Fmt("Failed to create reference space %s with error %d", visualizedSpace.data(), res));
            }
        }
#endif
    }

    void InitializeSession() override {
        CHECK(m_instance != XR_NULL_HANDLE);
        CHECK(m_systemId != XR_NULL_SYSTEM_ID);
        CHECK(m_session == XR_NULL_HANDLE);
        {
            Log::Write(Log::Level::Verbose, Fmt("Creating session..."));

            const XrSessionCreateInfo createInfo{
                .type = XR_TYPE_SESSION_CREATE_INFO,
                .next = m_graphicsPlugin->GetGraphicsBinding(),
                .createFlags = 0,
                .systemId = m_systemId
            };
            CHECK_XRCMD(xrCreateSession(m_instance, &createInfo, &m_session));
            CHECK(m_session != XR_NULL_HANDLE);
        }
        
        InitializeExtensions();
        LogReferenceSpaces();
        InitializeActions();
        CreateVisualizedSpaces();

        {
            XrReferenceSpaceCreateInfo referenceSpaceCreateInfo = GetAppReferenceSpaceCreateInfo();
            CHECK_XRCMD(xrCreateReferenceSpace(m_session, &referenceSpaceCreateInfo, &m_appSpace));
            Log::Write(Log::Level::Verbose, Fmt("Selected app reference space: %s", to_string(referenceSpaceCreateInfo.referenceSpaceType)));
            m_streamConfig.trackingSpaceType = ToTrackingSpace(referenceSpaceCreateInfo.referenceSpaceType);

            referenceSpaceCreateInfo = GetXrReferenceSpaceCreateInfo("Stage");
            if (XR_FAILED(xrCreateReferenceSpace(m_session, &referenceSpaceCreateInfo, &m_boundingStageSpace)))
                m_boundingStageSpace = XR_NULL_HANDLE;

            referenceSpaceCreateInfo = GetXrReferenceSpaceCreateInfo("View");
            CHECK_XRCMD(xrCreateReferenceSpace(m_session, &referenceSpaceCreateInfo, &m_viewSpace));
        }
    }

    void ClearSwapchains()
    {
        m_swapchainImages.clear();
        if (const auto graphicsPlugin = m_graphicsPlugin) {
            graphicsPlugin->ClearSwapchainImageStructs();
        }
        for (const auto& swapchain : m_swapchains)
            xrDestroySwapchain(swapchain.handle);
        m_swapchains.clear();
        m_configViews.clear();
    }

    void CreateSwapchains(const std::uint32_t eyeWidth /*= 0*/, const std::uint32_t eyeHeight /*= 0*/) override {
        CHECK(m_session != XR_NULL_HANDLE);

        if (m_swapchains.size() > 0)
        {
            CHECK(m_configViews.size() > 0 && m_swapchainImages.size() > 0);
            if (eyeWidth == 0 || eyeHeight == 0)
                return;
            const bool isSameSize = std::all_of(m_configViews.begin(), m_configViews.end(), [&](const auto& vp)
            {
                const auto eW = std::min(eyeWidth,  vp.maxImageRectWidth);
                const auto eH = std::min(eyeHeight, vp.maxImageRectHeight);
                return eW == vp.recommendedImageRectWidth && eH == vp.recommendedImageRectHeight;
            });
            if (isSameSize)
                return;
            Log::Write(Log::Level::Info, "Clearing current swapchains...");
            ClearSwapchains();
            Log::Write(Log::Level::Info, "Creating new swapchains...");
        }
        CHECK(m_swapchainImages.empty());
        CHECK(m_swapchains.empty());
        CHECK(m_configViews.empty());

        // Read graphics properties for preferred swapchain length and logging.
        XrSystemProperties systemProperties{
            .type = XR_TYPE_SYSTEM_PROPERTIES,
            .next = nullptr
        };
        CHECK_XRCMD(xrGetSystemProperties(m_instance, m_systemId, &systemProperties));

        // Log system properties.
        Log::Write(Log::Level::Info,
                   Fmt("System Properties: Name=%s VendorId=%d", systemProperties.systemName, systemProperties.vendorId));
        Log::Write(Log::Level::Info, Fmt("System Graphics Properties: MaxWidth=%d MaxHeight=%d MaxLayers=%d",
                                         systemProperties.graphicsProperties.maxSwapchainImageWidth,
                                         systemProperties.graphicsProperties.maxSwapchainImageHeight,
                                         systemProperties.graphicsProperties.maxLayerCount));
        Log::Write(Log::Level::Info, Fmt("System Tracking Properties: OrientationTracking=%s PositionTracking=%s",
                                         systemProperties.trackingProperties.orientationTracking == XR_TRUE ? "True" : "False",
                                         systemProperties.trackingProperties.positionTracking == XR_TRUE ? "True" : "False"));

        // Note: No other view configurations exist at the time this code was written. If this
        // condition is not met, the project will need to be audited to see how support should be
        // added.
        CHECK_MSG(m_viewConfigType == XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, "Unsupported view configuration type");

        // Query and cache view configuration views.
        uint32_t viewCount = 0;
        CHECK_XRCMD(xrEnumerateViewConfigurationViews(m_instance, m_systemId, m_viewConfigType, 0, &viewCount, nullptr));
        CHECK(viewCount >= 2);
        m_configViews.resize(viewCount, {
            .type = XR_TYPE_VIEW_CONFIGURATION_VIEW,
            .next = nullptr
        });
        CHECK_XRCMD(xrEnumerateViewConfigurationViews(m_instance, m_systemId, m_viewConfigType, viewCount, &viewCount,
                                                      m_configViews.data()));

        // override recommended eye resolution
        if (eyeWidth != 0 && eyeHeight != 0) {
            for (auto& configView : m_configViews) {
                configView.recommendedImageRectWidth  = std::min(eyeWidth, configView.maxImageRectWidth);
                configView.recommendedImageRectHeight = std::min(eyeHeight, configView.maxImageRectHeight);
            }
        }

        // Create and cache view buffer for xrLocateViews later.
        m_views.resize(viewCount, IdentityView);
        if (viewCount < 2)
            return;

        // Create the swapchain and get the images.
        // 
        // Select a swapchain format.
        uint32_t swapchainFormatCount = 0;
        CHECK_XRCMD(xrEnumerateSwapchainFormats(m_session, 0, &swapchainFormatCount, nullptr));
        std::vector<int64_t> swapchainFormats(swapchainFormatCount);
        CHECK_XRCMD(xrEnumerateSwapchainFormats(m_session, (uint32_t)swapchainFormats.size(), &swapchainFormatCount,
                                                swapchainFormats.data()));
        CHECK(swapchainFormatCount == swapchainFormats.size());
        m_colorSwapchainFormat = m_graphicsPlugin->SelectColorSwapchainFormat(swapchainFormats);

        // Print swapchain formats and the selected one.
        {
            std::string swapchainFormatsString;
            for (int64_t format : swapchainFormats) {
                const bool selected = format == m_colorSwapchainFormat;
                swapchainFormatsString += " ";
                if (selected) {
                    swapchainFormatsString += "[";
                }
                swapchainFormatsString += std::to_string(format);
                if (selected) {
                    swapchainFormatsString += "]";
                }
            }
            Log::Write(Log::Level::Verbose, Fmt("Swapchain Formats: %s", swapchainFormatsString.c_str()));
        }

        if (m_isMultiViewEnabled)
        {
            CHECK(m_configViews[0].recommendedImageRectWidth ==
                  m_configViews[1].recommendedImageRectWidth);
            CHECK(m_configViews[0].recommendedImageRectHeight ==
                  m_configViews[1].recommendedImageRectHeight);
            CHECK(m_configViews[0].recommendedSwapchainSampleCount ==
                  m_configViews[1].recommendedSwapchainSampleCount);
            
            for (std::size_t i = 0; i < viewCount; ++i) {
                const XrViewConfigurationView& vp = m_configViews[i];
                Log::Write(Log::Level::Info, Fmt
                (
                    "Creating swapchain for view %d with dimensions Width=%d Height=%d SampleCount=%d", i,
                    vp.recommendedImageRectWidth, vp.recommendedImageRectHeight, vp.recommendedSwapchainSampleCount
                ));
            }

            const auto& vp = m_configViews[0];
            // Create the swapchain.
            const XrSwapchainCreateInfo swapchainCreateInfo{
                .type = XR_TYPE_SWAPCHAIN_CREATE_INFO,
                .next = nullptr,
                .createFlags = 0,
                .usageFlags = XR_SWAPCHAIN_USAGE_SAMPLED_BIT | XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT,
                .format = m_colorSwapchainFormat,
                .sampleCount = m_graphicsPlugin->GetSupportedSwapchainSampleCount(vp),
                .width = vp.recommendedImageRectWidth,
                .height = vp.recommendedImageRectHeight,
                .faceCount = 1,
                .arraySize = viewCount,
                .mipCount = 1,
            };
            Swapchain swapchain{
                .handle = XR_NULL_HANDLE,
                .width = static_cast<std::int32_t>(swapchainCreateInfo.width),
                .height = static_cast<std::int32_t>(swapchainCreateInfo.height)
            };
            CHECK_XRCMD(xrCreateSwapchain(m_session, &swapchainCreateInfo, &swapchain.handle));
            CHECK(swapchain.handle != XR_NULL_HANDLE);

            m_swapchains.push_back(swapchain);

            uint32_t imageCount = 0;
            CHECK_XRCMD(xrEnumerateSwapchainImages(swapchain.handle, 0, &imageCount, nullptr));
            // XXX This should really just return XrSwapchainImageBaseHeader*
            std::vector<XrSwapchainImageBaseHeader*> swapchainImages =
                m_graphicsPlugin->AllocateSwapchainImageStructs(imageCount, swapchainCreateInfo);
            CHECK_XRCMD(xrEnumerateSwapchainImages(swapchain.handle, imageCount, &imageCount, swapchainImages[0]));

            m_swapchainImages.insert(std::make_pair(swapchain.handle, std::move(swapchainImages)));
        }
        else
        {
            // Create a swapchain for each view.
            for (uint32_t i = 0; i < viewCount; i++) {
                const XrViewConfigurationView& vp = m_configViews[i];
                Log::Write(Log::Level::Info,
                    Fmt("Creating swapchain for view %d with dimensions Width=%d Height=%d SampleCount=%d", i,
                        vp.recommendedImageRectWidth, vp.recommendedImageRectHeight, vp.recommendedSwapchainSampleCount));

                // Create the swapchain.
                const XrSwapchainCreateInfo swapchainCreateInfo{
                    .type = XR_TYPE_SWAPCHAIN_CREATE_INFO,
                    .next = nullptr,
                    .usageFlags = XR_SWAPCHAIN_USAGE_SAMPLED_BIT | XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT,
                    .format = m_colorSwapchainFormat,
                    .sampleCount = m_graphicsPlugin->GetSupportedSwapchainSampleCount(vp),
                    .width = vp.recommendedImageRectWidth,
                    .height = vp.recommendedImageRectHeight,
                    .faceCount = 1,
                    .arraySize = 1,
                    .mipCount = 1,
                };
                Swapchain swapchain{
                    .handle = XR_NULL_HANDLE,
                    .width = static_cast<std::int32_t>(swapchainCreateInfo.width),
                    .height = static_cast<std::int32_t>(swapchainCreateInfo.height)
                };
                CHECK_XRCMD(xrCreateSwapchain(m_session, &swapchainCreateInfo, &swapchain.handle));
                CHECK(swapchain.handle != XR_NULL_HANDLE);

                m_swapchains.push_back(swapchain);

                uint32_t imageCount = 0;
                CHECK_XRCMD(xrEnumerateSwapchainImages(swapchain.handle, 0, &imageCount, nullptr));
                // XXX This should really just return XrSwapchainImageBaseHeader*
                std::vector<XrSwapchainImageBaseHeader*> swapchainImages =
                    m_graphicsPlugin->AllocateSwapchainImageStructs(imageCount, swapchainCreateInfo);
                CHECK_XRCMD(xrEnumerateSwapchainImages(swapchain.handle, imageCount, &imageCount, swapchainImages[0]));

                m_swapchainImages.insert(std::make_pair(swapchain.handle, std::move(swapchainImages)));
            }
        }
    }

    // Return event if one is available, otherwise return null.
    const XrEventDataBaseHeader* TryReadNextEvent() {
        // It is sufficient to clear the just the XrEventDataBuffer header to
        // XR_TYPE_EVENT_DATA_BUFFER
        XrEventDataBaseHeader* baseHeader = reinterpret_cast<XrEventDataBaseHeader*>(&m_eventDataBuffer);
        *baseHeader = {.type=XR_TYPE_EVENT_DATA_BUFFER, .next=nullptr};
        const XrResult xr = xrPollEvent(m_instance, &m_eventDataBuffer);
        if (xr == XR_SUCCESS) {
            if (baseHeader->type == XR_TYPE_EVENT_DATA_EVENTS_LOST) {
                const XrEventDataEventsLost* const eventsLost = reinterpret_cast<const XrEventDataEventsLost*>(baseHeader);
                Log::Write(Log::Level::Warning, Fmt("%d events lost", eventsLost));
            }
            return baseHeader;
        }
        if (xr == XR_EVENT_UNAVAILABLE) {
            return nullptr;
        }
        THROW_XR(xr, "xrPollEvent");
    }

    void PollEvents(bool* exitRenderLoop, bool* requestRestart) override {
        *exitRenderLoop = *requestRestart = false;

        PollStreamConfigEvents();

        // Process all pending messages.
        while (const XrEventDataBaseHeader* event = TryReadNextEvent()) {
            switch (event->type) {
                case XR_TYPE_EVENT_DATA_DISPLAY_REFRESH_RATE_CHANGED_FB: {
                    const auto& refreshRateChangedEvent = *reinterpret_cast<const XrEventDataDisplayRefreshRateChangedFB*>(event);
                    Log::Write(Log::Level::Info, Fmt("display refresh rate has changed from %f Hz to %f Hz", refreshRateChangedEvent.fromDisplayRefreshRate, refreshRateChangedEvent.toDisplayRefreshRate));
                    m_streamConfig.renderConfig.refreshRate = refreshRateChangedEvent.toDisplayRefreshRate;
                    break;
                }
                case XR_TYPE_EVENT_DATA_INSTANCE_LOSS_PENDING: {
                    const auto& instanceLossPending = *reinterpret_cast<const XrEventDataInstanceLossPending*>(event);
                    Log::Write(Log::Level::Warning, Fmt("XrEventDataInstanceLossPending by %lld", instanceLossPending.lossTime));
                    *exitRenderLoop = true;
                    *requestRestart = true;
                    return;
                }
                case XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED: {
                    const auto& sessionStateChangedEvent = *reinterpret_cast<const XrEventDataSessionStateChanged*>(event);
                    HandleSessionStateChangedEvent(sessionStateChangedEvent, exitRenderLoop, requestRestart);
                    break;
                }
                case XR_TYPE_EVENT_DATA_INTERACTION_PROFILE_CHANGED: {
                    m_interactionManager->SetActiveFromCurrentProfile();
                    m_interactionManager->LogActions();
                    break;
                }
                case XR_TYPE_EVENT_DATA_PERF_SETTINGS_EXT: {
                    const auto& perfSettingsEvent = *reinterpret_cast<const XrEventDataPerfSettingsEXT*>(event);
                    Log::Write(Log::Level::Info,
                        Fmt("PerfSettingsChanged: type %d subdomain %d : level %d -> level %d",
                            perfSettingsEvent.type,
                            perfSettingsEvent.subDomain,
                            perfSettingsEvent.fromLevel,
                            perfSettingsEvent.toLevel));
                } break;
                case XR_TYPE_EVENT_DATA_REFERENCE_SPACE_CHANGE_PENDING: {
                    const auto& spaceChangedEvent = *reinterpret_cast<const XrEventDataReferenceSpaceChangePending*>(event);
                    Log::Write(Log::Level::Verbose, Fmt("reference space: %d changing", spaceChangedEvent.referenceSpaceType));
                    const auto appRefSpace = ToXrReferenceSpaceType(m_streamConfig.trackingSpaceType);
                    if (spaceChangedEvent.referenceSpaceType == appRefSpace)
                        enqueueGuardianChanged(spaceChangedEvent.changeTime);
                }  break;
                default: {
                    Log::Write(Log::Level::Verbose, Fmt("Ignoring event type %d", event->type));
                    break;
                }
            }
        }
    }

    void HandleSessionStateChangedEvent(const XrEventDataSessionStateChanged& stateChangedEvent, bool* exitRenderLoop,
                                        bool* requestRestart) {
        const XrSessionState oldState = m_sessionState;
        m_sessionState = stateChangedEvent.state;

        Log::Write(Log::Level::Info, Fmt("XrEventDataSessionStateChanged: state %s->%s session=%lld time=%lld", to_string(oldState),
                                         to_string(m_sessionState), stateChangedEvent.session, stateChangedEvent.time));

        if ((stateChangedEvent.session != XR_NULL_HANDLE) && (stateChangedEvent.session != m_session)) {
            Log::Write(Log::Level::Error, "XrEventDataSessionStateChanged for unknown session");
            return;
        }

        switch (m_sessionState) {
            case XR_SESSION_STATE_SYNCHRONIZED: {
                m_delayOnGuardianChanged = true;
                break;
            }
            case XR_SESSION_STATE_READY: {
                CHECK(m_session != XR_NULL_HANDLE);
#ifdef XR_USE_OXR_PICO
                const XrSessionBeginInfoEXT sessionBeginInfoExt {
                    .type = XR_TYPE_SESSION_BEGIN_INFO,
                    .next = nullptr,
                    .enableSinglePass = m_isMultiViewEnabled
                };
#endif
                const XrSessionBeginInfo sessionBeginInfo{
                    .type = XR_TYPE_SESSION_BEGIN_INFO,
#ifdef XR_USE_OXR_PICO
                    .next = &sessionBeginInfoExt,
#else
                    .next = nullptr,
#endif
                    .primaryViewConfigurationType = m_viewConfigType
                };
                XrResult result;
                CHECK_XRCMD(result = xrBeginSession(m_session, &sessionBeginInfo));
                m_sessionRunning = (result == XR_SUCCESS);
                SetPerformanceLevels();
                SetAndroidAppThread(AndroidThreadType::AppMain);
                SetAndroidAppThread(AndroidThreadType::RendererMain);
                break;
            }
            case XR_SESSION_STATE_STOPPING: {
                CHECK(m_session != XR_NULL_HANDLE);
                StopPassthroughMode();
                CHECK_XRCMD(xrEndSession(m_session))
                m_sessionRunning = false;
                break;
            }
            case XR_SESSION_STATE_EXITING: {
                *exitRenderLoop = true;
                // Do not attempt to restart because user closed this session.
                *requestRestart = false;
                break;
            }
            case XR_SESSION_STATE_LOSS_PENDING: {
                *exitRenderLoop = true;
                // Poll for a new instance.
                *requestRestart = true;
                break;
            }
            default:
                break;
        }
    }

    bool IsSessionRunning() const override { return m_sessionRunning; }
    bool IsSessionFocused() const override { return m_sessionState == XR_SESSION_STATE_FOCUSED; }

    template < typename ControllerInfoArray >
    void PollHandTrackers(const XrTime time, ControllerInfoArray& controllerInfo)
    {
        if (m_pfnLocateHandJointsEXT == nullptr || time == 0)
            return;

        const bool isHandOnControllerPose = IsRuntime(OxrRuntimeType::HTCWave) ||
                                            IsRuntime(OxrRuntimeType::SteamVR) ||
                                            IsRuntime(OxrRuntimeType::WMR);
        std::array<XrMatrix4x4f, XR_HAND_JOINT_COUNT_EXT> oculusOrientedJointPoses;
        for (const auto hand : { Side::LEFT,Side::RIGHT })
        {
            auto& controller = controllerInfo[hand];//m_input.controllerInfo[hand];
            // TODO: v17/18 server does not allow for both controller & hand tracking data, this needs changing,
            //       we don't want to override a controller device pose with potentially an emulated pose for
            //       runtimes such as WMR & SteamVR.
            if (isHandOnControllerPose && controller.enabled)
                continue;

            auto& handerTracker = m_input.handerTrackers[hand];
            //XrHandJointVelocitiesEXT velocities {
            //    .type = XR_TYPE_HAND_JOINT_VELOCITIES_EXT,
            //    .next = nullptr,
            //    .jointCount = XR_HAND_JOINT_COUNT_EXT,
            //    .jointVelocities = handerTracker.jointVelocities.data(),
            //};
            XrHandJointLocationsEXT locations {
                .type = XR_TYPE_HAND_JOINT_LOCATIONS_EXT,
                .next = nullptr, //&velocities,
                .isActive = XR_FALSE,
                .jointCount = XR_HAND_JOINT_COUNT_EXT,
                .jointLocations = handerTracker.jointLocations.data(),
            };
            const XrHandJointsLocateInfoEXT locateInfo{
                .type = XR_TYPE_HAND_JOINTS_LOCATE_INFO_EXT,
                .next = nullptr,
                .baseSpace = m_appSpace,
                .time = time
            };
            CHECK_XRCMD(m_pfnLocateHandJointsEXT(handerTracker.tracker, &locateInfo, &locations));
            if (locations.isActive == XR_FALSE)
                continue;

            const auto& jointLocations = handerTracker.jointLocations;
            const auto& handBaseOrientation = handerTracker.baseOrientation;
            for (size_t jointIdx = 0; jointIdx < XR_HAND_JOINT_COUNT_EXT; ++jointIdx)
            {
                const auto& jointLoc = jointLocations[jointIdx];
                XrMatrix4x4f& jointMatFixed = oculusOrientedJointPoses[jointIdx];
                if (!Math::Pose::IsPoseValid(jointLoc))
                {
                    XrMatrix4x4f_CreateIdentity(&jointMatFixed);
                    continue;
                }
                const auto jointMat = Math::XrMatrix4x4f_CreateFromPose(jointLoc.pose);
                XrMatrix4x4f_CreateIdentity(&jointMatFixed);
                XrMatrix4x4f_Multiply(&jointMatFixed, &jointMat, &handBaseOrientation);
            }
            
            for (size_t boneIndex = 0; boneIndex < ALVR_HAND::alvrHandBone_MaxSkinnable; ++boneIndex)
            {
                auto& boneRot = controller.boneRotations[boneIndex];
                auto& bonePos = controller.bonePositionsBase[boneIndex];
                boneRot = { 0,0,0,1 };
                bonePos = { 0,0,0 };
                
                const auto xrJoint = ToXRHandJointType(static_cast<ALVR_HAND>(boneIndex));
                if (xrJoint == XR_HAND_JOINT_MAX_ENUM_EXT)
                    continue;

                const auto xrJointParent = GetJointParent(xrJoint);
                const XrMatrix4x4f& jointParentWorld = oculusOrientedJointPoses[xrJointParent];
                const XrMatrix4x4f& JointWorld       = oculusOrientedJointPoses[xrJoint];

                XrMatrix4x4f jointLocal, jointParentInv;
                XrMatrix4x4f_InvertRigidBody(&jointParentInv, &jointParentWorld);
                XrMatrix4x4f_Multiply(&jointLocal, &jointParentInv, &JointWorld);

                XrQuaternionf localizedRot;
                XrMatrix4x4f_GetRotation(&localizedRot, &jointLocal);
                XrVector3f localizedPos;
                XrMatrix4x4f_GetTranslation(&localizedPos, &jointLocal);

                boneRot = ToTrackingQuat(localizedRot);
                bonePos = ToTrackingVector3(localizedPos);
            }

            controller.enabled = true;
            controller.isHand = true;

            const XrMatrix4x4f& palmMatP = oculusOrientedJointPoses[XR_HAND_JOINT_PALM_EXT];
            XrQuaternionf palmRot;
            XrVector3f palmPos;
            XrMatrix4x4f_GetTranslation(&palmPos, &palmMatP);
            XrMatrix4x4f_GetRotation(&palmRot, &palmMatP);
            controller.boneRootPosition = ToTrackingVector3(palmPos);
            controller.boneRootOrientation = ToTrackingQuat(palmRot);
        }
    }

    void PollActions() override {
        using ControllerInfo = TrackingInfo::Controller;
        constexpr static const ControllerInfo ControllerIdentity {
            .enabled = false,
            .isHand = false,
            .buttons = 0,
            .trackpadPosition = { 0,0 },
            .triggerValue = 0.0f,
            .gripValue = 0.0f,
            .orientation = { 0,0,0,1 },
            .position = { 0,0,0 },
            .angularVelocity = { 0,0,0 },
            .linearVelocity = { 0,0,0 },
            .boneRotations = {},
            .bonePositionsBase = {},
            .boneRootOrientation = {0,0,0,1},
            .boneRootPosition = {0,0,0},
            .handFingerConfidences = 0
        };
        m_input.controllerInfo = { ControllerIdentity, ControllerIdentity };
        m_interactionManager->PollActions(m_input.controllerInfo);
    }

    inline bool UseNetworkPredicatedDisplayTime() const
    {
        return !IsRuntime(OxrRuntimeType::SteamVR) &&
               !IsRuntime(OxrRuntimeType::Monado);
    }

    void RenderFrame() override {
        CHECK(m_session != XR_NULL_HANDLE);
        const auto renderMode = m_renderMode.load();
        const bool isVideoStream = renderMode == RenderMode::VideoStream;
        std::uint64_t videoFrameDisplayTime = std::uint64_t(-1);
        if (isVideoStream) {
            m_graphicsPlugin->BeginVideoView();
            videoFrameDisplayTime = m_graphicsPlugin->GetVideoFrameIndex();
        }
        const bool timeRender = videoFrameDisplayTime != std::uint64_t(-1);
        // rendered1 appears to be unused.
        //if (timeRender)
        //    LatencyCollector::Instance().rendered1(videoFrameIndex);

        constexpr const XrFrameWaitInfo frameWaitInfo{
            .type = XR_TYPE_FRAME_WAIT_INFO,
            .next = nullptr
        };
        XrFrameState frameState{
            .type = XR_TYPE_FRAME_STATE,
            .next = nullptr
        };
        CHECK_XRCMD(xrWaitFrame(m_session, &frameWaitInfo, &frameState));
        m_PredicatedLatencyOffset.store(frameState.predictedDisplayPeriod);
        m_lastPredicatedDisplayTime.store(frameState.predictedDisplayTime);

        XrTime predictedDisplayTime;
        const auto predictedViews = GetPredicatedViews(frameState, renderMode, videoFrameDisplayTime, /*out*/ predictedDisplayTime);

        constexpr const XrFrameBeginInfo frameBeginInfo{
            .type = XR_TYPE_FRAME_BEGIN_INFO,
            .next = nullptr
        };
        CHECK_XRCMD(xrBeginFrame(m_session, &frameBeginInfo));

        XrCompositionLayerPassthroughFB passthroughLayer;
        XrCompositionLayerProjection    layer;
        std::uint32_t layerCount = 0;
        std::array<const XrCompositionLayerBaseHeader*, 2> layers{};
        std::array<XrCompositionLayerProjectionView,2> projectionLayerViews;
        if (frameState.shouldRender == XR_TRUE)
        {
            XrCompositionLayerFlags ptRenderLayerFlags = 0;
            const auto passthroughMode = m_currentPTMode.load();
            if (passthroughMode != ALXR::PassthroughMode::None &&
                m_ptLayerData.reconPassthroughLayer != XR_NULL_HANDLE) {
                passthroughLayer = {
                    .type = XR_TYPE_COMPOSITION_LAYER_PASSTHROUGH_FB,
                    .next = nullptr,
                    .flags = XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT,
                    .space = XR_NULL_HANDLE,
                    .layerHandle = m_ptLayerData.reconPassthroughLayer,
                };
                layers[layerCount++] = reinterpret_cast<const XrCompositionLayerBaseHeader*>(&passthroughLayer);
                ptRenderLayerFlags = XR_COMPOSITION_LAYER_UNPREMULTIPLIED_ALPHA_BIT;
            }
            const std::span<const XrView> views { predictedViews.begin(), predictedViews.end() };
            if (RenderLayer(predictedDisplayTime, views, projectionLayerViews, layer, passthroughMode)) {
                layer.layerFlags |= ptRenderLayerFlags;
                layers[layerCount++] = reinterpret_cast<const XrCompositionLayerBaseHeader*>(&layer);
            }
        }

        if (timeRender)
            LatencyCollector::Instance().rendered2(videoFrameDisplayTime);

#ifdef XR_USE_OXR_PICO
        const XrFrameEndInfoEXT xrFrameEndInfoEXT {
            .type = XR_TYPE_FRAME_END_INFO,
            .next = nullptr,
            .useHeadposeExt = 1,
            .gsIndex = m_gsIndex.load()
        };
#endif
        const XrFrameEndInfo frameEndInfo{
            .type = XR_TYPE_FRAME_END_INFO,
#ifdef XR_USE_OXR_PICO
            .next = &xrFrameEndInfoEXT,
#else
            .next = nullptr,
#endif
            // TODO: Figure out why steamvr doesn't like using custom predicated display times!!!
            .displayTime = UseNetworkPredicatedDisplayTime() ?
                predictedDisplayTime : frameState.predictedDisplayTime,
            //.displayTime = frameState.predictedDisplayTime,
            .environmentBlendMode = m_environmentBlendMode,
            .layerCount = layerCount,
            .layers = layers.data()
        };
        CHECK_XRCMD(xrEndFrame(m_session, &frameEndInfo));

        LatencyManager::Instance().SubmitAndSync(videoFrameDisplayTime);
        if (isVideoStream)
            m_graphicsPlugin->EndVideoView();
        
        if (m_delayOnGuardianChanged)
        {
            m_delayOnGuardianChanged = false;
            enqueueGuardianChanged();
        }
    }

    inline bool LocateViews(const XrTime predictedDisplayTime, const std::uint32_t viewCapacityInput, XrView* views) const
    {
#ifdef XR_USE_OXR_PICO
        XrViewStatePICOEXT xrViewStatePICOEXT {};
#endif
        const XrViewLocateInfo viewLocateInfo{
            .type = XR_TYPE_VIEW_LOCATE_INFO,
#ifdef XR_USE_OXR_PICO
            .next = &xrViewStatePICOEXT,
#else
            .next = nullptr,
#endif
            .viewConfigurationType = m_viewConfigType,
            .displayTime = predictedDisplayTime,
            .space = m_appSpace,
        };
        XrViewState viewState {
            .type = XR_TYPE_VIEW_STATE,
            .next = nullptr
        };
        uint32_t viewCountOutput = 0;
        const XrResult res = xrLocateViews(m_session, &viewLocateInfo, &viewState, viewCapacityInput, &viewCountOutput, views);
#ifdef XR_USE_OXR_PICO
        m_gsIndex.store(xrViewStatePICOEXT.gsIndex);
#endif
        CHECK_XRRESULT(res, "LocateViews");
        if ((viewState.viewStateFlags & XR_VIEW_STATE_POSITION_VALID_BIT) == 0 ||
            (viewState.viewStateFlags & XR_VIEW_STATE_ORIENTATION_VALID_BIT) == 0) {
            return false;  // There is no valid tracking poses for the views.
        }
        CHECK(viewCountOutput == viewCapacityInput);
#if 0
        CHECK(viewCountOutput == m_configViews.size());
        CHECK(viewCountOutput == m_swapchains.size());
#endif
        return true;
    }

    using VizCubeList = std::vector<Cube>;
    VizCubeList GetVisualizedCubes(const XrTime predictedDisplayTime) const {
        // For each locatable space that we want to visualize, render a 25cm cube.
        VizCubeList cubes;
#ifdef ALXR_ENGINE_ENABLE_VIZ_SPACES
        cubes.reserve(m_visualizedSpaces.size() + Side::COUNT);
        for (XrSpace visualizedSpace : m_visualizedSpaces) {
            XrSpaceLocation spaceLocation{ .type = XR_TYPE_SPACE_LOCATION, .next = nullptr };
            XrResult res = xrLocateSpace(visualizedSpace, m_appSpace, predictedDisplayTime, &spaceLocation);
            CHECK_XRRESULT(res, "xrLocateSpace");
            if (XR_UNQUALIFIED_SUCCESS(res)) {
                if ((spaceLocation.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) != 0 &&
                    (spaceLocation.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT) != 0) {
                    cubes.push_back(Cube{ spaceLocation.pose, {0.25f, 0.25f, 0.25f} });
                }
            }
            else {
                Log::Write(Log::Level::Verbose, Fmt("Unable to locate a visualized reference space in app space: %d", res));
            }
        }
#else
        cubes.reserve(Side::COUNT);
#endif
        constexpr const std::array<const float, Side::COUNT> HandScale = { {1.0f, 1.0f} };
        // Render a 10cm cube scaled by grabAction for each hand. Note renderHand will only be
        // true when the application has focus.
        for (const auto hand : { Side::LEFT, Side::RIGHT }) {
            const auto spaceLocation = GetHandSpaceLocation(hand, predictedDisplayTime, ALXR::InfinitySpaceLoc);
            if (!spaceLocation.is_infinity()) {
                const float scale = 0.1f * HandScale[hand];
                cubes.push_back(Cube{ spaceLocation.pose, {scale, scale, scale} });
            }
            else if (m_interactionManager->IsHandActive(hand)) {
                // Tracking loss is expected when the hand is not active so only log a message
                // if the hand is active.                    
                constexpr static const char* const handName[] = { "left", "right" };
                Log::Write(Log::Level::Verbose,
                    Fmt("Unable to locate %s hand action space in app space", handName[hand]));
            }
        }
        return cubes;
    }

    inline bool RenderLayer
    (
        const XrTime predictedDisplayTime,
        const std::span<const XrView>& views,
        std::array<XrCompositionLayerProjectionView, 2>& projectionLayerViews,
        XrCompositionLayerProjection& layer,
        const ALXR::PassthroughMode mode
    ) {
        if (m_isMultiViewEnabled)
            return RenderLayerMultiView
            (
                predictedDisplayTime, views, projectionLayerViews,
                layer, mode
            );
        else
            return RenderLayerSeperateViews
            (
                predictedDisplayTime, views, projectionLayerViews,
                layer, mode
            );
    }

    static inline std::uint32_t AcquireAndWaitForSwapchainImage(const Swapchain& swapChain) {
        std::uint32_t swapchainImageIndex = 0;
        constexpr const XrSwapchainImageAcquireInfo acquireInfo{
            .type = XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO,
            .next = nullptr
        };
        CHECK_XRCMD(xrAcquireSwapchainImage(swapChain.handle, &acquireInfo, &swapchainImageIndex));

        constexpr const XrSwapchainImageWaitInfo waitInfo{
            .type = XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO,
            .next = nullptr,
            .timeout = XR_INFINITE_DURATION
        };
        CHECK_XRCMD(xrWaitSwapchainImage(swapChain.handle, &waitInfo));
        return swapchainImageIndex;
    }

    constexpr static const XrCompositionLayerFlags RenderLayerFlags =
        XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT |
        XR_COMPOSITION_LAYER_CORRECT_CHROMATIC_ABERRATION_BIT;

    bool RenderLayerMultiView
    (
        const XrTime predictedDisplayTime,
        const std::span<const XrView>& views,
        std::array<XrCompositionLayerProjectionView, 2>& projectionLayerViews,
        XrCompositionLayerProjection& layer,
        const ALXR::PassthroughMode mode
    )
    {
        assert(projectionLayerViews.size() == views.size());
        assert(m_isMultiViewEnabled);

        const bool isVideoStream = m_renderMode == RenderMode::VideoStream;
        const auto vizCubes = isVideoStream ? VizCubeList{} : GetVisualizedCubes(predictedDisplayTime);
        const auto ptMode = static_cast<const ::PassthroughMode>(mode);

        const Swapchain& viewSwapchain = m_swapchains[0];
        const XrRect2Di imageRect {
            .offset = {0, 0},
            .extent = {viewSwapchain.width, viewSwapchain.height}
        };

        const std::uint32_t swapchainImageIndex = AcquireAndWaitForSwapchainImage(viewSwapchain);
        for (std::uint32_t viewIndex = 0; viewIndex < views.size(); ++viewIndex) {
            const auto& view = views[viewIndex];
            projectionLayerViews[viewIndex] = {
                .type = XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW,
                .next = nullptr,
                .pose = view.pose,
                .fov = view.fov,
                .subImage = {
                    .swapchain = viewSwapchain.handle,
                    .imageRect = imageRect,
                    .imageArrayIndex = viewIndex
                }
            };
        }

        const XrSwapchainImageBaseHeader* const swapchainImage = m_swapchainImages[viewSwapchain.handle][swapchainImageIndex];
        if (isVideoStream)
            m_graphicsPlugin->RenderVideoMultiView(projectionLayerViews, swapchainImage, m_colorSwapchainFormat, ptMode);
        else
            m_graphicsPlugin->RenderMultiView(projectionLayerViews, swapchainImage, m_colorSwapchainFormat, ptMode, vizCubes);

        constexpr const XrSwapchainImageReleaseInfo releaseInfo{
            .type = XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO,
            .next = nullptr
        };
        CHECK_XRCMD(xrReleaseSwapchainImage(viewSwapchain.handle, &releaseInfo));

        layer = XrCompositionLayerProjection{
            .type = XR_TYPE_COMPOSITION_LAYER_PROJECTION,
            .next = nullptr,
            .layerFlags = RenderLayerFlags,
            .space = m_appSpace,
            .viewCount = (uint32_t)projectionLayerViews.size(),
            .views = projectionLayerViews.data()
        };
        return true;
    }

    bool RenderLayerSeperateViews
    (
        const XrTime predictedDisplayTime,
        const std::span<const XrView>& views,
        std::array<XrCompositionLayerProjectionView,2>& projectionLayerViews,
        XrCompositionLayerProjection& layer,
        const ALXR::PassthroughMode mode
    )
    {
        assert(projectionLayerViews.size() == views.size());

        const bool isVideoStream = m_renderMode == RenderMode::VideoStream;
        const auto vizCubes = isVideoStream ? VizCubeList{} : GetVisualizedCubes(predictedDisplayTime);
        const auto ptMode = static_cast<const ::PassthroughMode>(mode);
        // Render view to the appropriate part of the swapchain image.
        for (std::uint32_t i = 0; i < views.size(); ++i) {
            // Each view has a separate swapchain which is acquired, rendered to, and released.
            const Swapchain& viewSwapchain = m_swapchains[i];
            const std::uint32_t swapchainImageIndex = AcquireAndWaitForSwapchainImage(viewSwapchain);

            const auto& view = views[i];
            projectionLayerViews[i] = {
                .type = XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW,
                .next = nullptr,
                .pose = view.pose,
                .fov  = view.fov,
                .subImage = {
                    .swapchain = viewSwapchain.handle,
                    .imageRect = {
                        .offset = {0, 0},
                        .extent = {viewSwapchain.width, viewSwapchain.height}
                    },
                    .imageArrayIndex = 0
                }
            };
            const XrSwapchainImageBaseHeader* const swapchainImage = m_swapchainImages[viewSwapchain.handle][swapchainImageIndex];
            if (isVideoStream)
                m_graphicsPlugin->RenderVideoView(i, projectionLayerViews[i], swapchainImage, m_colorSwapchainFormat, ptMode);
            else
                m_graphicsPlugin->RenderView(projectionLayerViews[i], swapchainImage, m_colorSwapchainFormat, ptMode, vizCubes);
            
            constexpr const XrSwapchainImageReleaseInfo releaseInfo{
                .type = XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO,
                .next = nullptr
            };
            CHECK_XRCMD(xrReleaseSwapchainImage(viewSwapchain.handle, &releaseInfo));
        }

        layer = XrCompositionLayerProjection {
            .type = XR_TYPE_COMPOSITION_LAYER_PROJECTION,
            .next = nullptr,
            .layerFlags = RenderLayerFlags,
            .space = m_appSpace,
            .viewCount = (uint32_t)projectionLayerViews.size(),
            .views = projectionLayerViews.data()
        };
        return true;
    }

    virtual void SetRenderMode(const RenderMode newMode) override
    {
        m_renderMode = newMode;
    }

    virtual RenderMode GetRenderMode() const override {
        return m_renderMode;
    }

    float EstimateDisplayRefreshRate()
    {
#ifdef ALXR_ENABLE_ESTIMATE_DISPLAY_REFRESH_RATE
        if (m_session == XR_NULL_HANDLE)
            return 60.0f;

        using ClockType = XrSteadyClock;
        static_assert(ClockType::is_steady);
        using secondsf = std::chrono::duration<float, std::chrono::seconds::period>;

        using namespace std::literals::chrono_literals;
        constexpr const auto OneSecond = 1s;
        constexpr const size_t SamplesPerSec = 30;

        std::vector<size_t> frame_count_per_sec;
        frame_count_per_sec.reserve(SamplesPerSec);

        bool isStarted = false;
        size_t frameIdx = 0;
        auto last = XrSteadyClock::now();
        while (frame_count_per_sec.size() != SamplesPerSec) {
            bool exitRenderLoop = false, requestRestart = false;
            PollEvents(&exitRenderLoop, &requestRestart);
            if (exitRenderLoop)
                break;
            if (!IsSessionRunning())
                continue;
            if (!isStarted)
            {
                last = ClockType::now();
                isStarted = true;
            }
            XrFrameState frameState{ .type=XR_TYPE_FRAME_STATE, .next=nullptr };
            CHECK_XRCMD(xrWaitFrame(m_session, nullptr, &frameState));            
            CHECK_XRCMD(xrBeginFrame(m_session, nullptr));
            const XrFrameEndInfo frameEndInfo{
                .type = XR_TYPE_FRAME_END_INFO,
                .next = nullptr,
                .displayTime = frameState.predictedDisplayTime,
                .environmentBlendMode = m_environmentBlendMode,
                .layerCount = 0,
                .layers = nullptr
            };
            CHECK_XRCMD(xrEndFrame(m_session, &frameEndInfo));

            if (!frameState.shouldRender)
                continue;
            
            ++frameIdx;
            auto curr = ClockType::now();
            if ((curr - last) >= OneSecond)
            {
                Log::Write(Log::Level::Info, Fmt("Frame Count at %d = %d frames", frame_count_per_sec.size(), frameIdx));
                frame_count_per_sec.push_back(frameIdx);
                last = curr;
                frameIdx = 0;              
            }
        }

        const float dom = static_cast<float>(std::accumulate(frame_count_per_sec.begin(), frame_count_per_sec.end(), size_t(0)));
        const float result = dom == 0 ? 60.0f : (dom / static_cast<float>(SamplesPerSec));
        Log::Write(Log::Level::Info, Fmt("Estimated display refresh rate: %f Hz", result));
        return result;
#else
        return 90.0f;
#endif
    }

    void UpdateSupportedDisplayRefreshRates()
    {
        if (m_pfnGetDisplayRefreshRateFB) {
            CHECK_XRCMD(m_pfnGetDisplayRefreshRateFB(m_session, &m_streamConfig.renderConfig.refreshRate));
        }

        if (m_pfnEnumerateDisplayRefreshRatesFB) {
            std::uint32_t size = 0;
            CHECK_XRCMD(m_pfnEnumerateDisplayRefreshRatesFB(m_session, 0, &size, nullptr));
            m_displayRefreshRates.resize(size);
            CHECK_XRCMD(m_pfnEnumerateDisplayRefreshRatesFB(m_session, size, &size, m_displayRefreshRates.data()));
            return;
        }
        // If OpenXR runtime does not support XR_FB_display_refresh_rate extension
        // and currently core spec has no method of query the supported refresh rates
        // the only way to determine this is with a dumy loop
        m_displayRefreshRates = 
#ifdef ALXR_ENABLE_ESTIMATE_DISPLAY_REFRESH_RATE
        m_displayRefreshRates = { EstimateDisplayRefreshRate() };
#else
        m_displayRefreshRates = { 60.0f, 72.0f, 80.0f, 90.0f, 120.0f, 144.0f };
#endif
        assert(m_displayRefreshRates.size() > 0);
    }

    virtual bool GetSystemProperties(ALXRSystemProperties& systemProps) const override
    {
        if (m_instance == XR_NULL_HANDLE)
            return false;
        XrSystemProperties xrSystemProps = { .type=XR_TYPE_SYSTEM_PROPERTIES, .next=nullptr };
        CHECK_XRCMD(xrGetSystemProperties(m_instance, m_systemId, &xrSystemProps));
        std::strncpy(systemProps.systemName, xrSystemProps.systemName, sizeof(systemProps.systemName));
        if (m_configViews.size() > 0)
        {
            const auto& configView = m_configViews[0];
            systemProps.recommendedEyeWidth = configView.recommendedImageRectWidth;// / 2;
            systemProps.recommendedEyeHeight = configView.recommendedImageRectHeight;
        }
        assert(m_displayRefreshRates.size() > 0);
        systemProps.refreshRates = m_displayRefreshRates.data();
        systemProps.refreshRatesCount = static_cast<std::uint32_t>(m_displayRefreshRates.size());
        systemProps.currentRefreshRate = m_displayRefreshRates.back();
        if (m_pfnGetDisplayRefreshRateFB)
            CHECK_XRCMD(m_pfnGetDisplayRefreshRateFB(m_session, &systemProps.currentRefreshRate));
        return true;
    }

    inline ALXR::SpaceLoc GetSpaceLocation(const XrSpace& targetSpace, const XrSpace& baseSpace, const ALXR::SpaceLoc& initLoc = ALXR::IdentitySpaceLoc) const
    {
        return ALXR::GetSpaceLocation(targetSpace, baseSpace, m_lastPredicatedDisplayTime, initLoc);
    }

    inline ALXR::SpaceLoc GetSpaceLocation(const XrSpace& targetSpace, const XrTime& time, const ALXR::SpaceLoc& initLoc = ALXR::IdentitySpaceLoc) const
    {
        return ALXR::GetSpaceLocation(targetSpace, m_appSpace, time, initLoc);
    }

    inline ALXR::SpaceLoc GetSpaceLocation(const XrSpace& targetSpace, const ALXR::SpaceLoc& initLoc = ALXR::IdentitySpaceLoc) const
    {
        return GetSpaceLocation(targetSpace, m_lastPredicatedDisplayTime, initLoc);
    }

    inline ALXR::SpaceLoc GetHandSpaceLocation(const std::size_t hand, const XrSpace& baseSpace, const ALXR::SpaceLoc& initLoc = ALXR::IdentitySpaceLoc) const
    {
        assert(m_interactionManager != nullptr);
        return m_interactionManager->GetSpaceLocation(hand, baseSpace, m_lastPredicatedDisplayTime, initLoc);
    }

    inline ALXR::SpaceLoc GetHandSpaceLocation(const std::size_t hand, const XrTime& time, const ALXR::SpaceLoc& initLoc = ALXR::IdentitySpaceLoc) const
    {
        assert(m_interactionManager != nullptr);
        return m_interactionManager->GetSpaceLocation(hand, m_appSpace, time, initLoc);
    }

    inline ALXR::SpaceLoc GetHandSpaceLocation(const std::size_t hand, const ALXR::SpaceLoc& initLoc = ALXR::IdentitySpaceLoc) const
    {
        return GetHandSpaceLocation(hand, m_lastPredicatedDisplayTime, initLoc);
    }

    inline std::array<XrView,2> GetPredicatedViews
    (
        const XrFrameState& frameState, const RenderMode renderMode, const std::uint64_t videoTimeStampNs,
        XrTime& predicateDisplayTime
    )
    {
        assert(frameState.predictedDisplayPeriod >= 0);
        const auto GetDefaultViews = [&]()-> std::array<XrView, 2> {
            LocateViews(frameState.predictedDisplayTime, (uint32_t)m_views.size(), m_views.data());
            return { m_views[0], m_views[1] };
        };
        predicateDisplayTime = frameState.predictedDisplayTime;
        if (renderMode == RenderMode::Lobby)
            return GetDefaultViews();

        std::shared_lock<std::shared_mutex> l(m_trackingFrameMapMutex);
        if (videoTimeStampNs != std::uint64_t(-1))
        {
            const auto trackingFrameItr = m_trackingFrameMap.find(videoTimeStampNs);
            if (trackingFrameItr != m_trackingFrameMap.cend()) {
                predicateDisplayTime = trackingFrameItr->second.displayTime;
                return trackingFrameItr->second.views;
            }
        }
        const auto result = m_trackingFrameMap.rbegin();
        if (result == m_trackingFrameMap.rend())
            return GetDefaultViews();
        predicateDisplayTime = result->second.displayTime;
        return result->second.views;
    }

    static inline ALXREyeInfo GetEyeInfo(const XrView& left_view, const XrView& right_view)
    {
        XrVector3f v;
        XrVector3f_Sub(&v, &right_view.pose.position, &left_view.pose.position);
        float ipd = std::fabs(XrVector3f_Length(&v));
        if (ipd < 0.00001f)
            ipd = 0.063f;
        constexpr const auto ToEyeFov = [](const XrFovf& fov) -> EyeFov
        {
            return EyeFov{
                .left   = fov.angleLeft,
                .right  = fov.angleRight,
                .top    = fov.angleUp,
                .bottom = fov.angleDown
            };
        };
        return ALXREyeInfo{
            .eyeFov = {
                ToEyeFov(left_view.fov),
                ToEyeFov(right_view.fov)
            },
            .ipd = ipd
        };
    }

    static inline ALXREyeInfo GetEyeInfo(const std::array<XrView, 2>& views)
    {
        return GetEyeInfo(views[0], views[1]);
    }

    virtual inline bool GetEyeInfo(ALXREyeInfo& eyeInfo, const XrTime& time) const override
    {
        std::array<XrView, 2> newViews{ IdentityView, IdentityView };
        LocateViews(time, static_cast<const std::uint32_t>(newViews.size()), newViews.data());
        eyeInfo = GetEyeInfo(newViews[0], newViews[1]);
        return true;
    }

    virtual inline bool GetEyeInfo(ALXREyeInfo& eyeInfo) const override
    {
        return GetEyeInfo(eyeInfo, m_lastPredicatedDisplayTime);
    }

    virtual bool GetTrackingInfo(TrackingInfo& info) /*const*/ override
    {
        const XrDuration predicatedLatencyOffsetNs = m_PredicatedLatencyOffset.load();
        info = {
            .mounted = true,
            .controller = { m_input.controllerInfo[0], m_input.controllerInfo[1] }
        };
        assert(predicatedLatencyOffsetNs >= 0);
        
        const auto trackingPredictionLatencyUs = LatencyCollector::Instance().getTrackingPredictionLatency();
        const auto [xrTimeStamp, timeStampUs] = XrTimeNow();
        assert(timeStampUs != std::uint64_t(-1) && xrTimeStamp >= 0);

        const XrDuration totalLatencyOffsetNs = static_cast<XrDuration>(trackingPredictionLatencyUs * 1000) + predicatedLatencyOffsetNs;
        const auto predicatedDisplayTimeXR = xrTimeStamp + totalLatencyOffsetNs;
        const auto predicatedDisplayTimeNs = (timeStampUs * 1000) + static_cast<std::uint64_t>(totalLatencyOffsetNs);
        
        std::array<XrView, 2> newViews { IdentityView, IdentityView };
        LocateViews(predicatedDisplayTimeXR, (const std::uint32_t)newViews.size(), newViews.data());
         {
             std::unique_lock<std::shared_mutex> lock(m_trackingFrameMapMutex);
             m_trackingFrameMap[predicatedDisplayTimeNs] = {
                 .views       = newViews,
                 //.timestamp   = predicatedDisplayTimeNs,
                 .displayTime = predicatedDisplayTimeXR
             };
             if (m_trackingFrameMap.size() > MaxTrackingFrameCount)
                 m_trackingFrameMap.erase(m_trackingFrameMap.begin());
         }
        info.targetTimestampNs = predicatedDisplayTimeNs;
        
        const auto hmdSpaceLoc = GetSpaceLocation(m_viewSpace, predicatedDisplayTimeXR);
        info.HeadPose_Pose_Orientation  = ToTrackingQuat(hmdSpaceLoc.pose.orientation);
        info.HeadPose_Pose_Position     = ToTrackingVector3(hmdSpaceLoc.pose.position);
        // info.HeadPose_LinearVelocity    = ToTrackingVector3(hmdSpaceLoc.linearVelocity);
        // info.HeadPose_AngularVelocity   = ToTrackingVector3(hmdSpaceLoc.angularVelocity);

        for (const auto hand : { Side::LEFT, Side::RIGHT }) {
            auto& newContInfo = info.controller[hand];
#ifdef XR_USE_OXR_PICO
            // 
            // As of writing, there are bugs in Pico's OXR runtime with either/both:
            //      * xrLocateSpace for controller action spaces not working with any other times beyond XrFrameState::predicateDisplayTime (and zero, in a non-conforming way).
            //      * xrConvertTimeToTimespecTimeKHR appears to return values in microseconds instead of nanoseconds and values seem to be completely of from what
            //        XrFrameState::predicateDisplayTime values are.   
            //
            //  This workaround will induce some small amount of "lag" as the times don't account for network latency and the HMD poses being in future times.
            //
            const auto spaceLoc = GetHandSpaceLocation(hand, m_lastPredicatedDisplayTime);
#else
            const auto spaceLoc = GetHandSpaceLocation(hand, predicatedDisplayTimeXR);
#endif
            newContInfo.position        = ToTrackingVector3(spaceLoc.pose.position);
            newContInfo.orientation     = ToTrackingQuat(spaceLoc.pose.orientation);
            newContInfo.linearVelocity  = ToTrackingVector3(spaceLoc.linearVelocity);
            newContInfo.angularVelocity = ToTrackingVector3(spaceLoc.angularVelocity);
        }

        PollHandTrackers(predicatedDisplayTimeXR, info.controller);

        LatencyCollector::Instance().tracking(predicatedDisplayTimeNs);
        return true;
    }

    virtual inline void EnqueueHapticFeedback(const ALXR::HapticsFeedback& hapticFeedback) override
    {
        assert(m_interactionManager != nullptr);
        m_interactionManager->EnqueueHapticFeedback(hapticFeedback);
    }

    virtual inline void SetStreamConfig(const ALXRStreamConfig& config) override
    {
        m_streamConfigQueue.push(config);
    }

    virtual inline bool GetStreamConfig(ALXRStreamConfig& config) const override
    {
        // TODO: Check for thread sync!
        config = m_streamConfig;
        return true;
    }

    void PollStreamConfigEvents()
    {
        ALXRStreamConfig newConfig;
        if (!m_streamConfigQueue.try_pop(newConfig))
            return;

        if (newConfig.trackingSpaceType != m_streamConfig.trackingSpaceType) {
            const auto IsRefSpaceTypeSupported = [this](const ALXRTrackingSpace ts) {
                const auto xrSpaceRefType = ToXrReferenceSpaceType(ts);
                const auto availSpaces = GetAvailableReferenceSpaces();
                return std::find(availSpaces.begin(), availSpaces.end(), xrSpaceRefType) != availSpaces.end();
            };
            if (IsRefSpaceTypeSupported(newConfig.trackingSpaceType)) {
                if (m_appSpace != XR_NULL_HANDLE) {
                    xrDestroySpace(m_appSpace);
                    m_appSpace = XR_NULL_HANDLE;
                }
                const auto oldTrackingSpaceName = ToTrackingSpaceName(m_streamConfig.trackingSpaceType);
                const auto newTrackingSpaceName = ToTrackingSpaceName(newConfig.trackingSpaceType);
                Log::Write(Log::Level::Info, Fmt("Changing tracking space from %s to %s", oldTrackingSpaceName, newTrackingSpaceName));
                XrReferenceSpaceCreateInfo referenceSpaceCreateInfo = GetXrReferenceSpaceCreateInfo(newTrackingSpaceName);
                CHECK_XRCMD(xrCreateReferenceSpace(m_session, &referenceSpaceCreateInfo, &m_appSpace));

                m_streamConfig.trackingSpaceType = newConfig.trackingSpaceType;
            }
            else {
                Log::Write(Log::Level::Warning, Fmt("Tracking space %s is not supported, tracking space is not changed.", ToTrackingSpaceName(newConfig.trackingSpaceType)));
            }
        }

        auto& currRenderConfig = m_streamConfig.renderConfig;
        const auto& newRenderConfig = newConfig.renderConfig;        
        if (newRenderConfig.refreshRate != currRenderConfig.refreshRate) {
            [&]() {
                if (m_pfnRequestDisplayRefreshRateFB == nullptr) {
                    Log::Write(Log::Level::Warning, "This OpenXR runtime does not support setting the display refresh rate.");
                    return;
                }

                const auto itr = std::find(m_displayRefreshRates.begin(), m_displayRefreshRates.end(), newRenderConfig.refreshRate);
                if (itr == m_displayRefreshRates.end()) {
                    Log::Write(Log::Level::Warning, Fmt("Selected new refresh rate %f Hz is not supported, no change has been made.", newRenderConfig.refreshRate));
                    return;
                }

                Log::Write(Log::Level::Info, Fmt("Setting display refresh rate from %f Hz to %f Hz.", currRenderConfig.refreshRate, newRenderConfig.refreshRate));
                CHECK_XRCMD(m_pfnRequestDisplayRefreshRateFB(m_session, newRenderConfig.refreshRate));
                currRenderConfig.refreshRate = newRenderConfig.refreshRate;
            }();
        }

        //m_streamConfig = newConfig;
    }

    virtual inline void RequestExitSession() override
    {
        if (m_session == XR_NULL_HANDLE)
            return;
        CHECK_XRCMD(xrRequestExitSession(m_session));
    }

    virtual inline bool GetGuardianData(ALXRGuardianData& gd) /*const*/ override
    {
        gd.shouldSync = false;
        return m_guardianChangedQueue.try_pop(gd);
    }

    inline bool GetBoundingStageSpace(const XrTime& time, ALXR::SpaceLoc& space, XrExtent2Df& boundingArea) const
    {
        if (m_session == XR_NULL_HANDLE ||
            m_boundingStageSpace == XR_NULL_HANDLE)
            return false;
        if (XR_FAILED(xrGetReferenceSpaceBoundsRect(m_session, XR_REFERENCE_SPACE_TYPE_STAGE, &boundingArea)))
        {
            Log::Write(Log::Level::Info, "xrGetReferenceSpaceBoundsRect FAILED.");
            return false;
        }
        space = GetSpaceLocation(m_boundingStageSpace, time, ALXR::InfinitySpaceLoc);
        return !space.is_infinity();
    }

    inline bool GetBoundingStageSpace(const XrTime& time, ALXRGuardianData& gd) const
    {
        ALXR::SpaceLoc loc;
        XrExtent2Df boundingArea;
        if (!GetBoundingStageSpace(time, loc, boundingArea))
            return false;
        gd = {
            .shouldSync = true,
            .areaWidth = boundingArea.width,
            .areaHeight = boundingArea.height
        };
        return true;
    }

    inline bool enqueueGuardianChanged(const XrTime& time)
    {
        Log::Write(Log::Level::Verbose, "Enqueuing guardian changed");
        ALXRGuardianData gd {
            .shouldSync = false
        };
        if (!GetBoundingStageSpace(time, gd))
            return false;
        Log::Write(Log::Level::Verbose, "Guardian changed enqueud successfully.");
        m_guardianChangedQueue.push(gd);
        return true;
    }

    bool enqueueGuardianChanged() {
        return enqueueGuardianChanged(m_lastPredicatedDisplayTime);
    }

#ifdef XR_USE_OXR_PICO
    enum PxrHmdDof : int
    {
        PXR_HMD_3DOF = 0,
        PXR_HMD_6DOF
    };
    enum PxrControllerDof : int
    {
        PXR_CONTROLLER_3DOF = 0,
        PXR_CONTROLLER_6DOF
    };
#endif
    virtual inline void Resume() override
    {
#ifdef XR_USE_OXR_PICO
        if (m_instance == XR_NULL_HANDLE) {
            Log::Write(Log::Level::Warning, "OpenXrProgram::Resume invoked but an openxr instance not yet set.");
            return;
        }
        if (m_pfnSetEngineVersionPico) {
            Log::Write(Log::Level::Info, "Setting pico engine version to 2.8.0.1");
            m_pfnSetEngineVersionPico(m_instance, "2.8.0.1");
        }
        if (m_pfnStartCVControllerThreadPico) {
            Log::Write(Log::Level::Info, "Starting pico cv controller thread");
            m_pfnStartCVControllerThreadPico(m_instance, PXR_HMD_6DOF, PXR_CONTROLLER_6DOF);
        }
#endif
    }

    virtual inline void Pause() override
    {
#ifdef XR_USE_OXR_PICO
        if (m_instance == XR_NULL_HANDLE) {
            Log::Write(Log::Level::Warning, "OpenXrProgram::Paused invoked but an openxr instance not yet set.");
            return;
        }
        if (m_pfnSetEngineVersionPico) {
            Log::Write(Log::Level::Info, "Setting pico engine version to 2.7.0.0");
            m_pfnSetEngineVersionPico(m_instance, "2.7.0.0");
        }
        if (m_pfnStopCVControllerThreadPico) {
            Log::Write(Log::Level::Info, "Stopping pico cv controller thread");
            m_pfnStopCVControllerThreadPico(m_instance, PXR_HMD_6DOF, PXR_CONTROLLER_6DOF);
        }
#endif
    }

    virtual inline std::shared_ptr<const IGraphicsPlugin> GetGraphicsPlugin() const override {
        return m_graphicsPlugin;
    }
    virtual inline std::shared_ptr<IGraphicsPlugin> GetGraphicsPlugin() override {
        return m_graphicsPlugin;
    }

   private:
    const std::shared_ptr<Options> m_options;
    std::shared_ptr<IPlatformPlugin> m_platformPlugin;
    std::shared_ptr<IGraphicsPlugin> m_graphicsPlugin;
    XrInstance m_instance{XR_NULL_HANDLE};
    XrSession m_session{XR_NULL_HANDLE};
    XrSpace m_appSpace{XR_NULL_HANDLE};
    XrSpace m_boundingStageSpace{ XR_NULL_HANDLE };
    XrSpace m_viewSpace{ XR_NULL_HANDLE };
    XrFormFactor m_formFactor{XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY};
    XrViewConfigurationType m_viewConfigType{XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO};
    XrEnvironmentBlendMode m_environmentBlendMode{ XR_ENVIRONMENT_BLEND_MODE_OPAQUE };
    XrSystemId m_systemId{XR_NULL_SYSTEM_ID};

    std::vector<XrViewConfigurationView> m_configViews;
    std::vector<Swapchain> m_swapchains;
    std::map<XrSwapchain, std::vector<XrSwapchainImageBaseHeader*>> m_swapchainImages;
    std::vector<XrView> m_views;
    int64_t m_colorSwapchainFormat{-1};
    std::atomic<RenderMode> m_renderMode{ RenderMode::Lobby };

    std::vector<XrSpace> m_visualizedSpaces;

    // Application's current lifecycle state according to the runtime
    XrSessionState m_sessionState{XR_SESSION_STATE_UNKNOWN};
    std::atomic<bool> m_sessionRunning{false};
    OxrRuntimeType m_runtimeType { OxrRuntimeType::Unknown };

    XrEventDataBuffer m_eventDataBuffer{
        .type = XR_TYPE_EVENT_DATA_BUFFER,
        .next = nullptr
    };
    ALXR::ALXRPaths  m_alxrPaths;
    
    using InteractionManagerPtr = std::unique_ptr<ALXR::InteractionManager>;
    InteractionManagerPtr m_interactionManager{ nullptr };
    struct InputState
    {
        struct HandTrackerData
        {
            std::array<XrHandJointLocationEXT, XR_HAND_JOINT_COUNT_EXT> jointLocations;
            //std::array<XrHandJointVelocityEXT, XR_HAND_JOINT_COUNT_EXT> jointVelocities;
            XrMatrix4x4f baseOrientation;
            XrHandTrackerEXT tracker{ XR_NULL_HANDLE };
        };
        std::array<HandTrackerData, Side::COUNT> handerTrackers;
        std::array<TrackingInfo::Controller, Side::COUNT> controllerInfo{};
    };
    InputState m_input{};

    std::once_flag m_startPassthroughOnce{};
    struct PassthroughLayerData
    {
        XrPassthroughFB passthrough = XR_NULL_HANDLE;
        XrPassthroughLayerFB reconPassthroughLayer = XR_NULL_HANDLE;
    };
    PassthroughLayerData m_ptLayerData {};

#ifdef XR_USE_PLATFORM_WIN32
    // XR_KHR_win32_convert_performance_counter_time
    PFN_xrConvertTimeToWin32PerformanceCounterKHR m_pfnConvertTimeToWin32PerformanceCounterKHR = nullptr;
    PFN_xrConvertWin32PerformanceCounterToTimeKHR m_pfnConvertWin32PerformanceCounterToTimeKHR = nullptr;
#endif
    // XR_KHR_convert_timespec_time
    PFN_xrConvertTimespecTimeToTimeKHR  m_pfnConvertTimespecTimeToTimeKHR = nullptr;
    PFN_xrConvertTimeToTimespecTimeKHR  m_pfnConvertTimeToTimespecTimeKHR = nullptr;
    
    // XR_FB_color_space
    PFN_xrEnumerateColorSpacesFB m_pfnEnumerateColorSpacesFB = nullptr;
    PFN_xrSetColorSpaceFB        m_pfnSetColorSpaceFB = nullptr;

    // XR_EXT_hand_tracking fun pointers.
    PFN_xrCreateHandTrackerEXT  m_pfnCreateHandTrackerEXT = nullptr;
    PFN_xrLocateHandJointsEXT   m_pfnLocateHandJointsEXT = nullptr;
    PFN_xrDestroyHandTrackerEXT m_pfnDestroyHandTrackerEXT = nullptr;

    // XR_FB_display_refresh_rate fun pointers.
    PFN_xrEnumerateDisplayRefreshRatesFB m_pfnEnumerateDisplayRefreshRatesFB = nullptr;
    PFN_xrGetDisplayRefreshRateFB m_pfnGetDisplayRefreshRateFB = nullptr;
    PFN_xrRequestDisplayRefreshRateFB m_pfnRequestDisplayRefreshRateFB = nullptr;

    // XR_FB_PASSTHROUGH_EXTENSION_NAME fun pointers.
    PFN_xrCreatePassthroughFB m_pfnCreatePassthroughFB = nullptr;
    PFN_xrDestroyPassthroughFB m_pfnDestroyPassthroughFB = nullptr;
    PFN_xrPassthroughStartFB m_pfnPassthroughStartFB = nullptr;
    PFN_xrPassthroughPauseFB m_pfnPassthroughPauseFB = nullptr;
    PFN_xrCreatePassthroughLayerFB m_pfnCreatePassthroughLayerFB = nullptr;
    PFN_xrDestroyPassthroughLayerFB m_pfnDestroyPassthroughLayerFB = nullptr;
    PFN_xrPassthroughLayerSetStyleFB m_pfnPassthroughLayerSetStyleFB = nullptr;
    PFN_xrPassthroughLayerPauseFB m_pfnPassthroughLayerPauseFB = nullptr;
    PFN_xrPassthroughLayerResumeFB m_pfnPassthroughLayerResumeFB = nullptr;

#ifdef XR_USE_OXR_PICO
    mutable std::atomic<int>    m_gsIndex{ 0 };
    PFN_xrResetSensorPICO       m_pfnResetSensorPICO = nullptr;
    PFN_xrGetConfigPICO         m_pfnGetConfigPICO = nullptr;
    PFN_xrSetConfigPICO         m_pfnSetConfigPICO = nullptr;
    PFN_xrGetControllerConnectionStatePico m_pfnGetControllerConnectionStatePico = nullptr;
    PFN_xrSetEngineVersionPico  m_pfnSetEngineVersionPico = nullptr;
    PFN_xrStartCVControllerThreadPico m_pfnStartCVControllerThreadPico = nullptr;
    PFN_xrStopCVControllerThreadPico  m_pfnStopCVControllerThreadPico = nullptr;
    PFN_xrVibrateControllerPico m_pfnXrVibrateControllerPico = nullptr;
#endif

    std::atomic<XrTime>      m_lastPredicatedDisplayTime{ 0 };

/// Tracking Thread State ////////////////////////////////////////////////////////
    struct TrackingFrame {
        std::array<XrView, 2> views;
        XrTime                displayTime;
    };
    using TrackingFrameMap = std::map<std::uint64_t, TrackingFrame>;
    mutable std::shared_mutex m_trackingFrameMapMutex;        
    TrackingFrameMap          m_trackingFrameMap{};
    std::atomic<XrDuration>   m_PredicatedLatencyOffset{ 0 };
    static constexpr const std::size_t MaxTrackingFrameCount = 360 * 3;
/// End Tracking Thread State ////////////////////////////////////////////////////

    std::vector<float> m_displayRefreshRates;
    ALXRStreamConfig m_streamConfig {
        .trackingSpaceType = ALXRTrackingSpace::LocalRefSpace,
        .renderConfig {
            .refreshRate = 90.0f,
            .enableFoveation = false
        }
    };

    using StreamConfigQueue     = xrconcurrency::concurrent_queue<ALXRStreamConfig>;
    using GuardianChangedQueue  = xrconcurrency::concurrent_queue<ALXRGuardianData>;
    StreamConfigQueue    m_streamConfigQueue;
    GuardianChangedQueue m_guardianChangedQueue;
    bool                 m_delayOnGuardianChanged = false;
    bool                 m_isMultiViewEnabled = false;
};
}  // namespace

std::shared_ptr<IOpenXrProgram> CreateOpenXrProgram(const std::shared_ptr<Options>& options,
                                                    const std::shared_ptr<IPlatformPlugin>& platformPlugin,
                                                    const std::shared_ptr<IGraphicsPlugin>& graphicsPlugin) {
    return std::make_shared<OpenXrProgram>(options, platformPlugin, graphicsPlugin);
}

std::shared_ptr<IOpenXrProgram> CreateOpenXrProgram(const std::shared_ptr<Options>& options,
                                                    const std::shared_ptr<IPlatformPlugin>& platformPlugin) {
    return std::make_shared<OpenXrProgram>(options, platformPlugin);
}
