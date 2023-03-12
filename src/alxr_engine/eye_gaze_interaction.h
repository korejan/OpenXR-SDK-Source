#pragma once
#ifndef ALXR_EYE_GAZE_INTERACTION_H
#define ALXR_EYE_GAZE_INTERACTION_H
#include <cstring>
#include <string>
#include <optional>
#include <atomic>
#include "interaction_profiles.h"

namespace ALXR {;

struct EyeGazeInteraction {
		
	EyeGazeInteraction(XrInstance instance, XrSession session, XrActionSet actionSet);
	
	EyeGazeInteraction(const EyeGazeInteraction&) = delete;
	EyeGazeInteraction& operator=(const EyeGazeInteraction&) = delete;

	~EyeGazeInteraction();
	void Clear();

	void PollActions();

	inline std::optional<XrSpaceLocation> GetSpaceLocation
	(
		const XrSpace& baseSpace,
		const XrTime& time
	) const;

private:
	XrPath GetXrPath(const char* const str) const;
	XrPath GetXrPath(const InteractionProfile& profile) const;

	XrInstance m_instance		{ XR_NULL_HANDLE };
	XrSession  m_session		{ XR_NULL_HANDLE };

	XrAction m_eyeGazePoseAction{ XR_NULL_HANDLE };
	XrSpace  m_eyeGazeSpace		{ XR_NULL_HANDLE };
	std::atomic<XrBool32> m_eyeGazeActive{ XR_FALSE };

};
////////////////////////////////////////////////////////////////////////////////////////////////////
inline void EyeGazeInteraction::Clear() {
	if (m_eyeGazeSpace != XR_NULL_HANDLE) {
		Log::Write(Log::Level::Verbose, "Destroying Eye Gaze Action Spaces");
		xrDestroySpace(m_eyeGazeSpace);
		m_eyeGazeSpace = XR_NULL_HANDLE;
	}

	m_eyeGazeActive = XR_FALSE;
	m_eyeGazePoseAction = XR_NULL_HANDLE;
	m_session = XR_NULL_HANDLE;
	m_instance = XR_NULL_HANDLE;
}

inline EyeGazeInteraction::~EyeGazeInteraction() {
	Log::Write(Log::Level::Verbose, "Destroying EyeGazeInteraction");
	Clear();
}

inline XrPath EyeGazeInteraction::GetXrPath(const char* const str) const {
	XrPath path{ XR_NULL_PATH };
	CHECK_XRCMD(xrStringToPath(m_instance, str, &path));
	assert(path != XR_NULL_PATH);
	return path;
}

inline XrPath EyeGazeInteraction::GetXrPath(const InteractionProfile& profile) const {
	return GetXrPath(profile.path);
}

inline EyeGazeInteraction::EyeGazeInteraction(
	XrInstance instance,
	XrSession  session,
	XrActionSet actionSet
)
: m_instance(instance),
  m_session(session)
{
	CHECK(m_instance != XR_NULL_HANDLE);
	CHECK(m_session != XR_NULL_HANDLE);
	CHECK(actionSet != XR_NULL_HANDLE);

	XrActionCreateInfo actionInfo {
		.type = XR_TYPE_ACTION_CREATE_INFO,
		.next = nullptr,
		.actionType = XR_ACTION_TYPE_POSE_INPUT,
		.countSubactionPaths = 0,
		.subactionPaths = nullptr
	};
	std::strcpy(actionInfo.actionName, "eye_gaze_pose");
	std::strcpy(actionInfo.localizedActionName, "Eye Gaze Pose");
	CHECK_XRCMD(xrCreateAction(actionSet, &actionInfo, &m_eyeGazePoseAction));
	CHECK(m_eyeGazePoseAction != XR_NULL_HANDLE);

	const XrActionSpaceCreateInfo actionSpaceInfo {
		.type = XR_TYPE_ACTION_SPACE_CREATE_INFO,
		.next = nullptr,
		.action = m_eyeGazePoseAction,
		.poseInActionSpace {
			.orientation {
				.x = 0.0f,
				.y = 0.0f,
				.z = 0.0f,
				.w = 1.f
			},
			.position {
				.x = 0.0f,
				.y = 0.0f,
				.z = 0.0f
			},
		},
    };
	CHECK_XRCMD(xrCreateActionSpace(m_session, &actionSpaceInfo, &m_eyeGazeSpace));
	CHECK(m_eyeGazeSpace != XR_NULL_HANDLE);

	using namespace std::string_literals;
	const auto fullPath = EyeGazeProfile.userEyesPath + "/input/"s + EyeGazeProfile.eyeGazePosePath;

	const std::vector<XrActionSuggestedBinding> bindings{
		XrActionSuggestedBinding {
			.action = m_eyeGazePoseAction,
			.binding = GetXrPath(fullPath.c_str())
		}
	};

	Log::Write(Log::Level::Info, Fmt("Creating suggested bindings for profile: \"%s\"", EyeGazeProfile.path));
	const XrInteractionProfileSuggestedBinding suggestedBindings{
		.type = XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING,
		.next = nullptr,
		.interactionProfile = GetXrPath(EyeGazeProfile),
		.countSuggestedBindings = (uint32_t)bindings.size(),
		.suggestedBindings = bindings.data()
	};
	CHECK_XRCMD(xrSuggestInteractionProfileBindings(m_instance, &suggestedBindings));
}

inline void EyeGazeInteraction::PollActions() {

	const XrActionStateGetInfo getInfo {
		.type = XR_TYPE_ACTION_STATE_GET_INFO,
		.next = nullptr,
		.action = m_eyeGazePoseAction,
		.subactionPath = XR_NULL_PATH
	};
	XrActionStatePose poseState{
		.type = XR_TYPE_ACTION_STATE_POSE,
		.next = nullptr,
		.isActive = XR_FALSE
	};
	if (XR_FAILED(xrGetActionStatePose(m_session, &getInfo, &poseState))) {
		m_eyeGazeActive = XR_FALSE;
		return;
	}
	m_eyeGazeActive = poseState.isActive;
}

inline std::optional<XrSpaceLocation> EyeGazeInteraction::GetSpaceLocation
(
	const XrSpace& baseSpace,
	const XrTime& time
) const {
	if (m_eyeGazeActive == XR_FALSE)
		return {};
	XrEyeGazeSampleTimeEXT eyeGazeSampleTime {
		.type = XR_TYPE_EYE_GAZE_SAMPLE_TIME_EXT,
		.next = nullptr,
		.time = 0
	};
	XrSpaceLocation gazeLocation {
		.type = XR_TYPE_SPACE_LOCATION,
		.next = &eyeGazeSampleTime,
		.locationFlags = 0
	};
	if (XR_FAILED(xrLocateSpace(m_eyeGazeSpace, baseSpace, time, &gazeLocation)))
		return {};
	return gazeLocation;
}

}
#endif
