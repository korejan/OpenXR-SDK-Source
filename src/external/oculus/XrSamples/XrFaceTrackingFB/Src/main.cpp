// (c) Meta Platforms, Inc. and affiliates.

/*******************************************************************************

Sample app for FB_face_tracking

*******************************************************************************/

#include <cstdint>
#include <cstdio>

#include "XrApp.h"
#include <openxr/fb_face_tracking.h>
#include <openxr/fb_eye_tracking_social.h>

#include "Input/TinyUI.h"

class XrFaceApp : public OVRFW::XrApp {
   public:
    XrFaceApp() : OVRFW::XrApp() {
        BackgroundColor = OVR::Vector4f(1.0f, 0.65f, 0.1f, 1.0f);
    }

    // Returns a list of OpenXr extensions needed for this app
    virtual std::vector<const char*> GetExtensions() override {
        std::vector<const char*> extensions = XrApp::GetExtensions();
        extensions.push_back(XR_FB_FACE_TRACKING_EXTENSION_NAME);
        extensions.push_back(XR_FB_EYE_TRACKING_SOCIAL_EXTENSION_NAME);
        return extensions;
    }

    // Must return true if the application initializes successfully.
    virtual bool AppInit(const xrJava* context) override {
        if (false == ui_.Init(context, GetFileSys())) {
            ALOG("TinyUI::Init FAILED.");
            return false;
        }

        /// Build UI

        static constexpr float kZPos = -2.f;
        static constexpr float kXPosOffset = 1.f;

        static constexpr float kLabelWidth = 440.f;
        static constexpr float kLabelHeight = 40.f;

        static constexpr float kYPosTitleLabel = 2.2f;
        static constexpr float kTitleLabelWidth = 1000.f;
        static constexpr float kTitleLabelHeight = 50.f;
        ui_.AddLabel(
            "OpenXR Face Sample",
            {0.f, kYPosTitleLabel, kZPos},
            {kTitleLabelWidth, kTitleLabelHeight});

        static constexpr float kYPosIsValidLabel = 2.1f;
        labelIsValid_ = ui_.AddLabel(
            kIsValid, {-kXPosOffset, kYPosIsValidLabel, kZPos}, {kLabelWidth, kLabelHeight});
        labelTime_ =
            ui_.AddLabel(kTime, {0.f, kYPosIsValidLabel, kZPos}, {kLabelWidth, kLabelHeight});
        labelIsEyeFollowingBlendshapesValid_ = ui_.AddLabel(
            kIsEyeFollowingBlendshapesValid,
            {kXPosOffset, kYPosIsValidLabel, kZPos},
            {kLabelWidth, kLabelHeight});

        static constexpr float kYPosConfidenceLabel = 2.0f;
        labelUpperFaceConfidence_ = ui_.AddLabel(
            kUpperFaceConfidenceName,
            {-kXPosOffset, kYPosConfidenceLabel, kZPos},
            {kLabelWidth, kLabelHeight});
        labelLowerFaceConfidence_ = ui_.AddLabel(
            kLowerFaceConfidenceName,
            {kXPosOffset, kYPosConfidenceLabel, kZPos},
            {kLabelWidth, kLabelHeight});

        static constexpr float kYPosBlendshapeLabel = 1.9f;
        static constexpr float kYPosBlendshapeLabelOffset = 0.08f;
        static constexpr int kNumBlendshapesInColumn = XR_FACE_EXPRESSION_COUNT_FB / 3;
        for (uint32_t i = 0; i < XR_FACE_EXPRESSION_COUNT_FB; ++i) {
            labels_[i] = ui_.AddLabel(
                kBlendShapeNames[i],
                {-kXPosOffset + kXPosOffset * (i / kNumBlendshapesInColumn),
                 kYPosBlendshapeLabel - kYPosBlendshapeLabelOffset * (i % kNumBlendshapesInColumn),
                 kZPos},
                {kLabelWidth, kLabelHeight});
        }

        // Inspect face tracking system properties
        XrSystemFaceTrackingPropertiesFB faceTrackingSystemProperties{
            XR_TYPE_SYSTEM_FACE_TRACKING_PROPERTIES_FB};
        XrSystemProperties systemProperties{
            XR_TYPE_SYSTEM_PROPERTIES, &faceTrackingSystemProperties};
        OXR(xrGetSystemProperties(GetInstance(), GetSystemId(), &systemProperties));
        if (!faceTrackingSystemProperties.supportsFaceTracking) {
            // The system does not support face tracking
            ALOG("xrGetSystemProperties XR_TYPE_SYSTEM_FACE_TRACKING_PROPERTIES_FB FAILED.");
            return false;
        } else {
            ALOG(
                "xrGetSystemProperties XR_TYPE_SYSTEM_FACE_TRACKING_PROPERTIES_FB OK - initiallizing face tracking...");
        }

        /// Hook up extensions for face tracking
        OXR(xrGetInstanceProcAddr(
            GetInstance(),
            "xrCreateFaceTrackerFB",
            (PFN_xrVoidFunction*)(&xrCreateFaceTrackerFB_)));
        OXR(xrGetInstanceProcAddr(
            GetInstance(),
            "xrDestroyFaceTrackerFB",
            (PFN_xrVoidFunction*)(&xrDestroyFaceTrackerFB_)));
        OXR(xrGetInstanceProcAddr(
            GetInstance(),
            "xrGetFaceExpressionWeightsFB",
            (PFN_xrVoidFunction*)(&xrGetFaceExpressionWeightsFB_)));

        return true;
    }

    virtual void AppShutdown(const xrJava* context) override {
        /// unhook extensions for face tracking
        xrCreateFaceTrackerFB_ = nullptr;
        xrDestroyFaceTrackerFB_ = nullptr;
        xrGetFaceExpressionWeightsFB_ = nullptr;

        OVRFW::XrApp::AppShutdown(context);
        ui_.Shutdown();
    }

    virtual bool SessionInit() override {
        /// Disable scene navitgation
        GetScene().SetFootPos({0.0f, 0.0f, 0.0f});
        this->FreeMove = false;

        if (xrCreateFaceTrackerFB_) {
            XrFaceTrackerCreateInfoFB createInfo{XR_TYPE_FACE_TRACKER_CREATE_INFO_FB};
            OXR(xrCreateFaceTrackerFB_(GetSession(), &createInfo, &faceTracker_));
            ALOG("xrCreateFaceTrackerFB faceTracker_=%llx", (long long)faceTracker_);
        }

        return true;
    }

    virtual void SessionEnd() override {
        if (xrDestroyFaceTrackerFB_) {
            OXR(xrDestroyFaceTrackerFB_(faceTracker_));
        }
    }

    // Update state
    virtual void Update(const OVRFW::ovrApplFrameIn& in) override {
        ui_.HitTestDevices().clear();

        /// Face
        if (faceTracker_ != XR_NULL_HANDLE) {
            XrFaceExpressionWeightsFB expressionWeights{XR_TYPE_FACE_EXPRESSION_WEIGHTS_FB};
            expressionWeights.next = nullptr;
            expressionWeights.weights = weights_;
            expressionWeights.confidences = confidence_;
            expressionWeights.weightCount = XR_FACE_EXPRESSION_COUNT_FB;
            expressionWeights.confidenceCount = XR_FACE_CONFIDENCE_COUNT_FB;

            XrFaceExpressionInfoFB expressionInfo{XR_TYPE_FACE_EXPRESSION_INFO_FB};
            expressionInfo.time = ToXrTime(in.PredictedDisplayTime);

            OXR(xrGetFaceExpressionWeightsFB_(faceTracker_, &expressionInfo, &expressionWeights));

            isValid_ = expressionWeights.status.isValid;
            time_ = FromXrTime(expressionWeights.time);
            isEyeFollowingBlendshapesValid_ =
                expressionWeights.status.isEyeFollowingBlendshapesValid;

            updateLabels();
        }

        ui_.Update(in);
    }

    // Render eye buffers while running
    virtual void Render(const OVRFW::ovrApplFrameIn& in, OVRFW::ovrRendererOutput& out) override {
        /// Render UI
        ui_.Render(in, out);
    }

   public:
    /// Face - extension functions
    PFN_xrCreateFaceTrackerFB xrCreateFaceTrackerFB_ = nullptr;
    PFN_xrDestroyFaceTrackerFB xrDestroyFaceTrackerFB_ = nullptr;
    PFN_xrGetFaceExpressionWeightsFB xrGetFaceExpressionWeightsFB_ = nullptr;
    /// Face - tracker handles
    XrFaceTrackerFB faceTracker_ = XR_NULL_HANDLE;
    /// Face - data buffers
    float weights_[XR_FACE_EXPRESSION_COUNT_FB] = {};
    float confidence_[XR_FACE_CONFIDENCE_COUNT_FB] = {};
    bool isValid_;
    double time_;
    bool isEyeFollowingBlendshapesValid_;

   private:
    OVRFW::TinyUI ui_;

    OVRFW::VRMenuObject* labels_[XR_FACE_EXPRESSION_COUNT_FB];

    OVRFW::VRMenuObject* labelUpperFaceConfidence_;
    OVRFW::VRMenuObject* labelLowerFaceConfidence_;

    OVRFW::VRMenuObject* labelIsValid_;
    OVRFW::VRMenuObject* labelTime_;
    OVRFW::VRMenuObject* labelIsEyeFollowingBlendshapesValid_;

    const char* const kBlendShapeNames[XR_FACE_EXPRESSION_COUNT_FB] = {
        "BROW_LOWERER_L",
        "BROW_LOWERER_R",
        "CHEEK_PUFF_L",
        "CHEEK_PUFF_R",
        "CHEEK_RAISER_L",
        "CHEEK_RAISER_R",
        "CHEEK_SUCK_L",
        "CHEEK_SUCK_R",
        "CHIN_RAISER_B",
        "CHIN_RAISER_T",
        "DIMPLER_L",
        "DIMPLER_R",
        "EYES_CLOSED_L",
        "EYES_CLOSED_R",
        "EYES_LOOK_DOWN_L",
        "EYES_LOOK_DOWN_R",
        "EYES_LOOK_LEFT_L",
        "EYES_LOOK_LEFT_R",
        "EYES_LOOK_RIGHT_L",
        "EYES_LOOK_RIGHT_R",
        "EYES_LOOK_UP_L",
        "EYES_LOOK_UP_R",
        "INNER_BROW_RAISER_L",
        "INNER_BROW_RAISER_R",
        "JAW_DROP",
        "JAW_SIDEWAYS_LEFT",
        "JAW_SIDEWAYS_RIGHT",
        "JAW_THRUST",
        "LID_TIGHTENER_L",
        "LID_TIGHTENER_R",
        "LIP_CORNER_DEPRESSOR_L",
        "LIP_CORNER_DEPRESSOR_R",
        "LIP_CORNER_PULLER_L",
        "LIP_CORNER_PULLER_R",
        "LIP_FUNNELER_LB",
        "LIP_FUNNELER_LT",
        "LIP_FUNNELER_RB",
        "LIP_FUNNELER_RT",
        "LIP_PRESSOR_L",
        "LIP_PRESSOR_R",
        "LIP_PUCKER_L",
        "LIP_PUCKER_R",
        "LIP_STRETCHER_L",
        "LIP_STRETCHER_R",
        "LIP_SUCK_LB",
        "LIP_SUCK_LT",
        "LIP_SUCK_RB",
        "LIP_SUCK_RT",
        "LIP_TIGHTENER_L",
        "LIP_TIGHTENER_R",
        "LIPS_TOWARD",
        "LOWER_LIP_DEPRESSOR_L",
        "LOWER_LIP_DEPRESSOR_R",
        "MOUTH_LEFT",
        "MOUTH_RIGHT",
        "NOSE_WRINKLER_L",
        "NOSE_WRINKLER_R",
        "OUTER_BROW_RAISER_L",
        "OUTER_BROW_RAISER_R",
        "UPPER_LID_RAISER_L",
        "UPPER_LID_RAISER_R",
        "UPPER_LIP_RAISER_L",
        "UPPER_LIP_RAISER_R",
    };

    const char* kIsValid = "IS VALID";
    const char* kTime = "TIME";
    const char* kIsEyeFollowingBlendshapesValid = "IS EYE FOLLOWING SHAPES VALID";

    const char* kUpperFaceConfidenceName = "UPPER FACE CONFIDENCE";
    const char* kLowerFaceConfidenceName = "LOWER FACE CONFIDENCE";

    void updateLabels() {
        char buf[100];
        for (uint32_t i = 0; i < XR_FACE_EXPRESSION_COUNT_FB; ++i) {
            snprintf(buf, sizeof(buf), "%s:%.2f", kBlendShapeNames[i], weights_[i]);
            labels_[i]->SetText(buf);
        }
        snprintf(buf, sizeof(buf), "%s:%s", kIsValid, isValid_ ? "T" : "F");
        labelIsValid_->SetText(buf);
        snprintf(buf, sizeof(buf), "%s:%.3f", kTime, time_);
        labelTime_->SetText(buf);
        snprintf(
            buf,
            sizeof(buf),
            "%s:%s",
            kIsEyeFollowingBlendshapesValid,
            isEyeFollowingBlendshapesValid_ ? "T" : "F");
        labelIsEyeFollowingBlendshapesValid_->SetText(buf);
        snprintf(
            buf,
            sizeof(buf),
            "%s:%.2f",
            kUpperFaceConfidenceName,
            confidence_[XR_FACE_CONFIDENCE_UPPER_FACE_FB]);
        labelUpperFaceConfidence_->SetText(buf);
        snprintf(
            buf,
            sizeof(buf),
            "%s:%.2f",
            kLowerFaceConfidenceName,
            confidence_[XR_FACE_CONFIDENCE_LOWER_FACE_FB]);
        labelLowerFaceConfidence_->SetText(buf);
    }
};

ENTRY_POINT(XrFaceApp)
