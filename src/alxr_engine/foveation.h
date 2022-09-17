#pragma once
#ifndef ALXR_FOVEATION_H
#define ALXR_FOVEATION_H
#include <type_traits>
#include <cmath>
#include "alxr_ctypes.h"

namespace ALXR {
    struct alignas(16) FoveatedDecodeParams {
        XrVector2f eyeSizeRatio;
        XrVector2f centerSize;
        XrVector2f centerShift;
        XrVector2f edgeRatio;
    };
    static_assert(std::is_standard_layout<FoveatedDecodeParams>());

    inline FoveatedDecodeParams MakeFoveatedDecodeParams
    (
        const XrVector2f& targetEyeSize,
        const XrVector2f& centerSize,
        const XrVector2f& centerShift,
        const XrVector2f& edgeRatio
    )
    {
        const float edgeSizeX = targetEyeSize.x - centerSize.x * targetEyeSize.x;
        const float edgeSizeY = targetEyeSize.y - centerSize.y * targetEyeSize.y;

        const float centerSizeXAligned = static_cast<const float>(1. - std::ceil(edgeSizeX / (edgeRatio.x * 2.)) * (edgeRatio.x * 2.) / targetEyeSize.x);
        const float centerSizeYAligned = static_cast<const float>(1. - std::ceil(edgeSizeY / (edgeRatio.y * 2.)) * (edgeRatio.y * 2.) / targetEyeSize.y);

        const float edgeSizeXAligned = targetEyeSize.x - centerSizeXAligned * targetEyeSize.x;
        const float edgeSizeYAligned = targetEyeSize.y - centerSizeYAligned * targetEyeSize.y;

        const float centerShiftXAligned = static_cast<const float>(std::ceil(centerShift.x * edgeSizeXAligned / (edgeRatio.x * 2.)) * (edgeRatio.x * 2.) / edgeSizeXAligned);
        const float centerShiftYAligned = static_cast<const float>(std::ceil(centerShift.y * edgeSizeYAligned / (edgeRatio.y * 2.)) * (edgeRatio.y * 2.) / edgeSizeYAligned);

        const float foveationScaleX = (centerSizeXAligned + (1.0f - centerSizeXAligned) / edgeRatio.x);
        const float foveationScaleY = (centerSizeYAligned + (1.0f - centerSizeYAligned) / edgeRatio.y);

        const float optimizedEyeWidth  = foveationScaleX * targetEyeSize.x;
        const float optimizedEyeHeight = foveationScaleY * targetEyeSize.y;

        // round the frame dimensions to a number of pixel multiple of 32 for the encoder
        const auto optimizedEyeWidthAligned  = (uint32_t)std::ceil(optimizedEyeWidth / 32.f) * 32;
        const auto optimizedEyeHeightAligned = (uint32_t)std::ceil(optimizedEyeHeight / 32.f) * 32;

        const float eyeWidthRatioAligned  = optimizedEyeWidth / optimizedEyeWidthAligned;
        const float eyeHeightRatioAligned = optimizedEyeHeight / optimizedEyeHeightAligned;

        return {
            .eyeSizeRatio { eyeWidthRatioAligned, eyeHeightRatioAligned },
            .centerSize   { centerSizeXAligned,   centerSizeYAligned    },
            .centerShift  { centerShiftXAligned,  centerShiftYAligned   },
            .edgeRatio    { edgeRatio.x, edgeRatio.y }
        };
    }

    inline FoveatedDecodeParams MakeFoveatedDecodeParams(const ALXRRenderConfig& rc) {
        return MakeFoveatedDecodeParams
        (
            XrVector2f{ float(rc.eyeWidth), float(rc.eyeHeight) },
            XrVector2f{ rc.foveationCenterSizeX,  rc.foveationCenterSizeY  },
            XrVector2f{ rc.foveationCenterShiftX, rc.foveationCenterShiftY },
            XrVector2f{ rc.foveationEdgeRatioX,   rc.foveationEdgeRatioY   }
        );
    }
}
#endif
