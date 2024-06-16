#pragma once
#ifndef XR_VIDEO_DECODER_H
#define XR_VIDEO_DECODER_H

#include <cstdint>
#include <memory>
#include <atomic>
#include <span>
#include <string>
#include <unordered_map>

#include "alxr_ctypes.h"

struct OptionMap {
    template < typename Tp >
    using make_map = std::unordered_map<std::string, Tp>;
    using StringMap = make_map<std::string>;
    using FloatMap = make_map<float>;
    using Int64Map = make_map<std::int64_t>;
    using Int32Map = make_map<std::int32_t>;
private:
    StringMap stringMap;
    FloatMap  floatMap;
    Int64Map  int64Map;
    Int32Map  int32Map;
public:
    constexpr static const int COLOR_FormatYUV420Flexible = 2135033992;

    inline OptionMap() noexcept = default;
    inline OptionMap(const OptionMap&) noexcept = default;
    inline OptionMap(OptionMap&&) noexcept = default;
    inline OptionMap& operator=(const OptionMap&) noexcept = default;
    inline OptionMap& operator=(OptionMap&&) noexcept = default;

    void set(const std::string& key, const std::string& val)
    {
        if (key.length() == 0 || val.length() == 0)
            return;
        stringMap[key] = val;
    }
    void set(const std::string& key, const float val)
    {
        if (key.length() == 0)
            return;
        floatMap[key] = val;
    }
    void setInt64(const std::string& key, const std::int64_t val)
    {
        if (key.length() == 0)
            return;
        int64Map[key] = val;
    }
    void setInt32(const std::string& key, const std::int32_t val)
    {
        if (key.length() == 0)
            return;
        int32Map[key] = val;
    }
    const StringMap& string_map() const { return stringMap; }
    const FloatMap& float_map() const { return floatMap; }
    const Int64Map& int64_map() const { return int64Map; }
    const Int32Map& int32_map() const { return int32Map; }
};

struct VideoFrame;
struct ALXRClientCtx;
struct IOpenXrProgram;

struct IDecoderPlugin {
    using PacketType = std::span<const std::uint8_t>;

	virtual bool QueuePacket
	(
        const PacketType& /*newPacketData*/,
		const std::uint64_t /*trackingFrameIndex*/
	) = 0;

    using shared_bool = std::atomic<bool>;
    struct RunCtx final {
        using IOpenXrProgramPtr = std::shared_ptr<IOpenXrProgram>;
        using ClientCtxPtr = std::shared_ptr<const ALXRClientCtx>;

        OptionMap         optionMap;
        ALXRRenderConfig  renderConfig;
        ALXRDecoderConfig config;
        ClientCtxPtr      clientCtx;
        IOpenXrProgramPtr programPtr;
        ALXRDecoderType   decoderType;
    };
    virtual bool Run(shared_bool& /*isRunningToken*/) = 0;

    constexpr inline IDecoderPlugin() noexcept = default;
    inline virtual ~IDecoderPlugin() = default;
	IDecoderPlugin(const IDecoderPlugin&) noexcept = delete;
	IDecoderPlugin& operator=(const IDecoderPlugin&) noexcept = delete;
	IDecoderPlugin(IDecoderPlugin&&) noexcept = delete;
	IDecoderPlugin& operator=(IDecoderPlugin&&) noexcept = delete;
};

std::shared_ptr<IDecoderPlugin> CreateDecoderPlugin(const IDecoderPlugin::RunCtx&);

#endif
