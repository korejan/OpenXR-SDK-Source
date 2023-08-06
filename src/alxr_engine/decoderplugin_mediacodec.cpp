#if defined(XR_USE_PLATFORM_ANDROID) && !defined(XR_DISABLE_DECODER_THREAD)
#include "pch.h"
#include "common.h"
#include "decoderplugin.h"

#include <span>
#include <vector>
#include <chrono>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <thread>

#include <readerwritercircularbuffer.h>

#include <media/NdkMediaCodec.h>
#include <media/NdkMediaMuxer.h> 
#include <media/NdkMediaError.h>
#include <media/NdkMediaFormat.h>
#include <media/NdkImage.h>
#include <media/NdkImageReader.h>
#include <media/NdkMediaCrypto.h>
#include <media/NdkMediaDrm.h>
#include <media/NdkMediaExtractor.h>

#include "alxr_ctypes.h"
#include "nal_utils.h"
#include "graphicsplugin.h"
#include "openxr_program.h"
#include "latency_manager.h"
#include "timing.h"

namespace
{;
struct FrameIndexMap
{
    using TimeStamp = std::uint64_t;
    using FrameIndex = std::uint64_t;
    constexpr static const FrameIndex NullIndex = FrameIndex(-1);
private:
    using Value = std::atomic<FrameIndex>;
    using FrameMap = std::vector<Value>;
    FrameMap m_frameMap;

    constexpr inline std::size_t index(const TimeStamp ts) const
    {
        return ts % m_frameMap.size();
    }

public:
    inline FrameIndexMap(const std::size_t N)
    : m_frameMap(N) {}
    inline FrameIndexMap(const FrameIndexMap&) = delete;
    inline FrameIndexMap(FrameIndexMap&&) = delete;
    inline FrameIndexMap& operator=(const FrameIndexMap&) = delete;
    inline FrameIndexMap& operator=(FrameIndexMap&&) = delete;

    constexpr inline void set(const TimeStamp ts, const FrameIndex newIdx)
    {
        m_frameMap[index(ts)].store(newIdx);
    }

    constexpr inline FrameIndex get(const TimeStamp ts) const
    {
        return m_frameMap[index(ts)].load();
    }

    constexpr inline FrameIndex get_clear(const TimeStamp ts)
    {
        return m_frameMap[index(ts)].exchange(NullIndex);
    }
};

using EncodedFrame = std::vector<std::uint8_t>;
struct NALPacket
{
    EncodedFrame data;
    std::uint64_t frameIndex;

    /*constexpr*/ inline NALPacket(const ConstPacketType& p, const std::uint64_t newFrameIdx) noexcept //, const ALVR_CODEC codec)
        : data{ p.begin(), p.end() },
        frameIndex(newFrameIdx)
    {}

    constexpr inline NalType nal_type(const ALXRCodecType codec) const
    {
        return get_nal_type({ data.data(), data.size() }, static_cast<ALVR_CODEC>(codec));
    }

    constexpr inline bool is_config(const ALXRCodecType codec) const
    {
        return ::is_config(nal_type(codec), static_cast<ALVR_CODEC>(codec));
    }

    constexpr inline bool is_idr(const ALXRCodecType codec) const
    {
        return ::is_idr(nal_type(codec), static_cast<ALVR_CODEC>(codec));
    }

    constexpr inline bool empty() const { return data.empty(); }

    /*constexpr*/ inline NALPacket() noexcept = default;
    constexpr inline NALPacket(const NALPacket&) noexcept = delete;
    /*constexpr*/ inline NALPacket(NALPacket&&) noexcept = default;
    constexpr inline NALPacket& operator=(const NALPacket&) noexcept = delete;
    /*constexpr*/ inline NALPacket& operator=(NALPacket&&) noexcept = default;
};
using NALPacketPtr = std::unique_ptr<NALPacket>;

struct XrImageListener
{
    using IOpenXrProgramPtr = std::shared_ptr<IOpenXrProgram>;
    using AImageReaderPtr = std::unique_ptr<AImageReader, decltype(AImageReader_delete)*>;

    FrameIndexMap     frameIndexMap{ 4096 };
    IOpenXrProgramPtr programPtr;
    AImageReaderPtr   imageReader;
    AImageReader_ImageListener imageListener;

    // This mutex is only neccessary where in the case of a residue OnImageAvailable callback is still "processing" or waiting
    // during/after an XrImageListener has been destroyed in another thread, this should not be used in any other case.
    std::mutex        listenerDestroyMutex;

    constexpr static const std::uint64_t ImageReaderFlags =
        AHARDWAREBUFFER_USAGE_CPU_WRITE_NEVER |
        AHARDWAREBUFFER_USAGE_CPU_READ_NEVER |
        AHARDWAREBUFFER_USAGE_GPU_SAMPLED_IMAGE;

    constexpr static const std::int32_t MaxImageCount = 5;

    inline static AImageReaderPtr MakeImageReader()
    {
        AImageReader* newImgReader = nullptr;
        if (AImageReader_newWithUsage(1, 1, AIMAGE_FORMAT_PRIVATE, ImageReaderFlags, MaxImageCount, &newImgReader) != AMEDIA_OK ||
            newImgReader == nullptr) {
            return { nullptr, [](AImageReader*) {} };
        }
        return { newImgReader, AImageReader_delete };
    }

    XrImageListener(const IOpenXrProgramPtr& pptr)
    : programPtr(pptr),
      imageReader(MakeImageReader()),
      imageListener {
          .context = this,
          .onImageAvailable = &XrImageListener::OnImageAvailable
      }
    {
        if (programPtr == nullptr || imageReader == nullptr) {
            imageReader.reset();
            programPtr.reset();
            return;
        }
        if (AImageReader_setImageListener(imageReader.get(), &imageListener) != AMEDIA_OK) {
            imageReader.reset();
            programPtr.reset();
        }
    }

    inline XrImageListener(const XrImageListener&) noexcept = delete;
    inline XrImageListener(XrImageListener&&)  noexcept = delete;
    inline XrImageListener& operator=(const XrImageListener&) noexcept = delete;
    inline XrImageListener& operator=(XrImageListener&&)  noexcept = delete;

    ~XrImageListener()
    {
        {
            std::scoped_lock sl(listenerDestroyMutex);
            if (imageReader)
                AImageReader_setImageListener(imageReader.get(), nullptr);
            imageListener.onImageAvailable = nullptr;
        }
        Log::Write(Log::Level::Info, "XrImageListener destroyed");
    }

    inline bool IsValid() const { return imageReader != nullptr || programPtr != nullptr; }

    inline ANativeWindow* GetWindow() const
    {
        if (imageReader == nullptr)
            return nullptr;
        ANativeWindow* surface_handle = nullptr;
        CHECK(AImageReader_getWindow(imageReader.get(), &surface_handle) == AMEDIA_OK);
        return surface_handle;
    }

    inline void OnImageAvailable(AImageReader* reader)
    {
        std::scoped_lock sl(listenerDestroyMutex);
        using AImagePtr = std::unique_ptr<AImage, decltype(AImage_delete)*>;
        auto img = [&]() -> AImagePtr
        {
            AImage* tmp = nullptr;
            if (AImageReader_acquireLatestImage(reader, &tmp) != AMEDIA_OK)
                return { nullptr, [](AImage*) {} };
            return { tmp, AImage_delete };
        }();
        if (img == nullptr) {
            Log::Write(Log::Level::Error, "XrImageListener: Failed to acquire latest AImage");
            return;
        }

        std::int64_t presentationTimeNs = 0;
        AImage_getTimestamp(img.get(), &presentationTimeNs);
        const auto ptsUs = static_cast<std::uint64_t>(presentationTimeNs * 0.001);
        const auto frameIndex = frameIndexMap.get_clear(ptsUs);
        if (frameIndex == FrameIndexMap::NullIndex) {
            Log::Write(Log::Level::Warning, Fmt("XrImageListener: Unknown frame index for pts: %lld us, frame ignored", ptsUs));
            return;
        }

        if (const auto graphicsPluginPtr = programPtr->GetGraphicsPlugin()) {
            std::int32_t w = 0, h = 0;
            AImage_getWidth(img.get(), &w);
            AImage_getHeight(img.get(), &h);
            const IGraphicsPlugin::YUVBuffer buf{
                .luma {
                    .data = img.release(),
                    .pitch = (std::size_t)w,
                    .height = (std::size_t)h
                },
                .frameIndex = frameIndex
            };
            graphicsPluginPtr->UpdateVideoTextureMediaCodec(buf);
        }
    }

    static inline void OnImageAvailable(void* ctx, AImageReader* reader)
    {
        assert(ctx != nullptr);
        reinterpret_cast<XrImageListener*>(ctx)->OnImageAvailable(reader);
    }
};

struct AMediaCodecDeleter {
    void operator()(AMediaCodec* codec) const {
        if (codec == nullptr)
            return;
        CHECK(AMediaCodec_delete(codec) == AMEDIA_OK);
    }
};
using AMediaCodecPtr = std::shared_ptr<AMediaCodec>;

class DecoderOutputThread
{
    std::thread m_thread;
    FrameIndexMap& m_frameIndexMap;
    std::atomic<bool> m_isRunning{ false };
public:
    inline DecoderOutputThread(FrameIndexMap& frameMapRef)
    : m_frameIndexMap(frameMapRef)
    {}

    inline DecoderOutputThread(const DecoderOutputThread&) noexcept = delete;
    inline DecoderOutputThread(DecoderOutputThread&&) noexcept = delete;
    inline DecoderOutputThread& operator=(const DecoderOutputThread&) noexcept = delete;
    inline DecoderOutputThread& operator=(DecoderOutputThread&&) noexcept = delete;

    inline ~DecoderOutputThread() {
        Stop();
        assert(!m_thread.joinable());
        Log::Write(Log::Level::Info, "DecoderOutputThread destroyed");
    }

    bool Start(const AMediaCodecPtr& newCodec)
    {
        if (newCodec == nullptr || m_isRunning)
            return false;
        std::mutex startMutex;
        std::condition_variable cv;
        m_thread = std::thread([this, newCodec, &startMutex, &cv]()
        {
            {
                std::lock_guard lk(startMutex);
                m_isRunning.store(true);
            }
            cv.notify_one();
            Run(newCodec);
        });
        {
            using namespace std::chrono_literals;
            std::unique_lock lk(startMutex);
            if (!cv.wait_for(lk, 1s, [this] { return m_isRunning.load(); })) {
                Log::Write(Log::Level::Error, "Waiting for decoder thread to start timed out.");
            }
        }
        return m_isRunning;
    }

    inline void Stop()
    {
        if (!m_isRunning)
            return;
        Log::Write(Log::Level::Info, "shutting down decoder output thread");
        m_isRunning = false;
        if (m_thread.joinable())
            m_thread.join();
        Log::Write(Log::Level::Info, "Decoder output thread finished shutdown");
    }

    inline void Run(const AMediaCodecPtr& codec)
    {
        if (codec == nullptr) {
            Log::Write(Log::Level::Error, "Codec not set for decoder output thread.");
            return;
        }
        assert(codec != nullptr);
        while (m_isRunning)
        {
            AMediaCodecBufferInfo buffInfo{};
            const auto outputBufferId = AMediaCodec_dequeueOutputBuffer(codec.get(), &buffInfo, 300);
            if (outputBufferId >= 0 &&
                outputBufferId != AMEDIACODEC_INFO_OUTPUT_BUFFERS_CHANGED &&
                outputBufferId != AMEDIACODEC_INFO_OUTPUT_FORMAT_CHANGED &&
                outputBufferId != AMEDIACODEC_INFO_TRY_AGAIN_LATER)
            {
                const auto ptsUs = static_cast<std::uint64_t>(buffInfo.presentationTimeUs);
                const auto frameIndex = m_frameIndexMap.get(ptsUs);
                if (frameIndex != FrameIndexMap::NullIndex) {
                    LatencyCollector::Instance().decoderOutput(frameIndex);
                }
                AMediaCodec_releaseOutputBuffer(codec.get(), outputBufferId, true);
            }
            else if (outputBufferId == AMEDIACODEC_INFO_OUTPUT_FORMAT_CHANGED)
            {
                AMediaFormat* outputFormat = AMediaCodec_getOutputFormat(codec.get());
                std::int32_t w = 0, h = 0;
                AMediaFormat_getInt32(outputFormat, AMEDIAFORMAT_KEY_WIDTH, &w);
                AMediaFormat_getInt32(outputFormat, AMEDIAFORMAT_KEY_HEIGHT, &h);
                assert(w != 0 && h != 0);
                Log::Write(Log::Level::Info, Fmt("OUTPUT_FORMAT_CHANGED, w:%d, h:%d", w, h));
            }
        }
    }
};

struct MediaCodecDecoderPlugin final : IDecoderPlugin
{
    using AVPacketQueue = moodycamel::BlockingReaderWriterCircularBuffer<NALPacket>;
    using GraphicsPluginPtr = std::shared_ptr<IGraphicsPlugin>;

    AVPacketQueue           m_packetQueue { 360 };
    std::atomic<ALVR_CODEC> m_selectedCodecType { ALVR_CODEC_H265 };

    virtual ~MediaCodecDecoderPlugin() override {
        Log::Write(Log::Level::Info, "MediaCodecDecoderPlugin destroyed");
    }
    
	virtual inline bool QueuePacket
	(
        const IDecoderPlugin::PacketType& newPacketData,
		const std::uint64_t trackingFrameIndex
	) override
    {
        using namespace std::literals::chrono_literals;
        constexpr static const auto QueueWaitTimeout = 500ms;
        
        const auto selectedCodec = m_selectedCodecType.load();
        const auto vpssps = find_vpssps(newPacketData, selectedCodec);
        if (is_config(vpssps, selectedCodec))
        {
            NALPacket configPacket{ vpssps, trackingFrameIndex };
            const auto frameData = newPacketData.subspan(vpssps.size(), newPacketData.size() - vpssps.size());
            NALPacket framePacket{ frameData, trackingFrameIndex };
            m_packetQueue.wait_enqueue_timed(std::move(configPacket), QueueWaitTimeout);
            m_packetQueue.wait_enqueue_timed(std::move(framePacket), QueueWaitTimeout);
        }
        else
            m_packetQueue.wait_enqueue_timed({ newPacketData, trackingFrameIndex }, QueueWaitTimeout);
		return true;
	}

    struct AMediaFormatDeleter {
        void operator()(AMediaFormat* fmt) const {
            if (fmt == nullptr)
                return;
            CHECK(AMediaFormat_delete(fmt) == AMEDIA_OK);
        }
    };
    using AMediaFormatPtr = std::unique_ptr<AMediaFormat, AMediaFormatDeleter>;
    inline AMediaFormatPtr MakeMediaFormat
    (
        const char* const mimeType,
        const OptionMap& optionMap,
        const EncodedFrame& csd0,
        const bool realtimePriority = true
    )
    {
        AMediaFormat* format = AMediaFormat_new();
        if (format == nullptr)
            return nullptr;

        AMediaFormat_setString(format, AMEDIAFORMAT_KEY_MIME, mimeType);
        AMediaFormat_setInt32(format, AMEDIAFORMAT_KEY_WIDTH, 512);
        AMediaFormat_setInt32(format, AMEDIAFORMAT_KEY_HEIGHT, 1024);

        for (const auto& [key, val] : optionMap.string_map())
            AMediaFormat_setString(format, key.c_str(), val.c_str());
        for (const auto& [key, val] : optionMap.float_map())
            AMediaFormat_setFloat(format, key.c_str(), val);
        for (const auto& [key, val] : optionMap.int64_map())
            AMediaFormat_setInt64(format, key.c_str(), val);
        for (const auto& [key, val] : optionMap.int32_map())
            AMediaFormat_setInt32(format, key.c_str(), val);

        AMediaFormat_setInt32(format, AMEDIAFORMAT_KEY_OPERATING_RATE, std::numeric_limits<std::int16_t>::max());
        AMediaFormat_setInt32(format, AMEDIAFORMAT_KEY_PRIORITY, realtimePriority ? 0 : 1);//ctx.config.realtimePriority ? 0 : 1);

#if defined(__ANDROID_API__) && (__ANDROID_API__ >= 30)
#pragma message ("Setting android 11(+) LOW_LATENCY key enabled.")
        AMediaFormat_setInt32(format, AMEDIAFORMAT_KEY_LOW_LATENCY, 1);
#endif
        assert(csd0.size() > 0);
        AMediaFormat_setBuffer(format, AMEDIAFORMAT_KEY_CSD_0, csd0.data(), csd0.size());

        return AMediaFormatPtr { format };
    }

    virtual inline bool Run(const IDecoderPlugin::RunCtx& ctx, IDecoderPlugin::shared_bool& isRunningToken) override
    {
        if (!isRunningToken || ctx.programPtr == nullptr) {
            Log::Write(Log::Level::Error, "Decoder run parameters not valid.");
            return false;
        }
        m_selectedCodecType.store(static_cast<ALVR_CODEC>(ctx.config.codecType));
        
        XrImageListener imgListener { ctx.programPtr };
        if (!imgListener.IsValid()) {
            Log::Write(Log::Level::Error, "Failed to create image reader/listener.");
            return false;
        }

        AMediaCodecPtr codec{ nullptr };
        AMediaFormatPtr format{ nullptr };        
        DecoderOutputThread outputThread{ imgListener.frameIndexMap };
        static constexpr const std::int64_t QueueWaitTimeout = 5e+5;
        while (isRunningToken)
        {
            NALPacket packet{};
            if (!m_packetQueue.wait_dequeue_timed(packet, QueueWaitTimeout))
                continue;

            if (codec == nullptr && packet.is_config(ctx.config.codecType))
            {
                Log::Write(Log::Level::Info, "Spawning decoder...");
                const char* const mimeType = ctx.config.codecType == ALXRCodecType::HEVC_CODEC ? "video/hevc" : "video/avc";
                codec.reset(AMediaCodec_createDecoderByType(mimeType), AMediaCodecDeleter());
                if (codec == nullptr)
                {
                    Log::Write(Log::Level::Error, "AMediaCodec_createDecoderByType failed!");
                    break;
                }
                assert(codec != nullptr);

                char* codecName = nullptr;
                if (AMediaCodec_getName(codec.get(), &codecName) == AMEDIA_OK) {
                    Log::Write(Log::Level::Info, Fmt("Selected decoder: %s", codecName));
                    AMediaCodec_releaseName(codec.get(), codecName);
                }

                format = MakeMediaFormat(mimeType, ctx.optionMap, packet.data, ctx.config.realtimePriority);
                assert(format != nullptr);

                ANativeWindow* const surface_handle = imgListener.GetWindow();
                CHECK(surface_handle != nullptr);
                auto status = AMediaCodec_configure(codec.get(), format.get(), surface_handle, nullptr, 0);
                if (status != AMEDIA_OK) {
                    Log::Write(Log::Level::Error, Fmt("Failed to configure codec, code: %ld", status));
                    codec.reset();
                    break;
                }

                status = AMediaCodec_start(codec.get());
                if (status != AMEDIA_OK) {
                    Log::Write(Log::Level::Error, Fmt("Failed to start codec, code: %ld", status));
                    codec.reset();
                    break;
                }
                if (const auto clientCtx = ctx.clientCtx) {
                    //clientCtx->setWaitingNextIDR(true);
                    //clientCtx->requestIDR();
                    if (const auto programPtr = ctx.programPtr) {
                        programPtr->SetRenderMode(IOpenXrProgram::RenderMode::VideoStream);
                    }
                }
                if (!outputThread.Start(codec)) {
                    Log::Write(Log::Level::Error, "Decoder output thread failed to start.");
                    break;
                }
                Log::Write(Log::Level::Info, "Finished constructing and starting decoder...");
                continue;
            }

            if (codec == nullptr)
                continue;
            assert(codec != nullptr);

            while (isRunningToken)
            {
                if (const auto inputBufferId = AMediaCodec_dequeueInputBuffer(codec.get(), QueueWaitTimeout))
                {
                    const auto& packet_data = packet.data;
                    if (packet.is_idr(ctx.config.codecType)) {
                        if (const auto clientCtx = ctx.clientCtx) {
                            clientCtx->setWaitingNextIDR(false);
                            //Log::Write(Log::Level::Verbose, "Finished waiting for next IDR.");
                        }
                    }
                    const bool is_config_packet = packet.is_config(ctx.config.codecType);
                    if (!is_config_packet) {
                        LatencyCollector::Instance().decoderInput(packet.frameIndex);
                    }
                    
                    std::size_t inBuffSize = 0;
                    const auto inputBuffer = AMediaCodec_getInputBuffer(codec.get(), static_cast<std::size_t>(inputBufferId), &inBuffSize);
                    assert(packet_data.size() <= inBuffSize);
                    const std::size_t size = std::min(inBuffSize, packet_data.size());
                    std::memcpy(inputBuffer, packet_data.data(), size);

                    using namespace std::chrono;
                    using ClockType = XrSteadyClock;
                    static_assert(ClockType::is_steady);
                    using microseconds64 = duration<std::uint64_t, microseconds::period>;

                    const auto pts = is_config_packet ? 0 : duration_cast<microseconds64>(ClockType::now().time_since_epoch()).count();
                    const std::uint32_t flags = is_config_packet ? AMEDIACODEC_BUFFER_FLAG_CODEC_CONFIG : 0;
                    if (!is_config_packet) {
                        imgListener.frameIndexMap.set(pts, packet.frameIndex);
                    }

                    const auto result = AMediaCodec_queueInputBuffer(codec.get(), inputBufferId, 0, size, pts, flags);
                    if (result != AMEDIA_OK) {
                        Log::Write(Log::Level::Warning, Fmt("AMediaCodec_queueInputBuffer, error-code %d: ", (int)result));
                    }
                    break;
                }
                else Log::Write(Log::Level::Warning, Fmt("Waiting for decoder input buffer timed out after %f seconds, retrying...", QueueWaitTimeout * 1e-6f));
            }
        }

        outputThread.Stop();
        Log::Write(Log::Level::Info, "Decoder thread exiting...");
        if (codec != nullptr) {
            CHECK(AMediaCodec_stop(codec.get()) == AMEDIA_OK);
        }
        return true;
    }
};
}

std::shared_ptr<IDecoderPlugin> CreateDecoderPlugin_MediaCodec() {
    return std::make_shared<MediaCodecDecoderPlugin>();
}
#endif
