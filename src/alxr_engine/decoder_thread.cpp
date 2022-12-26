#include "decoder_thread.h"
#include "logger.h"
#include "decoderplugin.h"
#include "latency_manager.h"

bool XrDecoderThread::QueuePacket(const VideoFrame& header, const std::size_t packetSize)
{
	const auto decoderPlugin = m_decoderPlugin;
	if (decoderPlugin == nullptr)
		return false;
	LatencyManager::Instance().OnPreVideoPacketRecieved(header);

	bool fecFailure = false, isComplete = true;
	if (const auto fecQueue = m_fecQueue) {
		fecQueue->addVideoPacket(&header, static_cast<int>(packetSize), fecFailure);
		if (isComplete = fecQueue->reconstruct()) {
			const size_t frameBufferSize = fecQueue->getFrameByteSize();
			const auto frameBufferPtr = reinterpret_cast<const std::uint8_t*>(fecQueue->getFrameBuffer());
			decoderPlugin->QueuePacket({ frameBufferPtr, frameBufferSize }, header.trackingFrameIndex);
			fecQueue->clearFecFailure();
		}
	} else { // then FEC is disabled
		const size_t frameBufferSize = packetSize - sizeof(VideoFrame);
		const auto frameBufferPtr = reinterpret_cast<const std::uint8_t*>(&header) + sizeof(VideoFrame);
		decoderPlugin->QueuePacket({ frameBufferPtr, frameBufferSize }, header.trackingFrameIndex);
	}

	LatencyManager::Instance().OnPostVideoPacketRecieved(header, { isComplete, fecFailure });
	return true;
}

void XrDecoderThread::Stop()
{
	Log::Write(Log::Level::Info, "shutting down decoder thread");
	m_isRuningToken = false;
	if (m_decoderThread.joinable()) {
		Log::Write(Log::Level::Info, "Waiting for decoder thread to shutdown...");
		m_decoderThread.join();
	}
	m_fecQueue.reset();

	Log::Write(Log::Level::Info, "m_decoderPlugin destroying");
	m_decoderPlugin.reset();
	Log::Write(Log::Level::Info, "m_decoderPlugin destroyed");
	
	Log::Write(Log::Level::Info, "Decoder thread finished shutdown");
}

void XrDecoderThread::Start(const XrDecoderThread::StartCtx& ctx)
{
	if (m_isRuningToken)
		return;

	Log::Write(Log::Level::Info, "Starting decoder thread.");
	m_fecQueue = ctx.decoderConfig.enableFEC ?
		std::make_shared<FECQueue>() : nullptr;
	m_decoderPlugin = CreateDecoderPlugin();
	LatencyManager::Instance().ResetAll();
#ifdef XR_USE_PLATFORM_WIN32
	auto decoderType = ALXRDecoderType::D311VA;
#else
	auto decoderType = ALXRDecoderType::VAAPI;
#endif
	if (const auto rustCtx = ctx.rustCtx) {
		decoderType = rustCtx->decoderType;
		Log::Write(Log::Level::Verbose, "Sending IDR request");
		rustCtx->setWaitingNextIDR(true);
		rustCtx->requestIDR();
	}

#ifndef XR_DISABLE_DECODER_THREAD
	m_isRuningToken = true;
	m_decoderThread = std::thread
	{
		[=, startCtx = ctx]()
		{
			OptionMap optionMap{};
#ifdef XR_USE_PLATFORM_ANDROID
			//// Exynos
			optionMap.setInt32("vendor.rtc-ext-dec-low-latency.enable", 1);
			//// qualcom,e.g. Quest 1/2 hw decoder.
			optionMap.setInt32("vendor.qti-ext-dec-low-latency.enable", 1);
			//// AMD
			optionMap.setInt32("output-decode-order", 1);
			optionMap.setInt32("decode-low-latency", 1);
#endif
			const IDecoderPlugin::RunCtx runCtx {
				.optionMap	 = std::move(optionMap),
				.config		 = startCtx.decoderConfig,
				.rustCtx	 = startCtx.rustCtx,
				.programPtr	 = startCtx.programPtr,
				.decoderType = decoderType
			};
			m_decoderPlugin->Run(runCtx, m_isRuningToken);

			Log::Write(Log::Level::Info, "Decoder thread exiting.");
		}
	};
	Log::Write(Log::Level::Info, "Decoder Thread started.");
#endif
}
