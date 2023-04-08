#pragma once
#ifndef ALXR_LATENCY_MANAGER_H
#define ALXR_LATENCY_MANAGER_H

#include "latency_collector.h"
#include <cstdint>
#include <atomic>
#include <mutex>
#include <shared_mutex>

struct VideoFrame;
struct TimeSync;
struct TrackingInfo;

struct LatencyManager
{
	void OnPreVideoPacketRecieved(const VideoFrame& header);

	struct PacketRecievedStatus
	{
		bool complete;
		bool fecFailed;
	};
	void OnPostVideoPacketRecieved
	(
		const VideoFrame& header,
		const PacketRecievedStatus& status
	);
	void OnTimeSyncRecieved(const TimeSync& timeSync);

	inline void SubmitAndSync(const std::uint64_t frameIndex, const bool reRenderOnly = false)
	{
		if (frameIndex == std::uint64_t(-1))
			return;
		LatencyCollector::Instance().submit(frameIndex);
		if (reRenderOnly)
			SendFrameReRenderTimeSync();
		else
			SendTimeSync();
	}

	inline void ResetAll() {
		m_rt_state.isFecFailed = false;
		m_rt_state.prevVideoSequence = 0;
		m_rt_state.lastFrameIndex = 0;
		m_rt_state.timeDiff = 0;
		m_timeSyncSequence = uint64_t(-1);
		LatencyCollector::Instance().resetAll();
	}
	
	using SendFn = void (*)(const TrackingInfo* data);
	using TimeSyncSendFn = void (*)(const TimeSync* data);
	using VideoErrorReportSendFn = void (*)();
	struct CallbackCtx {
		SendFn					sendFn;
		TimeSyncSendFn			timeSyncSendFn;
		VideoErrorReportSendFn	videoErrorReportSendFn;
	};
	void Init(const CallbackCtx& ctx)
	{
		m_callbackCtx = ctx;
		ResetAll();
	}
	static LatencyManager& Instance() { return m_instance; }

private:
	std::int64_t ProcessVideoSeq(const VideoFrame& header);
	void SendPacketLossReport
	(
		const std::uint32_t fromPacketCounter,
		const std::uint32_t toPacketCounter
	);
	void SendTimeSync();
	void SendFrameReRenderTimeSync();

	CallbackCtx m_callbackCtx {
		.sendFn = nullptr,
		.timeSyncSendFn = nullptr
	};

	std::uint64_t m_timeSyncSequence = uint64_t(-1);
	struct RecieveThreadState
	{
		std::atomic<std::int64_t>  timeDiff{ 0 };
		std::uint64_t lastFrameIndex = 0;
		std::uint32_t prevVideoSequence = 0;
		std::atomic<bool> isFecFailed{ false };
	};
	RecieveThreadState m_rt_state{};

	static LatencyManager m_instance;
};
#endif //ALXR_LATENCY_MANAGER_H
