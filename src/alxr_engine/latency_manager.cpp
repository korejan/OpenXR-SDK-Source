#include "pch.h"
#include "common.h"
#include "latency_manager.h"
#include <cstdlib>
#include <chrono>
#include "timing.h"
#include "packet_types.h"

LatencyManager LatencyManager::m_instance{};

void LatencyManager::OnTimeSyncRecieved(const TimeSync& timeSync)
{
    if (timeSync.mode == 1) {
        LatencyCollector::Instance().setTotalLatency(timeSync.serverTotalLatency);
        const std::uint64_t Current = GetSystemTimestampUs();
        const std::uint64_t RTT = Current - timeSync.clientTime;
        m_rt_state.timeDiff =
            ((std::int64_t)timeSync.serverTime + (std::int64_t)RTT / 2) - (std::int64_t)Current;
        //LOG("TimeSync: server - client = %ld us RTT = %lu us", m_rt_state.timeDiff, RTT);
        if (m_callbackCtx.timeSyncSendFn) {
            TimeSync sendBuf = timeSync;
            sendBuf.mode = 2;
            sendBuf.clientTime = Current;
            m_callbackCtx.timeSyncSendFn(&sendBuf);
        }
    }
    if (timeSync.mode == 3)
        LatencyCollector::Instance().received(timeSync.trackingRecvFrameIndex);
}

void LatencyManager::OnPreVideoPacketRecieved(const VideoFrame& header)
{
    if (m_rt_state.lastFrameIndex != header.trackingFrameIndex) {
        LatencyCollector::Instance().receivedFirst(header.trackingFrameIndex);
        const std::int64_t timeDiff = m_rt_state.timeDiff.load();
        const auto diff = static_cast<std::int64_t>(header.sentTime) - timeDiff;
        const auto timeStamp = static_cast<std::int64_t>(GetSystemTimestampUs());
        const auto offset = diff > timeStamp ?
            0 : ((std::int64_t)header.sentTime - timeDiff - timeStamp);
        LatencyCollector::Instance().estimatedSent(header.trackingFrameIndex, offset);
        m_rt_state.lastFrameIndex = header.trackingFrameIndex;
    }
    if (const auto lostCount = ProcessVideoSeq(header))
        LatencyCollector::Instance().packetLoss(lostCount);
}

void LatencyManager::OnPostVideoPacketRecieved
(
    const VideoFrame& header,
    const LatencyManager::PacketRecievedStatus& status
)
{
    if (status.complete)
        LatencyCollector::Instance().receivedLast(header.trackingFrameIndex);
    if (status.fecFailed) {
        LatencyCollector::Instance().fecFailure();
        SendPacketLossReport(0, 0);
    }
}

std::int64_t LatencyManager::ProcessVideoSeq(const VideoFrame& header)
{
    const auto nextSeq = m_rt_state.prevVideoSequence + 1;
    const bool isPacketsLost = m_rt_state.prevVideoSequence != 0 && nextSeq != header.packetCounter;
    m_rt_state.prevVideoSequence = header.packetCounter;
    return isPacketsLost ?
        std::abs(static_cast<std::int32_t>(header.packetCounter - nextSeq)) : 0;
}

void LatencyManager::SendPacketLossReport
(
    const std::uint32_t /*fromPacketCounter*/,
    const std::uint32_t /*toPacketCounter*/
)
{
    if (m_callbackCtx.videoErrorReportSendFn)
        m_callbackCtx.videoErrorReportSendFn();
}

void LatencyManager::SendTimeSync() {
    if (m_callbackCtx.timeSyncSendFn == nullptr)
        return;
    TimeSync timeSync
    {
        .type = ALVR_PACKET_TYPE_TIME_SYNC,
        .mode = 0,
        .sequence = ++m_timeSyncSequence,

        .packetsLostTotal = LatencyCollector::Instance().getPacketsLostTotal(),
        .packetsLostInSecond = LatencyCollector::Instance().getPacketsLostInSecond(),

        .averageTotalLatency = (uint32_t)LatencyCollector::Instance().getLatency(0),

        .averageSendLatency = (uint32_t)LatencyCollector::Instance().getLatency(3),

        .averageTransportLatency = (uint32_t)LatencyCollector::Instance().getLatency(1),

        .averageDecodeLatency = (uint64_t)LatencyCollector::Instance().getLatency(2),

        .idleTime = (uint32_t)LatencyCollector::Instance().getLatency(4),

        .fecFailure = m_rt_state.isFecFailed.load(),
        .fecFailureInSecond = LatencyCollector::Instance().getFecFailureInSecond(),
        .fecFailureTotal = LatencyCollector::Instance().getFecFailureTotal(),

        .fps = LatencyCollector::Instance().getFramesInSecond()
    };
    timeSync.clientTime = GetSystemTimestampUs();
    m_callbackCtx.timeSyncSendFn(&timeSync);
}

void LatencyManager::SendFrameReRenderTimeSync() {
    if (m_callbackCtx.timeSyncSendFn == nullptr)
        return;
    TimeSync timeSync
    {
        .type = ALVR_PACKET_TYPE_TIME_SYNC,
        .mode = 0,
        .sequence = ++m_timeSyncSequence,

        .packetsLostTotal = LatencyCollector::Instance().getPacketsLostTotal(),
        .packetsLostInSecond = LatencyCollector::Instance().getPacketsLostInSecond(),

        .averageTotalLatency = 0,
        .averageSendLatency = 0,
        .averageTransportLatency = 0,
        .averageDecodeLatency = 0,
        .idleTime = 0,

        .fecFailure = m_rt_state.isFecFailed.load(),
        .fecFailureInSecond = LatencyCollector::Instance().getFecFailureInSecond(),
        .fecFailureTotal = LatencyCollector::Instance().getFecFailureTotal(),

        .fps = LatencyCollector::Instance().getFramesInSecond()
    };
    timeSync.clientTime = GetSystemTimestampUs();
    m_callbackCtx.timeSyncSendFn(&timeSync);
}
