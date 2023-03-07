#pragma once
#ifndef ALXR_VRCFT_PROXY_SERVER_H

#include <cstdint>
#include <array>
#include <memory>
#include <functional>
#include <asio/buffer.hpp>
#include <asio/ts/internet.hpp>

namespace ALXR::VRCFT {

    using asio::ip::tcp;

    struct Session final : public std::enable_shared_from_this<Session>
    {
        inline Session(tcp::socket&& socket)
        : m_socket(std::move(socket))
        {}

        inline Session(const Session&) = delete;
        inline Session(Session&&) = default;
        inline Session& operator=(const Session&) = delete;
        inline Session& operator=(Session&&) = default;

        inline void Close() {
            Log::Write(Log::Level::Info, "VRCFTServer: shutting down connection.");
            try {
                m_socket.shutdown(tcp::socket::shutdown_both);
            }
            catch (const asio::system_error& sysError) {
                Log::Write(Log::Level::Warning, Fmt("VRCFTServer: Failed to shutdown gracefully, reason: \"%s\"", sysError.what()));
            }
            try {
                m_socket.close();
            }
            catch (const asio::system_error& sysError) {
                Log::Write(Log::Level::Error, Fmt("VRCFTServer: Failed to close connection, reason: \"%s\"", sysError.what()));
            }
            Log::Write(Log::Level::Info, "VRCFTServer: connection closed.");
        }

        inline ~Session() {
            Close();
        }

        template < const std::size_t N >
        using Buffer = std::array < std::uint8_t, N >;

        template < const std::size_t N >
        inline bool send(const Buffer<N>& buffer)
        {
            return m_socket.send(asio::buffer(buffer)) == buffer.size();
        }

        template < typename Tp >
        inline bool send(const Tp& buffer)
        {
            return m_socket.send
            (
                asio::buffer(&buffer, sizeof(buffer))
            ) == sizeof(buffer);
        }

    private:
        tcp::socket m_socket;
    };

    struct Server
    {
        constexpr static const std::uint16_t PortNo = 13191;

        inline Server()
        : m_acceptor(m_ioContext, tcp::endpoint(tcp::v4(), PortNo)),
          m_socket(m_ioContext)
        {
            AsyncAccept();
        }

        inline Server(const Server&) = delete;
        inline Server(Server&&) = default;

        inline Server& operator=(const Server&) = delete;
        inline Server& operator=(Server&&) = default;

        inline void PollOne() {
            m_ioContext.poll_one();
        }

        inline void Poll() {
            m_ioContext.poll();
        }

        inline bool IsConnected() const {
            return m_session != nullptr;
        }

        template < const std::size_t N >
        using Buffer = Session::Buffer<N>;

        template < typename Tp >
        inline bool Send(const Tp& buf) {
            if (!IsConnected())
                return false;
            try {
                return m_session->send(buf);
            }
            catch (const asio::system_error& sysError) {
                Log::Write(Log::Level::Warning, Fmt("VRCFTServer: Failed to send, reason: \"%s\"", sysError.what()));
                m_session.reset();
                return false;
            }
        }

        void Close() {
            try {
                Log::Write(Log::Level::Info, "VRCFTServer: shutting down server.");
                m_session.reset();
                m_socket.close();
                m_acceptor.close();
                m_ioContext.stop();
                Log::Write(Log::Level::Info, "VRCFTServer: server shutdown.");
            }
            catch (const asio::system_error& sysError) {
                Log::Write(Log::Level::Error, Fmt("VRCFTServer: Failed to cleanly shutdown server, reason: \"%s\"", sysError.what()));
            }
        }

        template < typename Fun >
        inline void SetOnNewConnection(Fun&& fn) {
            m_onNewConnectionFn = fn;
        }

    private:
        using SessionPtr = std::shared_ptr<Session>;
        using OnNewConFn = std::function<void()>;

        inline void AsyncAccept() {
            m_acceptor.async_accept(m_socket, [this](std::error_code ec) {
                if (!ec) {
                    Log::Write(Log::Level::Info, "VRCFTServer: connection accepted.");
                    m_socket.set_option(tcp::no_delay(true));
                    m_session = std::make_shared<Session>(std::move(m_socket));
                    if (m_onNewConnectionFn)
                        m_onNewConnectionFn();
                }
                AsyncAccept();
            });
        }

        asio::io_context m_ioContext{};
        tcp::acceptor    m_acceptor;
        tcp::socket      m_socket;
        SessionPtr       m_session{ nullptr };
        OnNewConFn       m_onNewConnectionFn{};
    };
}
#endif
