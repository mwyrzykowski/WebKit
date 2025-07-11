/*
 * Copyright (C) 2017 Apple Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY APPLE INC. AND ITS CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL APPLE INC. OR ITS CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#if !PLATFORM(COCOA)

#if USE(LIBWEBRTC)

#include "NetworkRTCProvider.h"

WTF_IGNORE_WARNINGS_IN_THIRD_PARTY_CODE_BEGIN
#include <webrtc/rtc_base/async_packet_socket.h>
WTF_IGNORE_WARNINGS_IN_THIRD_PARTY_CODE_END

#include <wtf/TZoneMalloc.h>

namespace webrtc {
class AsyncPacketSocket;
struct SentPacket;
typedef int64_t PacketTime;
}

namespace WebKit {

class LibWebRTCSocketClient final : public NetworkRTCProvider::Socket, public sigslot::has_slots<> {
    WTF_MAKE_TZONE_ALLOCATED(LibWebRTCSocketClient);
public:
    LibWebRTCSocketClient(WebCore::LibWebRTCSocketIdentifier, NetworkRTCProvider&, std::unique_ptr<webrtc::AsyncPacketSocket>&&, Type, Ref<IPC::Connection>&&);

private:
    WebCore::LibWebRTCSocketIdentifier identifier() const final { return m_identifier; }
    Type type() const final { return m_type; }
    void close() final;

    void setOption(int option, int value) final;
    void sendTo(std::span<const uint8_t>, const webrtc::SocketAddress&, const webrtc::AsyncSocketPacketOptions&) final;

    void signalReadPacket(webrtc::AsyncPacketSocket*, const unsigned char*, size_t, const webrtc::SocketAddress&, int64_t);
    void signalSentPacket(webrtc::AsyncPacketSocket*, const webrtc::SentPacketInfo&);
    void signalAddressReady(webrtc::AsyncPacketSocket*, const webrtc::SocketAddress&);
    void signalConnect(webrtc::AsyncPacketSocket*);
    void signalClose(webrtc::AsyncPacketSocket*, int);

    void signalAddressReady();

    WebCore::LibWebRTCSocketIdentifier m_identifier;
    Type m_type;
    CheckedRef<NetworkRTCProvider> m_rtcProvider;
    std::unique_ptr<webrtc::AsyncPacketSocket> m_socket;
    const Ref<IPC::Connection> m_connection;
    int m_sendError { 0 };
};

} // namespace WebKit

#endif // USE(LIBWEBRTC)

#endif // !PLATFORM(COCOA)
