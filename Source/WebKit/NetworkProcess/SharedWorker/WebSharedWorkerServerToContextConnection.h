/*
 * Copyright (C) 2022 Apple Inc. All rights reserved.
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

#include "MessageReceiver.h"
#include "MessageSender.h"
#include "SharedPreferencesForWebProcess.h"
#include "WebPageProxyIdentifier.h"
#include <WebCore/SharedWorkerIdentifier.h>
#include <WebCore/SharedWorkerObjectIdentifier.h>
#include <WebCore/Site.h>
#include <WebCore/Timer.h>
#include <WebCore/TransferredMessagePort.h>
#include <wtf/CheckedRef.h>
#include <wtf/RefCounted.h>
#include <wtf/TZoneMalloc.h>

namespace WebKit {
class WebSharedWorkerServerToContextConnection;
}

namespace WebCore {
class ScriptBuffer;
struct ClientOrigin;
struct WorkerFetchResult;
struct WorkerOptions;
}

namespace WebKit {

class NetworkConnectionToWebProcess;
class WebSharedWorker;
class WebSharedWorkerServer;
struct SharedPreferencesForWebProcess;

class WebSharedWorkerServerToContextConnection final : public IPC::MessageSender, public IPC::MessageReceiver, public RefCounted<WebSharedWorkerServerToContextConnection> {
    WTF_MAKE_TZONE_ALLOCATED(WebSharedWorkerServerToContextConnection);
public:
    static Ref<WebSharedWorkerServerToContextConnection> create(NetworkConnectionToWebProcess&, const WebCore::Site&, WebSharedWorkerServer&);

    ~WebSharedWorkerServerToContextConnection();

    void ref() const final { RefCounted::ref(); }
    void deref() const final { RefCounted::deref(); }

    std::optional<WebCore::ProcessIdentifier> webProcessIdentifier() const;
    const WebCore::RegistrableDomain& registrableDomain() const { return m_site.domain(); }
    const WebCore::Site& site() const { return m_site; }
    IPC::Connection* ipcConnection() const;

    void terminateWhenPossible() { m_shouldTerminateWhenPossible = true; }

    void launchSharedWorker(WebSharedWorker&);
    void postConnectEvent(const WebSharedWorker&, const WebCore::TransferredMessagePort&, CompletionHandler<void(bool)>&&);
    void terminateSharedWorker(const WebSharedWorker&);

    void suspendSharedWorker(WebCore::SharedWorkerIdentifier);
    void resumeSharedWorker(WebCore::SharedWorkerIdentifier);

    const HashMap<WebCore::ProcessIdentifier, HashSet<WebCore::SharedWorkerObjectIdentifier>>& sharedWorkerObjects() const { return m_sharedWorkerObjects; }

    void didReceiveMessage(IPC::Connection&, IPC::Decoder&) final;

    void addSharedWorkerObject(WebCore::SharedWorkerObjectIdentifier);
    void removeSharedWorkerObject(WebCore::SharedWorkerObjectIdentifier);

    std::optional<SharedPreferencesForWebProcess> sharedPreferencesForWebProcess() const;

private:
    WebSharedWorkerServerToContextConnection(NetworkConnectionToWebProcess&, const WebCore::Site&, WebSharedWorkerServer&);

    void idleTerminationTimerFired();
    void connectionIsNoLongerNeeded();

    // IPC messages.
    void postErrorToWorkerObject(WebCore::SharedWorkerIdentifier, const String& errorMessage, int lineNumber, int columnNumber, const String& sourceURL, bool isErrorEvent);
    void sharedWorkerTerminated(WebCore::SharedWorkerIdentifier);

    // IPC::MessageSender.
    IPC::Connection* messageSenderConnection() const final;
    uint64_t messageSenderDestinationID() const final;

    WeakPtr<NetworkConnectionToWebProcess> m_connection;
    WeakPtr<WebSharedWorkerServer> m_server;
    WebCore::Site m_site;
    HashMap<WebCore::ProcessIdentifier, HashSet<WebCore::SharedWorkerObjectIdentifier>> m_sharedWorkerObjects;
    WebCore::Timer m_idleTerminationTimer;
    bool m_shouldTerminateWhenPossible { false };
};

} // namespace WebKit
