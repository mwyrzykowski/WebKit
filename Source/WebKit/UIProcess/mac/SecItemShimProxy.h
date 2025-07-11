/*
 * Copyright (C) 2013-2025 Apple Inc. All rights reserved.
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

#if ENABLE(SEC_ITEM_SHIM)

#include "Connection.h"
#include "WorkQueueMessageReceiver.h"

namespace WebKit {

class SecItemRequestData;
class SecItemResponseData;

class SecItemShimProxy final : private IPC::MessageReceiver {
WTF_MAKE_NONCOPYABLE(SecItemShimProxy);
public:
    static SecItemShimProxy& singleton();

    void initializeConnection(IPC::Connection&);

    // Do nothing since this is a singleton.
    void ref() const final { }
    void deref() const final { }

private:
    SecItemShimProxy();
    ~SecItemShimProxy();

    // IPC::Connection::MessageReceiver overrides.
    void didReceiveMessage(IPC::Connection&, IPC::Decoder&) override;
    bool didReceiveSyncMessage(IPC::Connection&, IPC::Decoder&, UniqueRef<IPC::Encoder>&) override;

    void secItemRequest(IPC::Connection&, const SecItemRequestData&, CompletionHandler<void(std::optional<SecItemResponseData>&&)>&&);
    void secItemRequestSync(IPC::Connection&, const SecItemRequestData&, CompletionHandler<void(std::optional<SecItemResponseData>&&)>&&);

    const Ref<WorkQueue> m_queue;
};

} // namespace WebKit

#endif // ENABLE(SEC_ITEM_SHIM)
