/*
 * Copyright (C) 2021 Apple Inc. All rights reserved.
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

#include "config.h"
#include "RemoteImageDecoderAVFManager.h"

#if ENABLE(GPU_PROCESS) && HAVE(AVASSETREADER)

#include "GPUProcessConnection.h"
#include "RemoteImageDecoderAVF.h"
#include "RemoteImageDecoderAVFManagerMessages.h"
#include "RemoteImageDecoderAVFProxyMessages.h"
#include "SharedBufferReference.h"
#include "WebProcess.h"
#include <wtf/TZoneMallocInlines.h>

namespace WebKit {

using namespace WebCore;

WTF_MAKE_TZONE_ALLOCATED_IMPL(RemoteImageDecoderAVFManager);

RefPtr<RemoteImageDecoderAVF> RemoteImageDecoderAVFManager::createImageDecoder(FragmentedSharedBuffer& data, const String& mimeType, AlphaOption alphaOption, GammaAndColorProfileOption gammaAndColorProfileOption)
{
    if (!WebProcess::singleton().mediaPlaybackEnabled())
        return nullptr;

    auto sendResult = ensureGPUProcessConnection().connection().sendSync(Messages::RemoteImageDecoderAVFProxy::CreateDecoder(IPC::SharedBufferReference(data), mimeType), 0);
    auto [imageDecoderIdentifier] = sendResult.takeReplyOr(std::nullopt);
    if (!imageDecoderIdentifier)
        return nullptr;

    auto remoteImageDecoder = RemoteImageDecoderAVF::create(*this, *imageDecoderIdentifier, data, mimeType);
    m_remoteImageDecoders.add(*imageDecoderIdentifier, remoteImageDecoder);

    return remoteImageDecoder;
}

void RemoteImageDecoderAVFManager::deleteRemoteImageDecoder(const ImageDecoderIdentifier& identifier)
{
    m_remoteImageDecoders.take(identifier);
    if (auto gpuProcessConnection = m_gpuProcessConnection.get())
        gpuProcessConnection->connection().send(Messages::RemoteImageDecoderAVFProxy::DeleteDecoder(identifier), 0);
}

Ref<RemoteImageDecoderAVFManager> RemoteImageDecoderAVFManager::create()
{
    return adoptRef(*new RemoteImageDecoderAVFManager);
}

RemoteImageDecoderAVFManager::RemoteImageDecoderAVFManager() = default;

RemoteImageDecoderAVFManager::~RemoteImageDecoderAVFManager()
{
    if (auto gpuProcessConnection = m_gpuProcessConnection.get())
        gpuProcessConnection->messageReceiverMap().removeMessageReceiver(Messages::RemoteImageDecoderAVFManager::messageReceiverName());
}

void RemoteImageDecoderAVFManager::gpuProcessConnectionDidClose(GPUProcessConnection& connection)
{
    ASSERT(m_gpuProcessConnection.get() == &connection);
    if (auto gpuProcessConnection = m_gpuProcessConnection.get())
        gpuProcessConnection->messageReceiverMap().removeMessageReceiver(Messages::RemoteImageDecoderAVFManager::messageReceiverName());
    m_gpuProcessConnection = nullptr;
    // FIXME: Do we need to do more when m_remoteImageDecoders is not empty to re-create them?
}

GPUProcessConnection& RemoteImageDecoderAVFManager::ensureGPUProcessConnection()
{
    RefPtr gpuProcessConnection = m_gpuProcessConnection.get();
    if (!gpuProcessConnection) {
        gpuProcessConnection = WebProcess::singleton().ensureGPUProcessConnection();
        m_gpuProcessConnection = gpuProcessConnection;
        gpuProcessConnection->addClient(*this);
        gpuProcessConnection->messageReceiverMap().addMessageReceiver(Messages::RemoteImageDecoderAVFManager::messageReceiverName(), *this);
    }
    return *gpuProcessConnection;
}

void RemoteImageDecoderAVFManager::setUseGPUProcess(bool useGPUProcess)
{
    if (!useGPUProcess) {
        ImageDecoder::resetFactories();
        return;
    }

    ImageDecoder::clearFactories();
    ImageDecoder::installFactory({
        RemoteImageDecoderAVF::supportsMediaType,
        RemoteImageDecoderAVF::canDecodeType,
        [weakThis = ThreadSafeWeakPtr { *this }](FragmentedSharedBuffer& data, const String& mimeType, AlphaOption alphaOption, GammaAndColorProfileOption gammaAndColorProfileOption) {
            RefPtr protectedThis = weakThis.get();
            return protectedThis ? protectedThis->createImageDecoder(data, mimeType, alphaOption, gammaAndColorProfileOption) : nullptr;
        }
    });
}

void RemoteImageDecoderAVFManager::encodedDataStatusChanged(const ImageDecoderIdentifier& identifier, uint64_t frameCount, const WebCore::IntSize& size, bool hasTrack)
{
    if (!m_remoteImageDecoders.contains(identifier))
        return;

    auto remoteImageDecoder = m_remoteImageDecoders.get(identifier);
    if (!remoteImageDecoder)
        return;

    remoteImageDecoder->encodedDataStatusChanged(frameCount, size, hasTrack);
}

}

#endif
