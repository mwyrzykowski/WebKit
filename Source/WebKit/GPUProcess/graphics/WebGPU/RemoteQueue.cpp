/*
 * Copyright (C) 2021-2023 Apple Inc. All rights reserved.
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
#include "RemoteQueue.h"

#if ENABLE(GPU_PROCESS)

#include "GPUConnectionToWebProcess.h"
#include "RemoteQueueMessages.h"
#include "StreamServerConnection.h"
#include "WebGPUObjectHeap.h"
#include <WebCore/CoreVideoSoftLink.h>
#include <WebCore/SharedMemory.h>
#include <WebCore/WebGPUBuffer.h>
#include <WebCore/WebGPUQueue.h>
#include <wtf/TZoneMallocInlines.h>

namespace WebKit {

WTF_MAKE_TZONE_ALLOCATED_IMPL(RemoteQueue);

RemoteQueue::RemoteQueue(GPUConnectionToWebProcess& gpuConnectionToWebProcess, WebCore::WebGPU::Queue& queue, WebGPU::ObjectHeap& objectHeap, Ref<IPC::StreamServerConnection>&& streamConnection, RemoteGPU& gpu, WebGPUIdentifier identifier)
    : m_backing(queue)
    , m_objectHeap(objectHeap)
    , m_streamConnection(WTFMove(streamConnection))
    , m_gpu(gpu)
    , m_gpuConnectionToWebProcess(gpuConnectionToWebProcess)
    , m_identifier(identifier)
{
    Ref { m_streamConnection }->startReceivingMessages(*this, Messages::RemoteQueue::messageReceiverName(), m_identifier.toUInt64());
}

RemoteQueue::~RemoteQueue() = default;

void RemoteQueue::destruct()
{
    protectedObjectHeap()->removeObject(m_identifier);
}

void RemoteQueue::stopListeningForIPC()
{
    Ref { m_streamConnection }->stopReceivingMessages(Messages::RemoteQueue::messageReceiverName(), m_identifier.toUInt64());
}

void RemoteQueue::submit(Vector<WebGPUIdentifier>&& commandBuffers)
{
    Vector<Ref<WebCore::WebGPU::CommandBuffer>> convertedCommandBuffers;
    convertedCommandBuffers.reserveInitialCapacity(commandBuffers.size());
    for (WebGPUIdentifier identifier : commandBuffers) {
        auto convertedCommandBuffer = protectedObjectHeap()->convertCommandBufferFromBacking(identifier);
        ASSERT(convertedCommandBuffer);
        if (!convertedCommandBuffer)
            return;
        convertedCommandBuffers.append(*convertedCommandBuffer);
    }
    protectedBacking()->submit(WTFMove(convertedCommandBuffers));
}

void RemoteQueue::onSubmittedWorkDone(CompletionHandler<void()>&& callback)
{
    protectedBacking()->onSubmittedWorkDone([callback = WTFMove(callback)] () mutable {
        callback();
    });
}

void RemoteQueue::writeBuffer(
    WebGPUIdentifier buffer,
    WebCore::WebGPU::Size64 bufferOffset,
    std::optional<WebCore::SharedMemoryHandle>&& dataHandle,
    CompletionHandler<void(bool)>&& completionHandler)
{
    auto data = dataHandle ? WebCore::SharedMemory::map(WTFMove(*dataHandle), WebCore::SharedMemory::Protection::ReadOnly) : nullptr;
    auto convertedBuffer = protectedObjectHeap()->convertBufferFromBacking(buffer);
    ASSERT(convertedBuffer);
    if (!convertedBuffer) {
        completionHandler(false);
        return;
    }

    protectedBacking()->writeBufferNoCopy(*convertedBuffer, bufferOffset, data ? data->mutableSpan() : std::span<uint8_t> { }, 0, std::nullopt);
    completionHandler(true);
}

void RemoteQueue::writeBufferWithCopy(
    WebGPUIdentifier buffer,
    WebCore::WebGPU::Size64 bufferOffset,
    Vector<uint8_t>&& data)
{
    Ref objectHeap = m_objectHeap.get();
    auto convertedBuffer = objectHeap->convertBufferFromBacking(buffer);
    ASSERT(convertedBuffer);
    if (!convertedBuffer)
        return;

    protectedBacking()->writeBufferNoCopy(*convertedBuffer, bufferOffset, data.mutableSpan(), 0, std::nullopt);
}

void RemoteQueue::writeTexture(
    const WebGPU::ImageCopyTexture& destination,
    std::optional<WebCore::SharedMemoryHandle>&& dataHandle,
    const WebGPU::ImageDataLayout& dataLayout,
    const WebGPU::Extent3D& size,
    CompletionHandler<void(bool)>&& completionHandler)
{
    auto data = dataHandle ? WebCore::SharedMemory::map(WTFMove(*dataHandle), WebCore::SharedMemory::Protection::ReadOnly) : nullptr;
    Ref objectHeap = m_objectHeap.get();
    auto convertedDestination = objectHeap->convertFromBacking(destination);
    ASSERT(convertedDestination);
    auto convertedDataLayout = objectHeap->convertFromBacking(dataLayout);
    ASSERT(convertedDestination);
    auto convertedSize = objectHeap->convertFromBacking(size);
    ASSERT(convertedSize);
    if (!convertedDestination || !convertedDestination || !convertedSize) {
        completionHandler(false);
        return;
    }

    protectedBacking()->writeTexture(*convertedDestination, data ? data->mutableSpan() : std::span<uint8_t> { }, *convertedDataLayout, *convertedSize);
    completionHandler(true);
}

void RemoteQueue::writeTextureWithCopy(
    const WebGPU::ImageCopyTexture& destination,
    Vector<uint8_t>&& data,
    const WebGPU::ImageDataLayout& dataLayout,
    const WebGPU::Extent3D& size)
{
    Ref objectHeap = m_objectHeap.get();
    auto convertedDestination = objectHeap->convertFromBacking(destination);
    ASSERT(convertedDestination);
    auto convertedDataLayout = objectHeap->convertFromBacking(dataLayout);
    ASSERT(convertedDestination);
    auto convertedSize = objectHeap->convertFromBacking(size);
    ASSERT(convertedSize);
    if (!convertedDestination || !convertedDestination || !convertedSize)
        return;

    protectedBacking()->writeTexture(*convertedDestination, data.mutableSpan(), *convertedDataLayout, *convertedSize);
}

void RemoteQueue::copyExternalImageToTexture(
    const WebGPU::ImageCopyExternalImage& source,
    const WebGPU::ImageCopyTextureTagged& destination,
    WebCore::WebGPU::IntegerCoordinate width,
    WebCore::WebGPU::IntegerCoordinate height,
    CompletionHandler<void(bool)>&& completionHandler)
{
    Ref objectHeap = m_objectHeap.get();
    auto convertedSource = objectHeap->convertFromBacking(source);
    ASSERT(convertedSource);
    auto convertedDestination = objectHeap->convertFromBacking(destination);
    ASSERT(convertedDestination);
    if (!convertedDestination || !convertedDestination)
        return;

    RetainPtr<IOSurfaceRef> ioSurface;
    WTF::switchOn(source.source, [&] (const std::optional<WebCore::RenderingResourceIdentifier>& identifier) {
        if (identifier) {
            if (auto imageBuffer = m_gpu->imageBuffer(*identifier)) {
                if (auto webIOSurface = imageBuffer->surface())
                    ioSurface = webIOSurface->surface();
            }
        }
        UNUSED_PARAM(identifier);
    }, [&] (const std::optional<WebCore::MediaPlayerIdentifier>& mediaIdentifier) {
        if (auto connection = m_gpuConnectionToWebProcess.get(); connection && mediaIdentifier) {
            connection->performWithMediaPlayerOnMainThread(*mediaIdentifier, [&] (auto& player) mutable {
                auto videoFrame = player.videoFrameForCurrentTime();
                if (videoFrame)
                    ioSurface = WebCore::CVPixelBufferGetIOSurface(videoFrame->pixelBuffer());
            });
        }
    }, [&] (std::optional<WebKit::SharedVideoFrame> sharedVideoFrame) {
        if (auto videoFrame = m_gpu->sharedVideoFrameReader().read(WTFMove(*sharedVideoFrame)))
            ioSurface = WebCore::CVPixelBufferGetIOSurface(videoFrame->pixelBuffer());
    }, [&] (const std::optional<MachSendRight>& sendRight) {
        ioSurface = IOSurfaceLookupFromMachPort(sendRight->sendRight());
    });

    if (ioSurface.get())
        protectedBacking()->copyExternalImageToTexture(*convertedSource, *convertedDestination, width, height, WTFMove(completionHandler), ioSurface);
    else
        completionHandler(false);
}

void RemoteQueue::setLabel(String&& label)
{
    protectedBacking()->setLabel(WTFMove(label));
}

Ref<WebCore::WebGPU::Queue> RemoteQueue::protectedBacking()
{
    return m_backing;
}

Ref<WebGPU::ObjectHeap> RemoteQueue::protectedObjectHeap() const
{
    return m_objectHeap.get();
}

} // namespace WebKit

#endif // ENABLE(GPU_PROCESS)
