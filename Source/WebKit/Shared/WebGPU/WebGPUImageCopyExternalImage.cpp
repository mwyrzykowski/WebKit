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
#include "WebGPUImageCopyExternalImage.h"

#if ENABLE(GPU_PROCESS)

#include "SharedVideoFrame.h"
#include "WebGPUConvertFromBackingContext.h"
#include "WebGPUConvertToBackingContext.h"
#include <WebCore/BitmapImage.h>
#include <WebCore/CachedImage.h>
#include <WebCore/NativeImage.h>
#include <WebCore/SVGImage.h>
#include <WebCore/WebGPUImageCopyExternalImage.h>

namespace WebKit::WebGPU {

static std::optional<SourceType> convertSourceTypeToBacking(const WebCore::WebGPU::SourceType& sourceType, RefPtr<WebCore::VideoFrame>& videoFrame)
{
    return WTF::switchOn(sourceType, [&] (const RefPtr<WebCore::ImageBitmap>& imageBitmap) -> std::optional<SourceType> {
        return imageBitmap->buffer()->renderingResourceIdentifier();
#if ENABLE(VIDEO) && ENABLE(WEB_CODECS)
    }, [&] (const RefPtr<WebCore::ImageData>&) -> std::optional<SourceType> {
        return std::nullopt;
    }, [&] (const RefPtr<WebCore::HTMLImageElement>& imageElement) -> std::optional<SourceType> {
        auto* cachedImage = imageElement->cachedImage();
        if (!cachedImage)
            return std::nullopt;

        RefPtr image = dynamicDowncast<WebCore::BitmapImage>(cachedImage->image());
        RefPtr<WebCore::NativeImage> nativeImage;
        if (image)
            nativeImage = image->nativeImage();
        else {
            RefPtr svgImage = dynamicDowncast<WebCore::SVGImage>(cachedImage->image());
            nativeImage = svgImage->nativeImage(svgImage->size());
        }
        if (!nativeImage)
            return std::nullopt;

        return nativeImage->renderingResourceIdentifier();
    }, [&] (const RefPtr<WebCore::HTMLVideoElement>& videoElement) -> std::optional<SourceType> {
        if (auto playerIdentifier = videoElement->playerIdentifier())
            return playerIdentifier;
        if (videoElement->player())
            videoFrame = videoElement->protectedPlayer()->videoFrameForCurrentTime();

        return std::nullopt;
    }, [&] (const RefPtr<WebCore::WebCodecsVideoFrame>& webCodecsVideoFrame) -> std::optional<SourceType> {
        videoFrame = webCodecsVideoFrame->internalFrame();

        return std::nullopt;
#endif
#if ENABLE(OFFSCREEN_CANVAS)
    }, [&] (const RefPtr<WebCore::OffscreenCanvas>& canvasElement) -> std::optional<SourceType> {
        return canvasElement->makeRenderingResultsAvailable(WebCore::ShouldApplyPostProcessingToDirtyRect::No)->renderingResourceIdentifier();
#endif
    }, [&] (const RefPtr<WebCore::HTMLCanvasElement>& canvasElement) -> std::optional<SourceType> {
        return canvasElement->makeRenderingResultsAvailable(WebCore::ShouldApplyPostProcessingToDirtyRect::No)->renderingResourceIdentifier();
    });
}

std::optional<ImageCopyExternalImage> ConvertToBackingContext::convertToBacking(const WebCore::WebGPU::ImageCopyExternalImage& imageCopyExternalImage, RefPtr<WebCore::VideoFrame>& videoFrame)
{
    std::optional<Origin2D> origin;
    if (imageCopyExternalImage.origin) {
        origin = convertToBacking(*imageCopyExternalImage.origin);
        if (!origin)
            return std::nullopt;
    }

    auto sourceType = convertSourceTypeToBacking(imageCopyExternalImage.source, videoFrame);
    if (!sourceType)
        return std::nullopt;

    return { { *sourceType, WTFMove(origin), imageCopyExternalImage.flipY } };
}

static WebCore::WebGPU::SourceType convertSourceTypeFromBacking(const SourceType& sourceType)
{
    RefPtr<WebCore::ImageBitmap> nullBitmap;
    return nullBitmap;
}

std::optional<WebCore::WebGPU::ImageCopyExternalImage> ConvertFromBackingContext::convertFromBacking(const ImageCopyExternalImage& imageCopyExternalImage)
{
    std::optional<WebCore::WebGPU::Origin2D> origin;
    if (imageCopyExternalImage.origin) {
        origin = convertFromBacking(*imageCopyExternalImage.origin);
        if (!origin)
            return std::nullopt;
    }

    return { { convertSourceTypeFromBacking(imageCopyExternalImage.source), WTFMove(origin), imageCopyExternalImage.flipY } };
}

} // namespace WebKit

#endif // ENABLE(GPU_PROCESS)
