/*
 * Copyright (C) 2022 Apple Inc. All rights reserved.
 * Copyright (C) 2023 Igalia S.L
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
#include "AudioEncoder.h"

#if ENABLE(WEB_CODECS)

#include <wtf/UniqueRef.h>

#if USE(GSTREAMER)
#include "AudioEncoderGStreamer.h"
#elif USE(AVFOUNDATION)
#include "AudioEncoderCocoa.h"
#endif

namespace WebCore {

Ref<AudioEncoder::CreatePromise> AudioEncoder::create(const String& codecName, const Config& config, DescriptionCallback&& descriptionCallback, OutputCallback&& outputCallback)
{
#if USE(GSTREAMER)
    return GStreamerAudioEncoder::create(codecName, config, WTFMove(descriptionCallback), WTFMove(outputCallback));
#elif USE(AVFOUNDATION)
    return AudioEncoderCocoa::create(codecName, config, WTFMove(descriptionCallback), WTFMove(outputCallback));
#else
    UNUSED_PARAM(codecName);
    UNUSED_PARAM(config);
    UNUSED_PARAM(descriptionCallback);
    UNUSED_PARAM(outputCallback);

    return CreatePromise::createAndReject("Not supported"_s));
#endif
}

} // namespace WebCore

#endif // ENABLE(WEB_CODECS)
