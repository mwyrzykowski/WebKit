/*
 * Copyright (C) 2018 Apple Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1.  Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 * 2.  Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY APPLE INC. AND ITS CONTRIBUTORS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL APPLE INC. OR ITS CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#if ENABLE(WEB_RTC) && USE(LIBWEBRTC)

#include "LibWebRTCMacros.h"
#include <wtf/CompletionHandler.h>
#include <wtf/RefPtr.h>

WTF_IGNORE_WARNINGS_IN_THIRD_PARTY_CODE_BEGIN

// See Bug 274508: Disable thread-safety-reference-return warnings in libwebrtc
IGNORE_CLANG_WARNINGS_BEGIN("thread-safety-reference-return")
#include <webrtc/pc/rtc_stats_collector.h>
IGNORE_CLANG_WARNINGS_END

WTF_IGNORE_WARNINGS_IN_THIRD_PARTY_CODE_END

namespace WebCore {

class DOMMapAdapter;
class RTCStatsReport;

void initializeRTCStatsReportBackingMap(RTCStatsReport&);

class LibWebRTCStatsCollector : public webrtc::RTCStatsCollectorCallback {
public:
    using CollectorCallback = CompletionHandler<void(const webrtc::scoped_refptr<const webrtc::RTCStatsReport>&)>;
    static webrtc::scoped_refptr<LibWebRTCStatsCollector> create(CollectorCallback&& callback) { return webrtc::make_ref_counted<LibWebRTCStatsCollector>(WTFMove(callback)); }

    static Ref<RTCStatsReport> createReport(const webrtc::scoped_refptr<const webrtc::RTCStatsReport>&);

    explicit LibWebRTCStatsCollector(CollectorCallback&&);
    ~LibWebRTCStatsCollector();

private:
    void OnStatsDelivered(const webrtc::scoped_refptr<const webrtc::RTCStatsReport>&) final;

    CollectorCallback m_callback;
};

} // namespace WebCore

#endif // ENABLE(WEB_RTC) && USE(LIBWEBRTC)
