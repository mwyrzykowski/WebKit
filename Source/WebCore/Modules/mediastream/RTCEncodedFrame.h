/*
 * Copyright (C) 2020-2025 Apple Inc. All rights reserved.
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
 * THIS SOFTWARE IS PROVIDED BY APPLE INC. ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL APPLE INC. OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#if ENABLE(WEB_RTC)

#include "RTCRtpTransformableFrame.h"
#include <wtf/Ref.h>
#include <wtf/RefCounted.h>
#include <wtf/UniqueRef.h>

namespace JSC {
class ArrayBuffer;
class VM;
}

namespace WebCore {

class RTCEncodedFrame : public RefCounted<RTCEncodedFrame> {
public:
    virtual ~RTCEncodedFrame() { }

    RefPtr<JSC::ArrayBuffer> data() const;
    void setData(JSC::ArrayBuffer&);

    enum class ShouldNeuter : bool { No, Yes };
    Ref<RTCRtpTransformableFrame> rtcFrame(JSC::VM&, ShouldNeuter = ShouldNeuter::Yes);

    uint64_t timestamp() const;

    Ref<RTCRtpTransformableFrame> serialize();

protected:
    explicit RTCEncodedFrame(Ref<RTCRtpTransformableFrame>&&);

    const Ref<RTCRtpTransformableFrame> m_frame;
    mutable RefPtr<JSC::ArrayBuffer> m_data;
    bool m_isNeutered { false };
    mutable std::optional<uint64_t> m_timestamp;
};

} // namespace WebCore

#endif // ENABLE(WEB_RTC)
