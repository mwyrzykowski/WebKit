/*
 * Copyright (c) 2021-2024 Apple Inc. All rights reserved.
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

#include "APIConversions.h"
#include "Buffer.h"
#include "CommandEncoder.h"
#include "CommandsMixin.h"
#include "Device.h"
#include "QuerySet.h"
#include "Queue.h"
#include "Texture.h"
#include "WebGPU.h"
#include <cstdint>
#include <span>
#include <wtf/CheckedArithmetic.h>
#include <wtf/StdLibExtras.h>

using SpanConstUInt8 = std::span<const uint8_t>;
using SpanUInt8 = std::span<uint8_t>;
using WTFRangeSizeT = WTF::Range<size_t>;

__attribute__((used)) static const auto stdDynamicExtent = std::dynamic_extent;

// FIXME: importing WTF::Range does not work
namespace WTF {
template<typename PassedType>
class Range;
}

// FIXME: rdar://140819194
constexpr unsigned long int WGPU_COPY_STRIDE_UNDEFINED_ = WGPU_COPY_STRIDE_UNDEFINED;

// FIXME: rdar://140819448
constexpr auto MTLBlitOptionNone_ = MTLBlitOptionNone;

inline unsigned long roundUpToMultipleOfNonPowerOfTwoCheckedUInt32UnsignedLong(Checked<uint32_t> x, unsigned long y) { return WTF::roundUpToMultipleOfNonPowerOfTwo<unsigned long int, Checked<uint32_t>>(x, y); }

inline Checked<size_t> checkedDifferenceSizeT(size_t left, size_t right)
{
    return WTF::checkedDifference<size_t>(left, right);
}


#ifndef __swift__
#include "WebGPUSwift-Generated.h"
#endif
