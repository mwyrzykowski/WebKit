/*
 * Copyright (C) 2006, 2007 Apple Inc. All rights reserved.
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

#include "config.h"
#include "JSStringRefCF.h"

#include "APICast.h"
#include "InitializeThreading.h"
#include "JSCJSValue.h"
#include "JSStringRef.h"
#include "OpaqueJSString.h"
#include <wtf/StdLibExtras.h>

WTF_ALLOW_UNSAFE_BUFFER_USAGE_BEGIN

JSStringRef JSStringCreateWithCFString(CFStringRef string)
{
    JSC::initialize();

    // We cannot use CFIndex here since CFStringGetLength can return values larger than
    // it can hold.  (<rdar://problem/6806478>)
    size_t length = CFStringGetLength(string);
    if (!length)
        return &OpaqueJSString::create(""_span8).leakRef();

    Vector<LChar, 1024> lcharBuffer(length);
    CFIndex usedBufferLength;
    CFIndex convertedSize = CFStringGetBytes(string, CFRangeMake(0, length), kCFStringEncodingISOLatin1, 0, false, lcharBuffer.mutableSpan().data(), length, &usedBufferLength);
    if (static_cast<size_t>(convertedSize) == length && static_cast<size_t>(usedBufferLength) == length)
        return &OpaqueJSString::create(lcharBuffer.span()).leakRef();

    Vector<UniChar> buffer(length);
    CFStringGetCharacters(string, CFRangeMake(0, length), buffer.mutableSpan().data());
    static_assert(sizeof(UniChar) == sizeof(char16_t), "UniChar and char16_t must be same size");
    return &OpaqueJSString::create({ reinterpret_cast<const char16_t*>(buffer.span().data()), length }).leakRef();
}

CFStringRef JSStringCopyCFString(CFAllocatorRef allocator, JSStringRef string)
{
    if (!string || !string->length())
        return CFSTR("");

    if (string->is8Bit()) {
        auto characters = string->span8();
        return CFStringCreateWithBytes(allocator, characters.data(), characters.size(), kCFStringEncodingISOLatin1, false);
    }
    auto characters = string->span16();
    return CFStringCreateWithCharacters(allocator, reinterpret_cast<const UniChar*>(characters.data()), characters.size());
}

WTF_ALLOW_UNSAFE_BUFFER_USAGE_END
