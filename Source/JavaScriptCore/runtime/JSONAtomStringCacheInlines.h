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

#pragma once

#include "Identifier.h"
#include "JSONAtomStringCache.h"
#include "SmallStrings.h"
#include "VM.h"

namespace JSC {

template<typename CharacterType>
ALWAYS_INLINE Ref<AtomStringImpl> JSONAtomStringCache::makeIdentifier(std::span<const CharacterType> characters)
{
    if (characters.empty())
        return *emptyAtom().impl();

    auto firstCharacter = characters.front();
    if (characters.size() == 1) {
        if (firstCharacter <= maxSingleCharacterString)
            return vm().smallStrings.singleCharacterStringRep(firstCharacter);
    } else if (characters.size() > maxStringLengthForCache) [[unlikely]]
        return AtomStringImpl::add(characters).releaseNonNull();

    auto lastCharacter = characters.back();
    auto& slot = cacheSlot(firstCharacter, lastCharacter, characters.size());
    if (slot.m_length != characters.size() || !equal(slot.m_buffer, characters)) [[unlikely]] {
        auto result = AtomStringImpl::add(characters);
        slot.m_impl = result;
        slot.m_length = characters.size();
        WTF::copyElements(std::span<char16_t> { slot.m_buffer }, characters);
        return result.releaseNonNull();
    }

    return *slot.m_impl;
}

template<typename CharacterType>
ALWAYS_INLINE AtomStringImpl* JSONAtomStringCache::existingIdentifier(std::span<const CharacterType> characters)
{
    if (characters.empty())
        return emptyAtom().impl();

    auto firstCharacter = characters.front();
    if (characters.size() == 1) {
        if (firstCharacter <= maxSingleCharacterString)
            return vm().smallStrings.existingSingleCharacterStringRep(firstCharacter);
    } else if (characters.size() > maxStringLengthForCache) [[unlikely]]
        return nullptr;

    auto lastCharacter = characters.back();
    auto& slot = cacheSlot(firstCharacter, lastCharacter, characters.size());
    if (slot.m_length != characters.size() || !equal(slot.m_buffer, characters)) [[unlikely]]
        return nullptr;

    return slot.m_impl.get();
}

ALWAYS_INLINE VM& JSONAtomStringCache::vm() const
{
    return *std::bit_cast<VM*>(std::bit_cast<uintptr_t>(this) - OBJECT_OFFSETOF(VM, jsonAtomStringCache));
}

} // namespace JSC
