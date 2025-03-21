/*
 * Copyright (C) 2015 Apple Inc. All rights reserved.
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
#include "IndexKey.h"

#include <wtf/CrossThreadCopier.h>

namespace WebCore {

IndexKey::IndexKey() = default;

IndexKey::IndexKey(Data&& keys)
    : m_keys(WTFMove(keys))
{
}

IndexKey IndexKey::isolatedCopy() const &
{
    return { crossThreadCopy(m_keys) };
}

IndexKey IndexKey::isolatedCopy() &&
{
    return { crossThreadCopy(WTFMove(m_keys)) };
}

IDBKeyData IndexKey::asOneKey() const
{
    return WTF::switchOn(m_keys, [](std::nullptr_t) {
        return IDBKeyData { };
    }, [](const IDBKeyData& keyData) {
        return keyData;
    }, [](const Vector<IDBKeyData>& keyDataVector) {
        IDBKeyData result;
        result.setArrayValue(keyDataVector);
        return result;
    });
}

Vector<IDBKeyData> IndexKey::multiEntry() const
{
    Vector<IDBKeyData> multiEntry;

    WTF::switchOn(m_keys, [](std::nullptr_t) {
    }, [&](const IDBKeyData& keyData) {
        if (keyData.isValid())
            multiEntry.append(keyData);
    }, [&](const Vector<IDBKeyData>& keyDataVector) {
        for (auto& key : keyDataVector) {
            if (!key.isValid())
                continue;

            bool skip = false;
            for (auto& otherKey : multiEntry) {
                if (key == otherKey) {
                    skip = true;
                    break;
                }
            }

            if (!skip)
                multiEntry.append(key);
        }
    });

    return multiEntry;
}

void IndexKey::updatePlaceholderKeys(const IDBKeyData& newKeyData)
{
    ASSERT(newKeyData.isValid());

    WTF::switchOn(m_keys, [](std::nullptr_t) {
    }, [&](IDBKeyData& keyData) {
        if (!keyData.isValid() && keyData.isPlaceholder())
            keyData = newKeyData;
    }, [&](Vector<IDBKeyData>& keyDataVector) {
        for (auto& keyData : keyDataVector) {
            if (!keyData.isValid() && keyData.isPlaceholder())
                keyData = newKeyData;
        }
    });
}

} // namespace WebCore
