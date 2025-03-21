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
#include "FileSystemStorageManager.h"

#include "FileSystemStorageError.h"
#include "FileSystemStorageHandleRegistry.h"
#include "WebFileSystemStorageConnectionMessages.h"
#include <wtf/TZoneMallocInlines.h>

namespace WebKit {

WTF_MAKE_TZONE_ALLOCATED_IMPL(FileSystemStorageManager);

Ref<FileSystemStorageManager> FileSystemStorageManager::create(String&& path, FileSystemStorageHandleRegistry& registry, QuotaCheckFunction&& quotaCheckFunction)
{
    return adoptRef(*new FileSystemStorageManager(WTFMove(path), registry, WTFMove(quotaCheckFunction)));
}

FileSystemStorageManager::FileSystemStorageManager(String&& path, FileSystemStorageHandleRegistry& registry, QuotaCheckFunction&& quotaCheckFunction)
    : m_path(WTFMove(path))
    , m_registry(registry)
    , m_quotaCheckFunction(WTFMove(quotaCheckFunction))
{
    ASSERT(!RunLoop::isMain());
}

FileSystemStorageManager::~FileSystemStorageManager()
{
    ASSERT(!RunLoop::isMain());

    close();
}

bool FileSystemStorageManager::isActive() const
{
    return !m_handles.isEmpty();
}

uint64_t FileSystemStorageManager::allocatedUnusedCapacity() const
{
    CheckedUint64 result = 0;
    for (auto& handle : m_handles.values())
        result += handle->allocatedUnusedCapacity();

    if (result.hasOverflowed())
        return 0;

    return result;
}

Expected<WebCore::FileSystemHandleIdentifier, FileSystemStorageError> FileSystemStorageManager::createHandle(IPC::Connection::UniqueID connection, FileSystemStorageHandle::Type type, String&& path, String&& name, bool createIfNecessary)
{
    ASSERT(!RunLoop::isMain());

    if (path.isEmpty())
        return makeUnexpected(FileSystemStorageError::Unknown);

    auto fileExists = FileSystem::fileExists(path);
    if (!createIfNecessary && !fileExists)
        return makeUnexpected(FileSystemStorageError::FileNotFound);

    if (fileExists) {
        auto existingFileType = FileSystem::fileType(path);
        if (!existingFileType)
            return makeUnexpected(FileSystemStorageError::Unknown);

        auto existingHandleType = (existingFileType.value() == FileSystem::FileType::Regular) ? FileSystemStorageHandle::Type::File : FileSystemStorageHandle::Type::Directory;
        if (type == FileSystemStorageHandle::Type::Any)
            type = existingHandleType;
        else {
            // Requesting type and existing type should be a match.
            if (existingHandleType != type)
                return makeUnexpected(FileSystemStorageError::TypeMismatch);
        }
    }

    RefPtr newHandle = FileSystemStorageHandle::create(*this, type, WTFMove(path), WTFMove(name));
    if (!newHandle)
        return makeUnexpected(FileSystemStorageError::Unknown);
    auto newHandleIdentifier = newHandle->identifier();
    m_handlesByConnection.ensure(connection, [&] {
        return HashSet<WebCore::FileSystemHandleIdentifier> { };
    }).iterator->value.add(newHandleIdentifier);
    if (RefPtr registry = m_registry.get())
        registry->registerHandle(newHandleIdentifier, *newHandle);
    m_handles.add(newHandleIdentifier, WTFMove(newHandle));
    return newHandleIdentifier;
}

const String& FileSystemStorageManager::getPath(WebCore::FileSystemHandleIdentifier identifier)
{
    auto handle = m_handles.find(identifier);
    return handle == m_handles.end() ? emptyString() : handle->value->path();
}

FileSystemStorageHandle::Type FileSystemStorageManager::getType(WebCore::FileSystemHandleIdentifier identifier)
{
    auto handle = m_handles.find(identifier);
    return handle == m_handles.end() ? FileSystemStorageHandle::Type::Any : handle->value->type();
}

void FileSystemStorageManager::closeHandle(FileSystemStorageHandle& handle)
{
    auto identifier = handle.identifier();
    auto takenHandle = m_handles.take(identifier);
    ASSERT(takenHandle.get() == &handle);
    for (auto& handles : m_handlesByConnection.values()) {
        if (handles.remove(identifier))
            break;
    }
    if (RefPtr registry = m_registry.get())
        registry->unregisterHandle(identifier);
}

void FileSystemStorageManager::connectionClosed(IPC::Connection::UniqueID connection)
{
    ASSERT(!RunLoop::isMain());

    auto connectionHandles = m_handlesByConnection.find(connection);
    if (connectionHandles == m_handlesByConnection.end())
        return;

    auto identifiers = connectionHandles->value;
    for (auto identifier : identifiers) {
        if (RefPtr handle = m_handles.get(identifier))
            handle->close();
    }

    m_handlesByConnection.remove(connectionHandles);
}

Expected<WebCore::FileSystemHandleIdentifier, FileSystemStorageError> FileSystemStorageManager::getDirectory(IPC::Connection::UniqueID connection)
{
    ASSERT(!RunLoop::isMain());

    return createHandle(connection, FileSystemStorageHandle::Type::Directory, String { m_path }, { }, true);
}

// https://fs.spec.whatwg.org/#file-entry-lock-take
bool FileSystemStorageManager::acquireLockForFile(const String& path, LockType lockType)
{
    auto iterator = m_lockMap.ensure(path, [] {
        return Lock { };
    }).iterator;

    auto& lock = iterator->value;
    if (lockType == LockType::Exclusive) {
        if (lock.state == Lock::State::Open) {
            lock.state = Lock::State::TakenExclusive;
            return true;
        }
    } else if (lockType == LockType::Shared) {
        if (lock.state == Lock::State::Open) {
            lock.state = Lock::State::TakenShared;
            lock.count = 1;
            return true;
        }
        if (lock.state == Lock::State::TakenShared) {
            ++lock.count;
            return true;
        }
    }

    return false;
}

// https://fs.spec.whatwg.org/#file-entry-lock-release
bool FileSystemStorageManager::releaseLockForFile(const String& path)
{
    auto iterator = m_lockMap.find(path);
    if (iterator == m_lockMap.end())
        return false;

    auto& lock = iterator->value;
    if (lock.state == Lock::State::TakenShared) {
        if (--lock.count)
            return false;
    }
    m_lockMap.remove(iterator);
    return true;
}

void FileSystemStorageManager::close()
{
    ASSERT(!RunLoop::isMain());

    for (auto& [connectionID, identifiers] : m_handlesByConnection) {
        for (auto identifier : identifiers) {
            auto takenHandle = m_handles.take(identifier);
            if (RefPtr registry = m_registry.get())
                registry->unregisterHandle(identifier);

            // Send messages to web process to invalidate active sync access handle and writables.
            if (auto accessHandleIdentifier = takenHandle->activeSyncAccessHandle())
                IPC::Connection::send(connectionID, Messages::WebFileSystemStorageConnection::InvalidateAccessHandle(*accessHandleIdentifier), 0);

            for (auto writableIdentifier : takenHandle->writables())
                IPC::Connection::send(connectionID, Messages::WebFileSystemStorageConnection::InvalidateWritable(writableIdentifier), 0);
        }
    }

    ASSERT(m_handles.isEmpty());
    m_handlesByConnection.clear();
    m_lockMap.clear();
}

void FileSystemStorageManager::requestSpace(uint64_t size, CompletionHandler<void(bool)>&& completionHandler)
{
    m_quotaCheckFunction(size, WTFMove(completionHandler));
}

} // namespace WebKit
