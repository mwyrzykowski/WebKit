/*
 * Copyright (C) 2016, Canon Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *     * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following disclaimer
 * in the documentation and/or other materials provided with the
 * distribution.
 *     * Neither the name of Canon Inc. nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include "CachedRawResourceClient.h"
#include "CachedResourceHandle.h"
#include "ResourceLoaderIdentifier.h"
#include "ResourceRequest.h"

namespace WebCore {

class CachedRawResource;
class Document;
class DocumentThreadableLoader;
class ResourceError;

class CrossOriginPreflightChecker final : private CachedRawResourceClient {
public:
    static void doPreflight(DocumentThreadableLoader&, ResourceRequest&&);

    CrossOriginPreflightChecker(DocumentThreadableLoader&, ResourceRequest&&);
    ~CrossOriginPreflightChecker();

    void startPreflight();

    void setDefersLoading(bool);

private:
    void notifyFinished(CachedResource&, const NetworkLoadMetrics&, LoadWillContinueInAnotherProcess) final;
    void redirectReceived(CachedResource&, ResourceRequest&&, const ResourceResponse&, CompletionHandler<void(ResourceRequest&&)>&&) final;

    static void handleLoadingFailure(DocumentThreadableLoader&, unsigned long, const ResourceError&);
    static void validatePreflightResponse(DocumentThreadableLoader&, ResourceRequest&&, std::optional<ResourceLoaderIdentifier>, const ResourceResponse&);
    Ref<DocumentThreadableLoader> protectedLoader() const;

    SingleThreadWeakRef<DocumentThreadableLoader> m_loader;
    CachedResourceHandle<CachedRawResource> m_resource;
    ResourceRequest m_request;
};

} // namespace WebCore
