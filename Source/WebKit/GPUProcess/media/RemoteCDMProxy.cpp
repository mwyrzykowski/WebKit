/*
 * Copyright (C) 2020 Apple Inc. All rights reserved.
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
#include "RemoteCDMProxy.h"

#if ENABLE(GPU_PROCESS) && ENABLE(ENCRYPTED_MEDIA)

#include "RemoteCDMConfiguration.h"
#include "RemoteCDMInstanceConfiguration.h"
#include "RemoteCDMInstanceProxy.h"
#include <WebCore/CDMKeySystemConfiguration.h>
#include <WebCore/CDMPrivate.h>
#include <WebCore/SharedBuffer.h>

namespace WebKit {

using namespace WebCore;

RefPtr<RemoteCDMProxy> RemoteCDMProxy::create(RemoteCDMFactoryProxy& factory, std::unique_ptr<WebCore::CDMPrivate>&& priv)
{
    if (!priv)
        return nullptr;

    auto configuration = makeUniqueRefWithoutFastMallocCheck<RemoteCDMConfiguration, RemoteCDMConfiguration&&>({
        priv->supportedInitDataTypes(),
        priv->supportedRobustnesses(),
        priv->supportsServerCertificates(),
        priv->supportsSessions()
    });

    return adoptRef(new RemoteCDMProxy(factory, WTFMove(priv), WTFMove(configuration)));
}

RemoteCDMProxy::RemoteCDMProxy(RemoteCDMFactoryProxy& factory, std::unique_ptr<CDMPrivate>&& priv, UniqueRef<RemoteCDMConfiguration>&& configuration)
    : m_factory(factory)
    , m_private(WTFMove(priv))
    , m_configuration(WTFMove(configuration))
#if !RELEASE_LOG_DISABLED
    , m_logger(factory.logger())
#endif
{
}

RemoteCDMProxy::~RemoteCDMProxy() = default;

bool RemoteCDMProxy::supportsInitData(const AtomString& type, const SharedBuffer& data)
{
    return m_private->supportsInitData(type, data);
}

RefPtr<SharedBuffer> RemoteCDMProxy::sanitizeResponse(const SharedBuffer& response)
{
    return m_private->sanitizeResponse(response);
}

std::optional<String> RemoteCDMProxy::sanitizeSessionId(const String& sessionId)
{
    return m_private->sanitizeSessionId(sessionId);
}

void RemoteCDMProxy::getSupportedConfiguration(WebCore::CDMKeySystemConfiguration&& configuration, WebCore::CDMPrivate::LocalStorageAccess access, CompletionHandler<void(std::optional<WebCore::CDMKeySystemConfiguration>)>&& callback)
{
    m_private->getSupportedConfiguration(WTFMove(configuration), access, WTFMove(callback));
}

void RemoteCDMProxy::createInstance(CompletionHandler<void(std::optional<RemoteCDMInstanceIdentifier>, RemoteCDMInstanceConfiguration&&)>&& completion)
{
    auto privateInstance = m_private->createInstance();
    if (!privateInstance || !m_factory) {
        completion(std::nullopt, { });
        return;
    }
    auto identifier = RemoteCDMInstanceIdentifier::generate();
    auto instance = RemoteCDMInstanceProxy::create(*this, privateInstance.releaseNonNull(), identifier);
    RemoteCDMInstanceConfiguration configuration = instance->configuration();
    protectedFactory()->addInstance(identifier, WTFMove(instance));
    completion(identifier, WTFMove(configuration));
}

void RemoteCDMProxy::loadAndInitialize()
{
    m_private->loadAndInitialize();
}

void RemoteCDMProxy::setLogIdentifier(uint64_t logIdentifier)
{
#if !RELEASE_LOG_DISABLED
    m_logIdentifier = logIdentifier;
    if (m_factory)
        m_private->setLogIdentifier(m_logIdentifier);
#else
    UNUSED_PARAM(logIdentifier);
#endif
}

std::optional<SharedPreferencesForWebProcess> RemoteCDMProxy::sharedPreferencesForWebProcess() const
{
    RefPtr factory = m_factory.get();
    if (!factory)
        return std::nullopt;

    return factory->sharedPreferencesForWebProcess();
}

}

#endif
