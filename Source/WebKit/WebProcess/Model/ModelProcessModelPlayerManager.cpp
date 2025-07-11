/*
 * Copyright (C) 2024 Apple Inc. All rights reserved.
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
#include "ModelProcessModelPlayerManager.h"

#if ENABLE(MODEL_PROCESS)

#include "ModelProcessModelPlayer.h"
#include "ModelProcessModelPlayerManagerProxyMessages.h"
#include "WebPage.h"
#include "WebProcess.h"
#include "WebProcessPoolMessages.h"
#include <WebCore/ModelPlayer.h>
#include <WebCore/ModelPlayerClient.h>
#include <WebCore/ModelPlayerIdentifier.h>
#include <wtf/TZoneMallocInlines.h>

namespace WebKit {

WTF_MAKE_TZONE_ALLOCATED_IMPL(ModelProcessModelPlayerManager);

Ref<ModelProcessModelPlayerManager> ModelProcessModelPlayerManager::create()
{
    return adoptRef(*new ModelProcessModelPlayerManager());
}

ModelProcessModelPlayerManager::ModelProcessModelPlayerManager() = default;

ModelProcessModelPlayerManager::~ModelProcessModelPlayerManager() = default;

ModelProcessConnection& ModelProcessModelPlayerManager::modelProcessConnection()
{
    RefPtr modelProcessConnection = m_modelProcessConnection.get();
    if (!modelProcessConnection) {
        modelProcessConnection = WebProcess::singleton().ensureModelProcessConnection();
        m_modelProcessConnection = modelProcessConnection;
        modelProcessConnection = WebProcess::singleton().ensureModelProcessConnection();
        modelProcessConnection->addClient(*this);
    }

    return *modelProcessConnection;
}

Ref<ModelProcessModelPlayer> ModelProcessModelPlayerManager::createModelProcessModelPlayer(WebPage& page, WebCore::ModelPlayerClient& client)
{
    auto identifier = WebCore::ModelPlayerIdentifier::generate();
    modelProcessConnection().connection().send(Messages::ModelProcessModelPlayerManagerProxy::CreateModelPlayer(identifier), 0);

    auto player = ModelProcessModelPlayer::create(identifier, page, client);
    if (page.shouldDisableModelLoadDelaysForTesting())
        player->disableUnloadDelayForTesting();

    auto previousPlayerCount = m_players.size();
    m_players.add(identifier, player);
    if (!previousPlayerCount && m_players.size())
        WebProcess::singleton().send(Messages::WebProcessPool::StartedPlayingModels(), 0);

    return player;
}

void ModelProcessModelPlayerManager::deleteModelProcessModelPlayer(WebCore::ModelPlayer& modelPlayer)
{
    WebCore::ModelPlayerIdentifier identifier = modelPlayer.identifier();
    auto previousPlayerCount = m_players.size();
    m_players.take(identifier);
    if (previousPlayerCount && !m_players.size())
        WebProcess::singleton().send(Messages::WebProcessPool::StoppedPlayingModels(), 0);
    modelProcessConnection().connection().send(Messages::ModelProcessModelPlayerManagerProxy::DeleteModelPlayer(identifier), 0);
}

void ModelProcessModelPlayerManager::didReceivePlayerMessage(IPC::Connection& connection, IPC::Decoder& decoder)
{
    if (const auto& player = m_players.get(WebCore::ModelPlayerIdentifier(decoder.destinationID())))
        player->didReceiveMessage(connection, decoder);
}

void ModelProcessModelPlayerManager::didUnloadModelProcessModelPlayer(WebCore::ModelPlayerIdentifier modelPlayerIdentifier)
{
    auto previousPlayerCount = m_players.size();
    if (const auto& player = m_players.take(modelPlayerIdentifier))
        player->didUnload();
    if (previousPlayerCount && !m_players.size())
        WebProcess::singleton().send(Messages::WebProcessPool::StoppedPlayingModels(), 0);
}

void ModelProcessModelPlayerManager::modelProcessConnectionDidClose(ModelProcessConnection&)
{
    m_modelProcessConnection = nullptr;

    auto playersToNotify = m_players;
    for (auto& entry : playersToNotify) {
        if (RefPtr player = entry.value.get())
            player->didUnload();
    }
}

}

#endif // ENABLE(MODEL_PROCESS)
