/*
 * Copyright (C) 2025 Apple Inc. All rights reserved.
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
#include "MeshImpl.h"

#if ENABLE(GPU_PROCESS_MODEL)

#include "ModelTypes.h"
#include "WebKitMesh.h"
#include <WebCore/IOSurface.h>
#include <wtf/StdLibExtras.h>
#include <wtf/TZoneMallocInlines.h>

namespace WebKit {

WTF_MAKE_TZONE_ALLOCATED_IMPL(MeshImpl);

MeshImpl::MeshImpl(Ref<WebMesh>&& mesh, Vector<UniqueRef<WebCore::IOSurface>>&& renderBuffers)
    : m_backing(WTF::move(mesh))
    , m_renderBuffers(WTF::move(renderBuffers))
{
}

MeshImpl::~MeshImpl() = default;

void MeshImpl::setLabelInternal(const String&)
{
    // FIXME: Implement this.
}

void MeshImpl::update(const WebModel::UpdateMeshDescriptor& descriptor)
{
#if ENABLE(GPU_PROCESS_MODEL)
    m_backing->update(descriptor);
#else
    UNUSED_PARAM(descriptor);
#endif
}

void MeshImpl::updateTexture(const WebModel::UpdateTextureDescriptor& descriptor)
{
#if ENABLE(GPU_PROCESS_MODEL)
    m_backing->updateTexture(descriptor);
#else
    UNUSED_PARAM(descriptor);
#endif
}

void MeshImpl::updateMaterial(const WebModel::UpdateMaterialDescriptor& descriptor)
{
#if ENABLE(GPU_PROCESS_MODEL)
    m_backing->updateMaterial(descriptor);
#else
    UNUSED_PARAM(descriptor);
#endif
}

void MeshImpl::render()
{
#if ENABLE(GPU_PROCESS_MODEL)
    m_backing->render();
#endif
}

void MeshImpl::setEntityTransform(const WebModel::Float4x4& transform)
{
#if ENABLE(GPU_PROCESS_MODEL)
    m_backing->setTransform(transform);
#else
    UNUSED_PARAM(transform);
#endif
}

#if PLATFORM(COCOA)
std::optional<WebModel::Float4x4> MeshImpl::entityTransform() const
{
    return std::nullopt;
}
#endif

void MeshImpl::setCameraDistance(float distance)
{
#if ENABLE(GPU_PROCESS_MODEL)
    m_backing->setCameraDistance(distance);
#else
    UNUSED_PARAM(distance);
#endif
}

void MeshImpl::play(bool play)
{
#if ENABLE(GPU_PROCESS_MODEL)
    m_backing->play(play);
#else
    UNUSED_PARAM(play);
#endif
}

void MeshImpl::setEnvironmentMap(const WebModel::ImageAsset& imageAsset)
{
#if ENABLE(GPU_PROCESS_MODEL)
    m_backing->setEnvironmentMap(imageAsset);
#else
    UNUSED_PARAM(imageAsset);
#endif
}

#if PLATFORM(COCOA)
Vector<MachSendRight> MeshImpl::ioSurfaceHandles()
{
    return m_renderBuffers.map([](const auto& renderBuffer) {
        return renderBuffer->createSendRight();
    });
}
#endif

}

#endif
