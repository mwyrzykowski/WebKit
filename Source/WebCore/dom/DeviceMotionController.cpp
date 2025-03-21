/*
 * Copyright 2010 Apple Inc. All rights reserved.
 * Copyright (C) 2012 Samsung Electronics. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "config.h"
#include "DeviceMotionController.h"

#include "DeviceMotionClient.h"
#include "DeviceMotionData.h"
#include "DeviceMotionEvent.h"
#include "EventNames.h"
#include "Page.h"
#include <wtf/TZoneMallocInlines.h>

namespace WebCore {

WTF_MAKE_TZONE_ALLOCATED_IMPL(DeviceMotionClient);
WTF_MAKE_TZONE_ALLOCATED_IMPL(DeviceMotionController);

DeviceMotionController::DeviceMotionController(DeviceMotionClient& client)
    : m_client(client)
{
    client.setController(this);
}

#if PLATFORM(IOS_FAMILY)

// FIXME: We should look to reconcile the iOS vs. non-iOS differences with this class
// so that we can either remove these functions or the PLATFORM(IOS_FAMILY)-guard.

void DeviceMotionController::suspendUpdates()
{
    m_client->stopUpdating();
}

void DeviceMotionController::resumeUpdates(const SecurityOriginData& origin)
{
    if (hasListeners())
        m_client->startUpdating(origin);
}

#endif
    
void DeviceMotionController::didChangeDeviceMotion(DeviceMotionData* deviceMotionData)
{
    dispatchDeviceEvent(DeviceMotionEvent::create(eventNames().devicemotionEvent, deviceMotionData));
}

bool DeviceMotionController::hasLastData()
{
    return m_client->lastMotion();
}

RefPtr<Event> DeviceMotionController::getLastEvent()
{
    RefPtr lastMotion = m_client->lastMotion();
    return DeviceMotionEvent::create(eventNames().devicemotionEvent, lastMotion.get());
}

ASCIILiteral DeviceMotionController::supplementName()
{
    return "DeviceMotionController"_s;
}

DeviceMotionController* DeviceMotionController::from(Page* page)
{
    return static_cast<DeviceMotionController*>(Supplement<Page>::from(page, supplementName()));
}

bool DeviceMotionController::isActiveAt(Page* page)
{
    if (DeviceMotionController* self = DeviceMotionController::from(page))
        return self->isActive();
    return false;
}

DeviceClient& DeviceMotionController::client()
{
    return m_client.get();
}

} // namespace WebCore
