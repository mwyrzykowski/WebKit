/*
* Copyright (C) 2020-2025 Apple Inc. All rights reserved.
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
* THIS SOFTWARE IS PROVIDED BY APPLE COMPUTER, INC. ``AS IS'' AND ANY
* EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
* PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL APPLE COMPUTER, INC. OR
* CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
* EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
* PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
* OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "config.h"
#include "WebSleepDisablerClient.h"

#include "MessageSenderInlines.h"
#include "WebPage.h"
#include "WebPageProxyMessages.h"
#include "WebProcess.h"
#include <wtf/TZoneMallocInlines.h>

namespace WebKit {

WTF_MAKE_TZONE_ALLOCATED_IMPL(WebSleepDisablerClient);

void WebSleepDisablerClient::didCreateSleepDisabler(WebCore::SleepDisablerIdentifier identifier, const String& reason, bool display, std::optional<WebCore::PageIdentifier> pageID)
{
    if (RefPtr webPage = pageID ? WebProcess::singleton().webPage(*pageID) : nullptr)
        webPage->send(Messages::WebPageProxy::DidCreateSleepDisabler(identifier, reason, display));
}

void WebSleepDisablerClient::didDestroySleepDisabler(WebCore::SleepDisablerIdentifier identifier, std::optional<WebCore::PageIdentifier> pageID)
{
    if (RefPtr webPage = pageID ? WebProcess::singleton().webPage(*pageID) : nullptr)
        webPage->send(Messages::WebPageProxy::DidDestroySleepDisabler(identifier));
}

} // namespace WebKit
