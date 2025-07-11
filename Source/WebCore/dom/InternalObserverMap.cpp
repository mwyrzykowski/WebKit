/*
 * Copyright (C) 2024 Keith Cirkel <webkit@keithcirkel.co.uk>. All rights reserved.
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
#include "InternalObserverMap.h"

#include "InternalObserver.h"
#include "MapperCallback.h"
#include "Observable.h"
#include "ScriptExecutionContext.h"
#include "SubscribeOptions.h"
#include "Subscriber.h"
#include "SubscriberCallback.h"
#include <JavaScriptCore/JSCJSValueInlines.h>

namespace WebCore {

class InternalObserverMap final : public InternalObserver {
public:
    static Ref<InternalObserverMap> create(ScriptExecutionContext& context, Ref<Subscriber> subscriber, Ref<MapperCallback> mapper)
    {
        Ref internalObserver = adoptRef(*new InternalObserverMap(context, subscriber, mapper));
        internalObserver->suspendIfNeeded();
        return internalObserver;
    }

    class SubscriberCallbackMap final : public SubscriberCallback {
    public:
        static Ref<SubscriberCallbackMap> create(ScriptExecutionContext& context, Ref<Observable> source, Ref<MapperCallback> mapper)
        {
            return adoptRef(*new InternalObserverMap::SubscriberCallbackMap(context, source, mapper));
        }

        CallbackResult<void> invoke(Subscriber& subscriber) final
        {
            RefPtr context = scriptExecutionContext();

            if (!context) {
                subscriber.complete();
                return { };
            }

            SubscribeOptions options;
            options.signal = subscriber.signal();
            m_sourceObservable->subscribeInternal(*context, InternalObserverMap::create(*context, subscriber, m_mapper), options);

            return { };
        }

        CallbackResult<void> invokeRethrowingException(Subscriber& subscriber) final
        {
            return invoke(subscriber);
        }

    private:
        SubscriberCallbackMap(ScriptExecutionContext& context, Ref<Observable> source, Ref<MapperCallback> mapper)
            : SubscriberCallback(&context)
            , m_sourceObservable(source)
            , m_mapper(mapper)
        { }

        bool hasCallback() const final { return true; }

        const Ref<Observable> m_sourceObservable;
        const Ref<MapperCallback> m_mapper;
    };

private:
    void next(JSC::JSValue value) final
    {
        RefPtr context = scriptExecutionContext();
        if (!context)
            return;

        Ref vm = context->globalObject()->vm();
        JSC::JSLockHolder lock(vm);

        // The exception is not reported, instead it is forwarded to the
        // error handler.
        JSC::Exception* previousException = nullptr;
        {
            auto catchScope = DECLARE_CATCH_SCOPE(vm);
            auto result = protectedMapper()->invokeRethrowingException(value, m_idx);
            previousException = catchScope.exception();
            if (previousException) {
                catchScope.clearException();
                protectedSubscriber()->error(previousException->value());
                return;
            }

            m_idx += 1;

            if (result.type() == CallbackResultType::Success)
                protectedSubscriber()->next(result.releaseReturnValue());
        }
    }

    void error(JSC::JSValue value) final
    {
        protectedSubscriber()->error(value);
    }

    void complete() final
    {
        InternalObserver::complete();
        protectedSubscriber()->complete();
    }

    void visitAdditionalChildren(JSC::AbstractSlotVisitor& visitor) const final
    {
        m_subscriber->visitAdditionalChildren(visitor);
        m_mapper->visitJSFunction(visitor);
    }

    Ref<Subscriber> protectedSubscriber() const { return m_subscriber; }
    Ref<MapperCallback> protectedMapper() const { return m_mapper; }

    InternalObserverMap(ScriptExecutionContext& context, Ref<Subscriber> subscriber, Ref<MapperCallback> mapper)
        : InternalObserver(context)
        , m_subscriber(subscriber)
        , m_mapper(mapper)
    { }

    const Ref<Subscriber> m_subscriber;
    const Ref<MapperCallback> m_mapper;
    uint64_t m_idx { 0 };
};

Ref<SubscriberCallback> createSubscriberCallbackMap(ScriptExecutionContext& context, Ref<Observable> observable, Ref<MapperCallback> mapper)
{
    return InternalObserverMap::SubscriberCallbackMap::create(context, observable, mapper);
}

} // namespace WebCore
