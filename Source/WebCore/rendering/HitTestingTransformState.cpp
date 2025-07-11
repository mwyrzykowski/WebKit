/*
 * Copyright (C) 2011 Apple Inc. All rights reserved.
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
 * THIS SOFTWARE IS PROVIDED BY APPLE INC. ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL APPLE INC. OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
 */

#include "config.h"
#include "HitTestingTransformState.h"

#include "LayoutRect.h"

namespace WebCore {

void HitTestingTransformState::translate(int x, int y)
{
    m_accumulatedTransform.translate(x, y);
}

void HitTestingTransformState::applyTransform(const TransformationMatrix& transformFromContainer)
{
    m_accumulatedTransform.multiply(transformFromContainer);
}

void HitTestingTransformState::flatten()
{
    m_accumulatedTransform.flatten();
}

FloatPoint HitTestingTransformState::mappedPoint() const
{
    if (auto inverse = m_accumulatedTransform.inverse())
        return inverse.value().projectPoint(m_lastPlanarPoint);
    return m_lastPlanarPoint;
}

FloatQuad HitTestingTransformState::mappedQuad() const
{
    if (auto inverse = m_accumulatedTransform.inverse())
        return inverse.value().projectQuad(m_lastPlanarQuad);
    return m_lastPlanarQuad;
}

FloatQuad HitTestingTransformState::mappedArea() const
{
    if (auto inverse = m_accumulatedTransform.inverse())
        return inverse.value().projectQuad(m_lastPlanarArea);
    return m_lastPlanarArea;
}

LayoutRect HitTestingTransformState::boundsOfMappedArea() const
{
    if (auto inverse = m_accumulatedTransform.inverse())
        return inverse.value().clampedBoundsOfProjectedQuad(m_lastPlanarArea);
    TransformationMatrix identity;
    return identity.clampedBoundsOfProjectedQuad(m_lastPlanarArea);
}

LayoutRect HitTestingTransformState::boundsOfMappedQuad() const
{
    if (auto inverse = m_accumulatedTransform.inverse())
        return inverse.value().clampedBoundsOfProjectedQuad(m_lastPlanarQuad);
    TransformationMatrix identity;
    return identity.clampedBoundsOfProjectedQuad(m_lastPlanarQuad);
}


} // namespace WebCore
