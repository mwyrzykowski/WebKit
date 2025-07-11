/*
 * Copyright (C) 2008-2017 Apple Inc. All rights reserved.
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
#include "Theme.h"

#include "Color.h"
#include "FloatPoint.h"
#include "GraphicsContext.h"
#include <wtf/text/WTFString.h>

namespace WebCore {

void Theme::drawNamedImage(const String& name, GraphicsContext& context, const FloatSize& size) const
{
    // We only handle one icon at the moment.
    if (name != "wireless-playback"_s)
        return;

    GraphicsContextStateSaver stateSaver(context);
    context.setFillColor(Color::black);

    // Draw a generic Wireless Playback icon.

    context.scale(size / 100);
    context.translate(8, 1);

    Path outline;
    outline.moveTo(FloatPoint(59, 58.7));
    outline.addBezierCurveTo(FloatPoint(58.1, 58.7), FloatPoint(57.2, 58.4), FloatPoint(56.4, 57.7));
    outline.addLineTo(FloatPoint(42, 45.5));
    outline.addLineTo(FloatPoint(27.6, 57.8));
    outline.addBezierCurveTo(FloatPoint(25.9, 59.2), FloatPoint(23.4, 59), FloatPoint(22, 57.3));
    outline.addBezierCurveTo(FloatPoint(20.6, 55.6), FloatPoint(20.8, 53.1), FloatPoint(22.5, 51.7));
    outline.addLineTo(FloatPoint(39.5, 37.3));
    outline.addBezierCurveTo(FloatPoint(41, 36), FloatPoint(43.2, 36), FloatPoint(44.7, 37.3));
    outline.addLineTo(FloatPoint(61.7, 51.7));
    outline.addBezierCurveTo(FloatPoint(63.4, 53.1), FloatPoint(63.6, 55.7), FloatPoint(62.2, 57.3));
    outline.addBezierCurveTo(FloatPoint(61.3, 58.2), FloatPoint(60.1, 58.7), FloatPoint(59, 58.7));
    outline.addLineTo(FloatPoint(59, 58.7));
    outline.closeSubpath();

    outline.moveTo(FloatPoint(42, 98));
    outline.addBezierCurveTo(FloatPoint(39.8, 98), FloatPoint(38, 96.3), FloatPoint(38, 94.2));
    outline.addLineTo(FloatPoint(38, 43.6));
    outline.addBezierCurveTo(FloatPoint(38, 41.5), FloatPoint(39.8, 39.8), FloatPoint(42, 39.8));
    outline.addBezierCurveTo(FloatPoint(44.2, 39.8), FloatPoint(46, 41.5), FloatPoint(46, 43.6));
    outline.addLineTo(FloatPoint(46, 94.2));
    outline.addBezierCurveTo(FloatPoint(46, 96.3), FloatPoint(44.2, 98), FloatPoint(42, 98));
    outline.addLineTo(FloatPoint(42, 98));
    outline.closeSubpath();

    outline.moveTo(FloatPoint(83.6, 41.6));
    outline.addBezierCurveTo(FloatPoint(83.6, 18.6), FloatPoint(65, 0), FloatPoint(42, 0));
    outline.addBezierCurveTo(FloatPoint(19, 0), FloatPoint(0.4, 18.6), FloatPoint(0.4, 41.6));
    outline.addBezierCurveTo(FloatPoint(0.4, 62.2), FloatPoint(15, 79.2), FloatPoint(35, 82.6));
    outline.addLineTo(FloatPoint(35, 74.5));
    outline.addBezierCurveTo(FloatPoint(20, 71.2), FloatPoint(8.4, 57.7), FloatPoint(8.4, 41.6));
    outline.addBezierCurveTo(FloatPoint(8.4, 23.1), FloatPoint(23.5, 8), FloatPoint(42, 8));
    outline.addBezierCurveTo(FloatPoint(60.5, 8), FloatPoint(75.5, 23.1), FloatPoint(75.5, 41.6));
    outline.addBezierCurveTo(FloatPoint(75.6, 57.7), FloatPoint(64, 71.2), FloatPoint(49, 74.5));
    outline.addLineTo(FloatPoint(49, 82.6));
    outline.addBezierCurveTo(FloatPoint(69, 79.3), FloatPoint(83.6, 62.2), FloatPoint(83.6, 41.6));
    outline.addLineTo(FloatPoint(83.6, 41.6));
    outline.closeSubpath();

    context.fillPath(outline);
}

}
