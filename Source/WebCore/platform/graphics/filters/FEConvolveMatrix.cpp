/*
 * Copyright (C) 2004, 2005, 2006, 2007 Nikolas Zimmermann <zimmermann@kde.org>
 * Copyright (C) 2004, 2005 Rob Buis <buis@kde.org>
 * Copyright (C) 2005 Eric Seidel <eric@webkit.org>
 * Copyright (C) 2009 Dirk Schulze <krit@webkit.org>
 * Copyright (C) 2010 Zoltan Herczeg <zherczeg@webkit.org>
 * Copyright (C) 2021-2022 Apple Inc. All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public License
 * along with this library; see the file COPYING.LIB.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA 02110-1301, USA.
 */

#include "config.h"
#include "FEConvolveMatrix.h"

#include "FEConvolveMatrixSoftwareApplier.h"
#include "Filter.h"
#include <wtf/text/TextStream.h>

namespace WebCore {

Ref<FEConvolveMatrix> FEConvolveMatrix::create(const IntSize& kernelSize, float divisor, float bias, const IntPoint& targetOffset, EdgeModeType edgeMode, const FloatPoint& kernelUnitLength, bool preserveAlpha, const Vector<float>& kernelMatrix, DestinationColorSpace colorSpace)
{
    return adoptRef(*new FEConvolveMatrix(kernelSize, divisor, bias, targetOffset, edgeMode, kernelUnitLength, preserveAlpha, kernelMatrix, colorSpace));
}

FEConvolveMatrix::FEConvolveMatrix(const IntSize& kernelSize, float divisor, float bias, const IntPoint& targetOffset, EdgeModeType edgeMode, const FloatPoint& kernelUnitLength, bool preserveAlpha, const Vector<float>& kernelMatrix, DestinationColorSpace colorSpace)
    : FilterEffect(FilterEffect::Type::FEConvolveMatrix, colorSpace)
    , m_kernelSize(kernelSize)
    , m_divisor(divisor)
    , m_bias(bias)
    , m_targetOffset(targetOffset)
    , m_edgeMode(edgeMode)
    , m_kernelUnitLength(kernelUnitLength)
    , m_preserveAlpha(preserveAlpha)
    , m_kernelMatrix(kernelMatrix)
{
    ASSERT(m_kernelSize.width() > 0);
    ASSERT(m_kernelSize.height() > 0);
    ASSERT(IntRect(IntPoint::zero(), kernelSize).contains(targetOffset));
}

bool FEConvolveMatrix::operator==(const FEConvolveMatrix& other) const
{
    return FilterEffect::operator==(other)
        && m_kernelSize == other.m_kernelSize
        && m_divisor == other.m_divisor
        && m_bias == other.m_bias
        && m_targetOffset == other.m_targetOffset
        && m_edgeMode == other.m_edgeMode
        && m_kernelUnitLength == other.m_kernelUnitLength
        && m_preserveAlpha == other.m_preserveAlpha
        && m_kernelMatrix == other.m_kernelMatrix;
}

void FEConvolveMatrix::setKernelSize(const IntSize& kernelSize)
{
    ASSERT(kernelSize.width() > 0);
    ASSERT(kernelSize.height() > 0);
    m_kernelSize = kernelSize;
}

void FEConvolveMatrix::setKernel(const Vector<float>& kernel)
{
    m_kernelMatrix = kernel; 
}

bool FEConvolveMatrix::setDivisor(float divisor)
{
    ASSERT(divisor);
    if (m_divisor == divisor)
        return false;
    m_divisor = divisor;
    return true;
}

bool FEConvolveMatrix::setBias(float bias)
{
    if (m_bias == bias)
        return false;
    m_bias = bias;
    return true;
}

bool FEConvolveMatrix::setTargetOffset(const IntPoint& targetOffset)
{
    if (m_targetOffset == targetOffset)
        return false;
    m_targetOffset = targetOffset;
    return true;
}

bool FEConvolveMatrix::setEdgeMode(EdgeModeType edgeMode)
{
    if (m_edgeMode == edgeMode)
        return false;
    m_edgeMode = edgeMode;
    return true;
}

bool FEConvolveMatrix::setKernelUnitLength(const FloatPoint& kernelUnitLength)
{
    ASSERT(kernelUnitLength.x() > 0);
    ASSERT(kernelUnitLength.y() > 0);
    if (m_kernelUnitLength == kernelUnitLength)
        return false;
    m_kernelUnitLength = kernelUnitLength;
    return true;
}

bool FEConvolveMatrix::setPreserveAlpha(bool preserveAlpha)
{
    if (m_preserveAlpha == preserveAlpha)
        return false;
    m_preserveAlpha = preserveAlpha;
    return true;
}

FloatRect FEConvolveMatrix::calculateImageRect(const Filter& filter, std::span<const FloatRect>, const FloatRect& primitiveSubregion) const
{
    return filter.maxEffectRect(primitiveSubregion);
}

std::unique_ptr<FilterEffectApplier> FEConvolveMatrix::createSoftwareApplier() const
{
    return FilterEffectApplier::create<FEConvolveMatrixSoftwareApplier>(*this);
}

static TextStream& operator<<(TextStream& ts, const EdgeModeType& type)
{
    switch (type) {
    case EdgeModeType::Unknown:
        ts << "UNKNOWN"_s;
        break;
    case EdgeModeType::Duplicate:
        ts << "DUPLICATE"_s;
        break;
    case EdgeModeType::Wrap:
        ts << "WRAP"_s;
        break;
    case EdgeModeType::None:
        ts << "NONE"_s;
        break;
    }
    return ts;
}

TextStream& FEConvolveMatrix::externalRepresentation(TextStream& ts, FilterRepresentation representation) const
{
    ts << indent << "[feConvolveMatrix"_s;
    FilterEffect::externalRepresentation(ts, representation);

    ts << " order=\"" << m_kernelSize << '"';
    ts << " kernelMatrix=\"" << m_kernelMatrix  << '"';
    ts << " divisor=\"" << m_divisor << '"';
    ts << " bias=\"" << m_bias << '"';
    ts << " target=\"" << m_targetOffset << '"';
    ts << " edgeMode=\"" << m_edgeMode << '"';
    ts << " kernelUnitLength=\"" << m_kernelUnitLength << '"';
    ts << " preserveAlpha=\"" << m_preserveAlpha << '"';

    ts << "]\n"_s;
    return ts;
}

}; // namespace WebCore
