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

#pragma once

#if ENABLE(WEBGPU_SWIFT) && ENABLE(GPU_PROCESS_MODEL)

#include "ModelTypes.h"
#include <ImageIO/CGImageSource.h>
#include <Metal/Metal.h>
#include <wtf/cf/VectorCF.h>
#include <wtf/cocoa/VectorCocoa.h>

namespace WebKit {

// Helper conversion functions from Metal types to WebGPU types
static WebModel::VertexSemantic toVertexSemantic(WKBridgeVertexSemantic semantic)
{
    switch (semantic) {
    case WKBridgeVertexSemanticPosition:
        return WebModel::VertexSemantic::Position;
    case WKBridgeVertexSemanticColor:
        return WebModel::VertexSemantic::Color;
    case WKBridgeVertexSemanticNormal:
        return WebModel::VertexSemantic::Normal;
    case WKBridgeVertexSemanticTangent:
        return WebModel::VertexSemantic::Tangent;
    case WKBridgeVertexSemanticBitangent:
        return WebModel::VertexSemantic::Bitangent;
    case WKBridgeVertexSemanticUV0:
        return WebModel::VertexSemantic::UV0;
    case WKBridgeVertexSemanticUV1:
        return WebModel::VertexSemantic::UV1;
    case WKBridgeVertexSemanticUV2:
        return WebModel::VertexSemantic::UV2;
    case WKBridgeVertexSemanticUV3:
        return WebModel::VertexSemantic::UV3;
    case WKBridgeVertexSemanticUV4:
        return WebModel::VertexSemantic::UV4;
    case WKBridgeVertexSemanticUV5:
        return WebModel::VertexSemantic::UV5;
    case WKBridgeVertexSemanticUV6:
        return WebModel::VertexSemantic::UV6;
    case WKBridgeVertexSemanticUV7:
        return WebModel::VertexSemantic::UV7;
    case WKBridgeVertexSemanticUnspecified:
        return WebModel::VertexSemantic::Unspecified;
    }
}

static WebCore::WebGPU::VertexFormat toVertexFormat(MTLVertexFormat format)
{
    switch (format) {
    case MTLVertexFormatUChar:
        return WebCore::WebGPU::VertexFormat::Uint8;
    case MTLVertexFormatUChar2:
        return WebCore::WebGPU::VertexFormat::Uint8x2;
    case MTLVertexFormatUChar4:
        return WebCore::WebGPU::VertexFormat::Uint8x4;
    case MTLVertexFormatChar:
        return WebCore::WebGPU::VertexFormat::Sint8;
    case MTLVertexFormatChar2:
        return WebCore::WebGPU::VertexFormat::Sint8x2;
    case MTLVertexFormatChar4:
        return WebCore::WebGPU::VertexFormat::Sint8x4;
    case MTLVertexFormatUCharNormalized:
        return WebCore::WebGPU::VertexFormat::Unorm8;
    case MTLVertexFormatUChar2Normalized:
        return WebCore::WebGPU::VertexFormat::Unorm8x2;
    case MTLVertexFormatUChar4Normalized:
        return WebCore::WebGPU::VertexFormat::Unorm8x4;
    case MTLVertexFormatCharNormalized:
        return WebCore::WebGPU::VertexFormat::Snorm8;
    case MTLVertexFormatChar2Normalized:
        return WebCore::WebGPU::VertexFormat::Snorm8x2;
    case MTLVertexFormatChar4Normalized:
        return WebCore::WebGPU::VertexFormat::Snorm8x4;
    case MTLVertexFormatUShort:
        return WebCore::WebGPU::VertexFormat::Uint16;
    case MTLVertexFormatUShort2:
        return WebCore::WebGPU::VertexFormat::Uint16x2;
    case MTLVertexFormatUShort4:
        return WebCore::WebGPU::VertexFormat::Uint16x4;
    case MTLVertexFormatShort:
        return WebCore::WebGPU::VertexFormat::Sint16;
    case MTLVertexFormatShort2:
        return WebCore::WebGPU::VertexFormat::Sint16x2;
    case MTLVertexFormatShort4:
        return WebCore::WebGPU::VertexFormat::Sint16x4;
    case MTLVertexFormatUShortNormalized:
        return WebCore::WebGPU::VertexFormat::Unorm16;
    case MTLVertexFormatUShort2Normalized:
        return WebCore::WebGPU::VertexFormat::Unorm16x2;
    case MTLVertexFormatUShort4Normalized:
        return WebCore::WebGPU::VertexFormat::Unorm16x4;
    case MTLVertexFormatShortNormalized:
        return WebCore::WebGPU::VertexFormat::Snorm16;
    case MTLVertexFormatShort2Normalized:
        return WebCore::WebGPU::VertexFormat::Snorm16x2;
    case MTLVertexFormatShort4Normalized:
        return WebCore::WebGPU::VertexFormat::Snorm16x4;
    case MTLVertexFormatHalf:
        return WebCore::WebGPU::VertexFormat::Float16;
    case MTLVertexFormatHalf2:
        return WebCore::WebGPU::VertexFormat::Float16x2;
    case MTLVertexFormatHalf4:
        return WebCore::WebGPU::VertexFormat::Float16x4;
    case MTLVertexFormatFloat:
        return WebCore::WebGPU::VertexFormat::Float32;
    case MTLVertexFormatFloat2:
        return WebCore::WebGPU::VertexFormat::Float32x2;
    case MTLVertexFormatFloat3:
        return WebCore::WebGPU::VertexFormat::Float32x3;
    case MTLVertexFormatFloat4:
        return WebCore::WebGPU::VertexFormat::Float32x4;
    case MTLVertexFormatUInt:
        return WebCore::WebGPU::VertexFormat::Uint32;
    case MTLVertexFormatUInt2:
        return WebCore::WebGPU::VertexFormat::Uint32x2;
    case MTLVertexFormatUInt3:
        return WebCore::WebGPU::VertexFormat::Uint32x3;
    case MTLVertexFormatUInt4:
        return WebCore::WebGPU::VertexFormat::Uint32x4;
    case MTLVertexFormatInt:
        return WebCore::WebGPU::VertexFormat::Sint32;
    case MTLVertexFormatInt2:
        return WebCore::WebGPU::VertexFormat::Sint32x2;
    case MTLVertexFormatInt3:
        return WebCore::WebGPU::VertexFormat::Sint32x3;
    case MTLVertexFormatInt4:
        return WebCore::WebGPU::VertexFormat::Sint32x4;
    case MTLVertexFormatUInt1010102Normalized:
        return WebCore::WebGPU::VertexFormat::Unorm1010102;
    case MTLVertexFormatUChar4Normalized_BGRA:
        return WebCore::WebGPU::VertexFormat::Unorm8x4Bgra;
    default:
        return WebCore::WebGPU::VertexFormat::Float32;
    }
}

static WebCore::WebGPU::PrimitiveTopology toPrimitiveTopology(MTLPrimitiveType topology)
{
    switch (topology) {
    case MTLPrimitiveTypePoint:
        return WebCore::WebGPU::PrimitiveTopology::PointList;
    case MTLPrimitiveTypeLine:
        return WebCore::WebGPU::PrimitiveTopology::LineList;
    case MTLPrimitiveTypeLineStrip:
        return WebCore::WebGPU::PrimitiveTopology::LineStrip;
    case MTLPrimitiveTypeTriangle:
        return WebCore::WebGPU::PrimitiveTopology::TriangleList;
    case MTLPrimitiveTypeTriangleStrip:
        return WebCore::WebGPU::PrimitiveTopology::TriangleStrip;
    default:
        return WebCore::WebGPU::PrimitiveTopology::TriangleList;
    }
}

static WebModel::IndexType toIndexType(MTLIndexType indexType)
{
    switch (indexType) {
    case MTLIndexTypeUInt16:
        return WebModel::IndexType::UInt16;
    case MTLIndexTypeUInt32:
        return WebModel::IndexType::UInt32;
    default:
        return WebModel::IndexType::UInt16;
    }
}

static WebCore::WebGPU::TextureViewDimension toTextureViewDimension(MTLTextureType textureType)
{
    switch (textureType) {
    case MTLTextureType1D:
        return WebCore::WebGPU::TextureViewDimension::_1d;
    case MTLTextureType2D:
        return WebCore::WebGPU::TextureViewDimension::_2d;
    case MTLTextureType2DArray:
        return WebCore::WebGPU::TextureViewDimension::_2dArray;
    case MTLTextureTypeCube:
        return WebCore::WebGPU::TextureViewDimension::Cube;
    case MTLTextureTypeCubeArray:
        return WebCore::WebGPU::TextureViewDimension::CubeArray;
    case MTLTextureType3D:
        return WebCore::WebGPU::TextureViewDimension::_3d;
    default:
        return WebCore::WebGPU::TextureViewDimension::_2d;
    }
}

static WebCore::WebGPU::TextureFormat toTextureFormat(MTLPixelFormat pixelFormat)
{
    switch (pixelFormat) {
    case MTLPixelFormatR8Unorm:
        return WebCore::WebGPU::TextureFormat::R8unorm;
    case MTLPixelFormatR8Snorm:
        return WebCore::WebGPU::TextureFormat::R8snorm;
    case MTLPixelFormatR8Uint:
        return WebCore::WebGPU::TextureFormat::R8uint;
    case MTLPixelFormatR8Sint:
        return WebCore::WebGPU::TextureFormat::R8sint;
    case MTLPixelFormatR16Uint:
        return WebCore::WebGPU::TextureFormat::R16uint;
    case MTLPixelFormatR16Sint:
        return WebCore::WebGPU::TextureFormat::R16sint;
    case MTLPixelFormatR16Float:
        return WebCore::WebGPU::TextureFormat::R16float;
    case MTLPixelFormatRG8Unorm:
        return WebCore::WebGPU::TextureFormat::Rg8unorm;
    case MTLPixelFormatRG8Snorm:
        return WebCore::WebGPU::TextureFormat::Rg8snorm;
    case MTLPixelFormatRG8Uint:
        return WebCore::WebGPU::TextureFormat::Rg8uint;
    case MTLPixelFormatRG8Sint:
        return WebCore::WebGPU::TextureFormat::Rg8sint;
    case MTLPixelFormatR32Float:
        return WebCore::WebGPU::TextureFormat::R32float;
    case MTLPixelFormatR32Uint:
        return WebCore::WebGPU::TextureFormat::R32uint;
    case MTLPixelFormatR32Sint:
        return WebCore::WebGPU::TextureFormat::R32sint;
    case MTLPixelFormatRG16Uint:
        return WebCore::WebGPU::TextureFormat::Rg16uint;
    case MTLPixelFormatRG16Sint:
        return WebCore::WebGPU::TextureFormat::Rg16sint;
    case MTLPixelFormatRG16Float:
        return WebCore::WebGPU::TextureFormat::Rg16float;
    case MTLPixelFormatRGBA8Unorm:
        return WebCore::WebGPU::TextureFormat::Rgba8unorm;
    case MTLPixelFormatRGBA8Unorm_sRGB:
        return WebCore::WebGPU::TextureFormat::Rgba8unormSRGB;
    case MTLPixelFormatRGBA8Snorm:
        return WebCore::WebGPU::TextureFormat::Rgba8snorm;
    case MTLPixelFormatRGBA8Uint:
        return WebCore::WebGPU::TextureFormat::Rgba8uint;
    case MTLPixelFormatRGBA8Sint:
        return WebCore::WebGPU::TextureFormat::Rgba8sint;
    case MTLPixelFormatBGRA8Unorm:
        return WebCore::WebGPU::TextureFormat::Bgra8unorm;
    case MTLPixelFormatBGRA8Unorm_sRGB:
        return WebCore::WebGPU::TextureFormat::Bgra8unormSRGB;
    case MTLPixelFormatRGB10A2Unorm:
        return WebCore::WebGPU::TextureFormat::Rgb10a2unorm;
    case MTLPixelFormatRG11B10Float:
        return WebCore::WebGPU::TextureFormat::Rg11b10ufloat;
    case MTLPixelFormatRGB9E5Float:
        return WebCore::WebGPU::TextureFormat::Rgb9e5ufloat;
    case MTLPixelFormatRGB10A2Uint:
        return WebCore::WebGPU::TextureFormat::Rgb10a2uint;
    case MTLPixelFormatRG32Float:
        return WebCore::WebGPU::TextureFormat::Rg32float;
    case MTLPixelFormatRG32Uint:
        return WebCore::WebGPU::TextureFormat::Rg32uint;
    case MTLPixelFormatRG32Sint:
        return WebCore::WebGPU::TextureFormat::Rg32sint;
    case MTLPixelFormatRGBA16Uint:
        return WebCore::WebGPU::TextureFormat::Rgba16uint;
    case MTLPixelFormatRGBA16Sint:
        return WebCore::WebGPU::TextureFormat::Rgba16sint;
    case MTLPixelFormatRGBA16Float:
        return WebCore::WebGPU::TextureFormat::Rgba16float;
    case MTLPixelFormatRGBA32Float:
        return WebCore::WebGPU::TextureFormat::Rgba32float;
    case MTLPixelFormatRGBA32Uint:
        return WebCore::WebGPU::TextureFormat::Rgba32uint;
    case MTLPixelFormatRGBA32Sint:
        return WebCore::WebGPU::TextureFormat::Rgba32sint;
    case MTLPixelFormatStencil8:
        return WebCore::WebGPU::TextureFormat::Stencil8;
    case MTLPixelFormatDepth16Unorm:
        return WebCore::WebGPU::TextureFormat::Depth16unorm;
    case MTLPixelFormatDepth32Float:
        return WebCore::WebGPU::TextureFormat::Depth24plus;
    case MTLPixelFormatDepth32Float_Stencil8:
        return WebCore::WebGPU::TextureFormat::Depth24plusStencil8;
    case MTLPixelFormatETC2_RGB8:
        return WebCore::WebGPU::TextureFormat::Etc2Rgb8unorm;
    case MTLPixelFormatETC2_RGB8_sRGB:
        return WebCore::WebGPU::TextureFormat::Etc2Rgb8unormSRGB;
    case MTLPixelFormatETC2_RGB8A1:
        return WebCore::WebGPU::TextureFormat::Etc2Rgb8a1unorm;
    case MTLPixelFormatETC2_RGB8A1_sRGB:
        return WebCore::WebGPU::TextureFormat::Etc2Rgb8a1unormSRGB;
    case MTLPixelFormatEAC_RGBA8:
        return WebCore::WebGPU::TextureFormat::Etc2Rgba8unorm;
    case MTLPixelFormatEAC_RGBA8_sRGB:
        return WebCore::WebGPU::TextureFormat::Etc2Rgba8unormSRGB;
    case MTLPixelFormatEAC_R11Unorm:
        return WebCore::WebGPU::TextureFormat::EacR11unorm;
    case MTLPixelFormatEAC_R11Snorm:
        return WebCore::WebGPU::TextureFormat::EacR11snorm;
    case MTLPixelFormatEAC_RG11Unorm:
        return WebCore::WebGPU::TextureFormat::EacRg11unorm;
    case MTLPixelFormatEAC_RG11Snorm:
        return WebCore::WebGPU::TextureFormat::EacRg11snorm;
    case MTLPixelFormatASTC_4x4_LDR:
        return WebCore::WebGPU::TextureFormat::Astc4x4Unorm;
    case MTLPixelFormatASTC_4x4_sRGB:
        return WebCore::WebGPU::TextureFormat::Astc4x4UnormSRGB;
    case MTLPixelFormatASTC_5x4_LDR:
        return WebCore::WebGPU::TextureFormat::Astc5x4Unorm;
    case MTLPixelFormatASTC_5x4_sRGB:
        return WebCore::WebGPU::TextureFormat::Astc5x4UnormSRGB;
    case MTLPixelFormatASTC_5x5_LDR:
        return WebCore::WebGPU::TextureFormat::Astc5x5Unorm;
    case MTLPixelFormatASTC_5x5_sRGB:
        return WebCore::WebGPU::TextureFormat::Astc5x5UnormSRGB;
    case MTLPixelFormatASTC_6x5_LDR:
        return WebCore::WebGPU::TextureFormat::Astc6x5Unorm;
    case MTLPixelFormatASTC_6x5_sRGB:
        return WebCore::WebGPU::TextureFormat::Astc6x5UnormSRGB;
    case MTLPixelFormatASTC_6x6_LDR:
        return WebCore::WebGPU::TextureFormat::Astc6x6Unorm;
    case MTLPixelFormatASTC_6x6_sRGB:
        return WebCore::WebGPU::TextureFormat::Astc6x6UnormSRGB;
    case MTLPixelFormatASTC_8x5_LDR:
        return WebCore::WebGPU::TextureFormat::Astc8x5Unorm;
    case MTLPixelFormatASTC_8x5_sRGB:
        return WebCore::WebGPU::TextureFormat::Astc8x5UnormSRGB;
    case MTLPixelFormatASTC_8x6_LDR:
        return WebCore::WebGPU::TextureFormat::Astc8x6Unorm;
    case MTLPixelFormatASTC_8x6_sRGB:
        return WebCore::WebGPU::TextureFormat::Astc8x6UnormSRGB;
    case MTLPixelFormatASTC_8x8_LDR:
        return WebCore::WebGPU::TextureFormat::Astc8x8Unorm;
    case MTLPixelFormatASTC_8x8_sRGB:
        return WebCore::WebGPU::TextureFormat::Astc8x8UnormSRGB;
    case MTLPixelFormatASTC_10x5_LDR:
        return WebCore::WebGPU::TextureFormat::Astc10x5Unorm;
    case MTLPixelFormatASTC_10x5_sRGB:
        return WebCore::WebGPU::TextureFormat::Astc10x5UnormSRGB;
    case MTLPixelFormatASTC_10x6_LDR:
        return WebCore::WebGPU::TextureFormat::Astc10x6Unorm;
    case MTLPixelFormatASTC_10x6_sRGB:
        return WebCore::WebGPU::TextureFormat::Astc10x6UnormSRGB;
    case MTLPixelFormatASTC_10x8_LDR:
        return WebCore::WebGPU::TextureFormat::Astc10x8Unorm;
    case MTLPixelFormatASTC_10x8_sRGB:
        return WebCore::WebGPU::TextureFormat::Astc10x8UnormSRGB;
    case MTLPixelFormatASTC_10x10_LDR:
        return WebCore::WebGPU::TextureFormat::Astc10x10Unorm;
    case MTLPixelFormatASTC_10x10_sRGB:
        return WebCore::WebGPU::TextureFormat::Astc10x10UnormSRGB;
    case MTLPixelFormatASTC_12x10_LDR:
        return WebCore::WebGPU::TextureFormat::Astc12x10Unorm;
    case MTLPixelFormatASTC_12x10_sRGB:
        return WebCore::WebGPU::TextureFormat::Astc12x10UnormSRGB;
    case MTLPixelFormatASTC_12x12_LDR:
        return WebCore::WebGPU::TextureFormat::Astc12x12Unorm;
    case MTLPixelFormatASTC_12x12_sRGB:
        return WebCore::WebGPU::TextureFormat::Astc12x12UnormSRGB;
#if !PLATFORM(WATCHOS)
    case MTLPixelFormatBC1_RGBA:
        return WebCore::WebGPU::TextureFormat::Bc1RgbaUnorm;
    case MTLPixelFormatBC1_RGBA_sRGB:
        return WebCore::WebGPU::TextureFormat::Bc1RgbaUnormSRGB;
    case MTLPixelFormatBC2_RGBA:
        return WebCore::WebGPU::TextureFormat::Bc2RgbaUnorm;
    case MTLPixelFormatBC2_RGBA_sRGB:
        return WebCore::WebGPU::TextureFormat::Bc2RgbaUnormSRGB;
    case MTLPixelFormatBC3_RGBA:
        return WebCore::WebGPU::TextureFormat::Bc3RgbaUnorm;
    case MTLPixelFormatBC3_RGBA_sRGB:
        return WebCore::WebGPU::TextureFormat::Bc3RgbaUnormSRGB;
    case MTLPixelFormatBC4_RUnorm:
        return WebCore::WebGPU::TextureFormat::Bc4RUnorm;
    case MTLPixelFormatBC4_RSnorm:
        return WebCore::WebGPU::TextureFormat::Bc4RSnorm;
    case MTLPixelFormatBC5_RGUnorm:
        return WebCore::WebGPU::TextureFormat::Bc5RgUnorm;
    case MTLPixelFormatBC5_RGSnorm:
        return WebCore::WebGPU::TextureFormat::Bc5RgSnorm;
    case MTLPixelFormatBC6H_RGBUfloat:
        return WebCore::WebGPU::TextureFormat::Bc6hRgbUfloat;
    case MTLPixelFormatBC6H_RGBFloat:
        return WebCore::WebGPU::TextureFormat::Bc6hRgbFloat;
    case MTLPixelFormatBC7_RGBAUnorm:
        return WebCore::WebGPU::TextureFormat::Bc7RgbaUnorm;
    case MTLPixelFormatBC7_RGBAUnorm_sRGB:
        return WebCore::WebGPU::TextureFormat::Bc7RgbaUnormSRGB;
#endif
    case MTLPixelFormatInvalid:
    default:
        return WebCore::WebGPU::TextureFormat::R8unorm;
    }
}

static WebCore::WebGPU::TextureUsageFlags toTextureUsageFlags(MTLTextureUsage textureUsage)
{
    WebCore::WebGPU::TextureUsageFlags flags;

    if (textureUsage & MTLTextureUsageShaderRead)
        flags.add(WebCore::WebGPU::TextureUsage::TextureBinding);
    if (textureUsage & MTLTextureUsageShaderWrite)
        flags.add(WebCore::WebGPU::TextureUsage::StorageBinding);
    if (textureUsage & MTLTextureUsageRenderTarget)
        flags.add(WebCore::WebGPU::TextureUsage::RenderAttachment);
    if (textureUsage & MTLTextureUsagePixelFormatView)
        flags.add(WebCore::WebGPU::TextureUsage::TextureBinding);

    return flags;
}

static WebModel::VertexAttributeFormat toCpp(WKBridgeVertexAttributeFormat *format)
{
    return WebModel::VertexAttributeFormat {
        .semantic = toVertexSemantic(format.semantic),
        .format = toVertexFormat(format.format),
        .layoutIndex = format.layoutIndex,
        .offset = format.offset
    };
}

static Vector<WebModel::VertexAttributeFormat> toCpp(NSArray<WKBridgeVertexAttributeFormat *> *formats)
{
    Vector<WebModel::VertexAttributeFormat> result;
    for (WKBridgeVertexAttributeFormat *f in formats)
        result.append(toCpp(f));
    return result;
}

static WebModel::VertexLayout toCpp(WKBridgeVertexLayout *layout)
{
    return WebModel::VertexLayout {
        .bufferIndex = layout.bufferIndex,
        .bufferOffset = layout.bufferOffset,
        .bufferStride = layout.bufferStride,
    };
}
static Vector<WebModel::VertexLayout> toCpp(NSArray<WKBridgeVertexLayout *> *layouts)
{
    Vector<WebModel::VertexLayout> result;
    for (WKBridgeVertexLayout *l in layouts)
        result.append(toCpp(l));
    return result;
}

static WebModel::MeshPart toCpp(WKBridgeMeshPart *part)
{
    return WebModel::MeshPart {
        static_cast<uint32_t>(part.indexOffset),
        static_cast<uint32_t>(part.indexCount),
        toPrimitiveTopology(part.topology),
        static_cast<uint32_t>(part.materialIndex),
        part.boundsMin,
        part.boundsMax
    };
}

static Vector<WebModel::MeshPart> toCpp(NSArray<WKBridgeMeshPart *> *parts)
{
    Vector<WebModel::MeshPart> result;
    for (WKBridgeMeshPart *p in parts)
        result.append(toCpp(p));
    return result;
}

static WebModel::MeshDescriptor toCpp(WKBridgeMeshDescriptor *descriptor)
{
    return WebModel::MeshDescriptor {
        .vertexBufferCount = descriptor.vertexBufferCount,
        .vertexCapacity = descriptor.vertexCapacity,
        .vertexAttributes = toCpp(descriptor.vertexAttributes),
        .vertexLayouts = toCpp(descriptor.vertexLayouts),
        .indexCapacity = descriptor.indexCapacity,
        .indexType = toIndexType(descriptor.indexType)
    };
}

static Vector<Vector<uint8_t>> toCpp(NSArray<NSData *> *dataVector)
{
    Vector<Vector<uint8_t>> result;
    for (NSData *data in dataVector)
        result.append(makeVector(data));

    return result;
}

template<typename T>
static Vector<T> toCpp(NSData *data)
{
    return Vector<T> { unsafeMakeSpan(static_cast<const T*>(data.bytes), data.length / sizeof(T)) };
}

template<typename T>
static Vector<Vector<T>> toCpp(NSArray<NSData *> *dataVector)
{
    Vector<Vector<T>> result;
    for (NSData *d in dataVector)
        result.append(toCpp<T>(d));

    return result;
}

static std::optional<WebModel::SkinningData> toCpp(WKBridgeSkinningData* data)
{
    if (!data)
        return std::nullopt;

    return WebModel::SkinningData {
        .influencePerVertexCount = data.influencePerVertexCount,
        .jointTransforms = toCpp<WebModel::Float4x4>(data.jointTransformsData),
        .inverseBindPoses = toCpp<WebModel::Float4x4>(data.inverseBindPosesData),
        .influenceJointIndices = toCpp<uint32_t>(data.influenceJointIndicesData),
        .influenceWeights = toCpp<float>(data.influenceWeightsData),
        .geometryBindTransform = data.geometryBindTransform
    };
}

static std::optional<WebModel::BlendShapeData> toCpp(WKBridgeBlendShapeData* data)
{
    if (!data)
        return std::nullopt;

    return WebModel::BlendShapeData {
        .weights = toCpp<float>(data.weightsData),
        .positionOffsets = toCpp<WebModel::Float3>(data.positionOffsetsData),
        .normalOffsets = toCpp<WebModel::Float3>(data.normalOffsetsData)
    };
}

static std::optional<WebModel::RenormalizationData> toCpp(WKBridgeRenormalizationData* data)
{
    if (!data)
        return std::nullopt;

    return WebModel::RenormalizationData {
        .vertexIndicesPerTriangle = toCpp<uint32_t>(data.vertexIndicesPerTriangleData),
        .vertexAdjacencies = toCpp<uint32_t>(data.vertexAdjacenciesData),
        .vertexAdjacencyEndIndices = toCpp<uint32_t>(data.vertexAdjacencyEndIndicesData)
    };
}

static std::optional<WebModel::DeformationData> toCpp(WKBridgeDeformationData* data)
{
    if (!data)
        return std::nullopt;

    return WebModel::DeformationData {
        .skinningData = toCpp(data.skinningData),
        .blendShapeData = toCpp(data.blendShapeData),
        .renormalizationData = toCpp(data.renormalizationData)
    };
}

static WebModel::TypedResourceId toCpp(WKBridgeTypedResourceId *update)
{
    return WebModel::TypedResourceId {
        .value = update.value,
        .path = update.path,
        .hashValue = update.cachedHashValue
    };
}


static WebModel::NodeType toCpp(WKBridgeNodeType nodeType)
{
    switch (nodeType) {
    case WKBridgeNodeTypeBuiltin:
        return WebModel::NodeType::Builtin;
    case WKBridgeNodeTypeConstant:
        return WebModel::NodeType::Constant;
    case WKBridgeNodeTypeArguments:
        return WebModel::NodeType::Arguments;
    default:
    case WKBridgeNodeTypeResults:
        return WebModel::NodeType::Results;
    }
}

static WebModel::Builtin toCpp(WKBridgeBuiltin *builtin)
{
    return WebModel::Builtin {
        .definition = builtin.definition,
        .name = builtin.name
    };
}

static WebModel::Constant toCpp(WKBridgeConstant constant)
{
    switch (constant) {
    case WKBridgeConstantBool:
        return WebModel::Constant::kBool;
    case WKBridgeConstantUchar:
        return WebModel::Constant::kUchar;
    case WKBridgeConstantInt:
        return WebModel::Constant::kInt;
    case WKBridgeConstantUint:
        return WebModel::Constant::kUint;
    case WKBridgeConstantHalf:
        return WebModel::Constant::kHalf;
    case WKBridgeConstantFloat:
        return WebModel::Constant::kFloat;
    case WKBridgeConstantTimecode:
        return WebModel::Constant::kTimecode;
    case WKBridgeConstantString:
        return WebModel::Constant::kString;
    case WKBridgeConstantToken:
        return WebModel::Constant::kToken;
    case WKBridgeConstantAsset:
        return WebModel::Constant::kAsset;
    case WKBridgeConstantMatrix2f:
        return WebModel::Constant::kMatrix2f;
    case WKBridgeConstantMatrix3f:
        return WebModel::Constant::kMatrix3f;
    case WKBridgeConstantMatrix4f:
        return WebModel::Constant::kMatrix4f;
    case WKBridgeConstantQuatf:
        return WebModel::Constant::kQuatf;
    case WKBridgeConstantQuath:
        return WebModel::Constant::kQuath;
    case WKBridgeConstantFloat2:
        return WebModel::Constant::kFloat2;
    case WKBridgeConstantHalf2:
        return WebModel::Constant::kHalf2;
    case WKBridgeConstantInt2:
        return WebModel::Constant::kInt2;
    case WKBridgeConstantFloat3:
        return WebModel::Constant::kFloat3;
    case WKBridgeConstantHalf3:
        return WebModel::Constant::kHalf3;
    case WKBridgeConstantInt3:
        return WebModel::Constant::kInt3;
    case WKBridgeConstantFloat4:
        return WebModel::Constant::kFloat4;
    case WKBridgeConstantHalf4:
        return WebModel::Constant::kHalf4;
    case WKBridgeConstantInt4:
        return WebModel::Constant::kInt4;

    case WKBridgeConstantPoint3f:
        return WebModel::Constant::kPoint3f;
    case WKBridgeConstantPoint3h:
        return WebModel::Constant::kPoint3h;
    case WKBridgeConstantNormal3f:
        return WebModel::Constant::kNormal3f;
    case WKBridgeConstantNormal3h:
        return WebModel::Constant::kNormal3h;
    case WKBridgeConstantVector3f:
        return WebModel::Constant::kVector3f;
    case WKBridgeConstantVector3h:
        return WebModel::Constant::kVector3h;
    case WKBridgeConstantColor3f:
        return WebModel::Constant::kColor3f;
    case WKBridgeConstantColor3h:
        return WebModel::Constant::kColor3h;
    case WKBridgeConstantColor4f:
        return WebModel::Constant::kColor4f;
    case WKBridgeConstantColor4h:
        return WebModel::Constant::kColor4h;
    case WKBridgeConstantTexCoord2h:
        return WebModel::Constant::kTexCoord2h;
    case WKBridgeConstantTexCoord2f:
        return WebModel::Constant::kTexCoord2f;
    case WKBridgeConstantTexCoord3h:
        return WebModel::Constant::kTexCoord3h;
    case WKBridgeConstantTexCoord3f:
        return WebModel::Constant::kTexCoord3f;
    }
}

static Vector<WebModel::NumberOrString> toCpp(NSArray<WKBridgeValueString *> *constantValues)
{
    Vector<WebModel::NumberOrString> result;
    result.reserveCapacity(constantValues.count);
    for (WKBridgeValueString* v in constantValues) {
        if (v.isString)
            result.append(String(v.string));
        else
            result.append(v.number.doubleValue);
    }

    return result;
}

static WebModel::ConstantContainer toCpp(WKBridgeConstantContainer *container)
{
    return WebModel::ConstantContainer {
        .constant = toCpp(container.constant),
        .constantValues = toCpp(container.constantValues),
        .name = String(container.name)
    };
}

static WebModel::Node toCpp(WKBridgeNode *node)
{
    return WebModel::Node {
        .bridgeNodeType = toCpp(node.bridgeNodeType),
        .builtin = toCpp(node.builtin),
        .constant = toCpp(node.constant)
    };
}

static WebModel::Edge toCpp(WKBridgeEdge *edge)
{
    return WebModel::Edge {
        .outputNode = String(edge.outputNode),
        .outputPort = String(edge.outputPort),
        .inputNode = String(edge.inputNode),
        .inputPort = String(edge.inputPort)
    };
}

static WebModel::DataType toCpp(WKBridgeDataType type)
{
    switch (type) {
    case WKBridgeDataTypeBool:
        return WebModel::DataType::kBool;
    case WKBridgeDataTypeUchar:
        return WebModel::DataType::kUchar;
    case WKBridgeDataTypeInt:
        return WebModel::DataType::kInt;
    case WKBridgeDataTypeUint:
        return WebModel::DataType::kUint;
    case WKBridgeDataTypeInt2:
        return WebModel::DataType::kInt2;
    case WKBridgeDataTypeInt3:
        return WebModel::DataType::kInt3;
    case WKBridgeDataTypeInt4:
        return WebModel::DataType::kInt4;
    case WKBridgeDataTypeFloat:
        return WebModel::DataType::kFloat;
    case WKBridgeDataTypeColor3f:
        return WebModel::DataType::kColor3f;
    case WKBridgeDataTypeColor3h:
        return WebModel::DataType::kColor3h;
    case WKBridgeDataTypeColor4f:
        return WebModel::DataType::kColor4f;
    case WKBridgeDataTypeColor4h:
        return WebModel::DataType::kColor4h;
    case WKBridgeDataTypeFloat2:
        return WebModel::DataType::kFloat2;
    case WKBridgeDataTypeFloat3:
        return WebModel::DataType::kFloat3;
    case WKBridgeDataTypeFloat4:
        return WebModel::DataType::kFloat4;
    case WKBridgeDataTypeHalf:
        return WebModel::DataType::kHalf;
    case WKBridgeDataTypeHalf2:
        return WebModel::DataType::kHalf2;
    case WKBridgeDataTypeHalf3:
        return WebModel::DataType::kHalf3;
    case WKBridgeDataTypeHalf4:
        return WebModel::DataType::kHalf4;
    case WKBridgeDataTypeMatrix2f:
        return WebModel::DataType::kMatrix2f;
    case WKBridgeDataTypeMatrix3f:
        return WebModel::DataType::kMatrix3f;
    case WKBridgeDataTypeMatrix4f:
        return WebModel::DataType::kMatrix4f;
    case WKBridgeDataTypeMatrix2h:
        return WebModel::DataType::kMatrix2h;
    case WKBridgeDataTypeMatrix3h:
        return WebModel::DataType::kMatrix3h;
    case WKBridgeDataTypeMatrix4h:
        return WebModel::DataType::kMatrix4h;
    case WKBridgeDataTypeQuat:
        return WebModel::DataType::kQuat;
    case WKBridgeDataTypeSurfaceShader:
        return WebModel::DataType::kSurfaceShader;
    case WKBridgeDataTypeGeometryModifier:
        return WebModel::DataType::kGeometryModifier;
    case WKBridgeDataTypePostLightingShader:
        return WebModel::DataType::kPostLightingShader;
    case WKBridgeDataTypeString:
        return WebModel::DataType::kString;
    case WKBridgeDataTypeToken:
        return WebModel::DataType::kToken;
    case WKBridgeDataTypeAsset:
        return WebModel::DataType::kAsset;
    default:
        RELEASE_ASSERT_NOT_REACHED("USD file is corrupt");
    }
}

static WebModel::InputOutput toCpp(WKBridgeInputOutput *inputOutput)
{
    std::optional<WebModel::DataType> semanticType;
    if (inputOutput.hasSemanticType)
        semanticType = toCpp(inputOutput.semanticType);

    std::optional<WebModel::ConstantContainer> defaultValue;
    if (inputOutput.defaultValue)
        defaultValue = toCpp(inputOutput.defaultValue);

    return WebModel::InputOutput {
        .type = toCpp(inputOutput.type),
        .name = String(inputOutput.name),
        .semanticType = semanticType,
        .defaultValue = defaultValue
    };
}

template<typename T, typename U>
static Vector<U> toCpp(NSArray<T *> *nsArray)
{
    Vector<U> result;
    result.reserveCapacity(nsArray.count);
    for (T *v in nsArray)
        result.append(toCpp(v));

    return result;
}

static WebModel::UpdateMeshDescriptor toCpp(WKBridgeUpdateMesh *update)
{
    return WebModel::UpdateMeshDescriptor {
        .identifier = toCpp(update.identifier),
        .updateType = static_cast<uint8_t>(update.updateType),
        .descriptor = toCpp(update.descriptor),
        .parts = toCpp(update.parts),
        .indexData = makeVector(update.indexData),
        .vertexData = toCpp(update.vertexData),
        .instanceTransforms = toCpp<WebModel::Float4x4>(update.instanceTransformsData),
        .assignedMaterials = toCpp<WKBridgeTypedResourceId, WebModel::TypedResourceId>(update.assignedMaterials),
        .deformationData = toCpp(update.deformationData)
    };
}

static WebModel::MaterialGraph toCpp(WKBridgeMaterialGraph *materialGraph)
{
    return WebModel::MaterialGraph {
        .nodes = toCpp<WKBridgeNode, WebModel::Node>(materialGraph.nodes),
        .edges = toCpp<WKBridgeEdge, WebModel::Edge>(materialGraph.edges),
        .arguments = toCpp(materialGraph.arguments),
        .results = toCpp(materialGraph.results),
        .inputs = toCpp<WKBridgeInputOutput, WebModel::InputOutput>(materialGraph.inputs),
        .outputs = toCpp<WKBridgeInputOutput, WebModel::InputOutput>(materialGraph.outputs),
    };
}

static WebModel::ImageAssetSwizzle convert(MTLTextureSwizzleChannels swizzle)
{
    return WebModel::ImageAssetSwizzle {
        .red = swizzle.red,
        .green = swizzle.green,
        .blue = swizzle.blue,
        .alpha = swizzle.alpha
    };
}

static WebModel::ImageAsset convert(WKBridgeImageAsset *imageAsset)
{
    return WebModel::ImageAsset {
        .data = makeVector(imageAsset.data),
        .width = imageAsset.width,
        .height = imageAsset.height,
        .depth = 1,
        .bytesPerPixel = imageAsset.bytesPerPixel,
        .textureType = toTextureViewDimension(imageAsset.textureType),
        .pixelFormat = toTextureFormat(imageAsset.pixelFormat),
        .mipmapLevelCount = imageAsset.mipmapLevelCount,
        .arrayLength = imageAsset.arrayLength,
        .textureUsage = toTextureUsageFlags(imageAsset.textureUsage),
        .swizzle = convert(imageAsset.swizzle)
    };
}

static WebModel::UpdateTextureDescriptor toCpp(WKBridgeUpdateTexture *update)
{
    return WebModel::UpdateTextureDescriptor {
        .imageAsset = convert(update.imageAsset),
        .identifier = toCpp(update.identifier),
        .hashString = update.hashString
    };
}

static WebModel::UpdateMaterialDescriptor toCpp(WKBridgeUpdateMaterial *update)
{
    return WebModel::UpdateMaterialDescriptor {
        .materialGraph = toCpp(update.materialGraph),
        .identifier = toCpp(update.identifier)
    };
}

}

#endif

