// Copyright (C) 2024 Apple Inc. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY APPLE INC. AND ITS CONTRIBUTORS ``AS IS''
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL APPLE INC. OR ITS CONTRIBUTORS
// BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
// THE POSSIBILITY OF SUCH DAMAGE.

#if canImport(RealityCoreRenderer)
import Metal
import WebGPU_Internal

@_spi(RealityCoreRendererAPI) @_spi(ShaderGraph) import RealityCoreRenderer
@_spi(RealityCoreRendererAPI) @_spi(ShaderGraph) import RealityKit
@_spi(UsdLoaderAPI) import USDStageKit
@_spi(UsdLoaderAPI) import _USDStageKit_SwiftUI
@_spi(SwiftAPI) import DirectResource
import USDStageKit
import _USDStageKit_SwiftUI
import RealityKit
import ShaderGraph

@objc
@implementation
extension DDBridgeVertexAttributeFormat {
    let semantic: Int
    let format: UInt
    let layoutIndex: Int
    let offset: Int

    init(
        semantic: Int,
        format: UInt,
        layoutIndex: Int,
        offset: Int
    ) {
        self.semantic = semantic
        self.format = format
        self.layoutIndex = layoutIndex
        self.offset = offset
    }
}

@objc
@implementation
extension DDBridgeVertexLayout {
    let bufferIndex: Int
    let bufferOffset: Int
    let bufferStride: Int

    init(
        bufferIndex: Int,
        bufferOffset: Int,
        bufferStride: Int
    ) {
        self.bufferIndex = bufferIndex
        self.bufferOffset = bufferOffset
        self.bufferStride = bufferStride
    }
}

@objc
@implementation
extension DDBridgeMeshPart {
    let indexOffset: Int
    let indexCount: Int
    let topology: MTLPrimitiveType
    let materialIndex: Int
    let boundsMin: simd_float3
    let boundsMax: simd_float3

    init(
        indexOffset: Int,
        indexCount: Int,
        topology: MTLPrimitiveType,
        materialIndex: Int,
        boundsMin: simd_float3,
        boundsMax: simd_float3
    ) {
        self.indexOffset = indexOffset
        self.indexCount = indexCount
        self.topology = topology
        self.materialIndex = materialIndex
        self.boundsMin = boundsMin
        self.boundsMax = boundsMax
    }
}

@objc
@implementation
extension DDBridgeMeshDescriptor {
    let vertexBufferCount: Int
    let vertexCapacity: Int
    let vertexAttributes: [DDBridgeVertexAttributeFormat]
    let vertexLayouts: [DDBridgeVertexLayout]
    let indexCapacity: Int
    let indexType: MTLIndexType

    init(
        vertexBufferCount: Int,
        vertexCapacity: Int,
        vertexAttributes: [DDBridgeVertexAttributeFormat],
        vertexLayouts: [DDBridgeVertexLayout],
        indexCapacity: Int,
        indexType: MTLIndexType
    ) {
        self.vertexBufferCount = vertexBufferCount
        self.vertexCapacity = vertexCapacity
        self.vertexAttributes = vertexAttributes
        self.vertexLayouts = vertexLayouts
        self.indexCapacity = indexCapacity
        self.indexType = indexType
    }
}

@objc
@implementation
extension DDBridgeChainedFloat4x4 {
    var transform: simd_float4x4
    var next: DDBridgeChainedFloat4x4?

    init(
        transform: simd_float4x4
    ) {
        self.transform = transform
    }
}

@objc
@implementation
extension DDBridgeUpdateMesh {
    let identifier: String
    let updateType: DDBridgeDataUpdateType
    let descriptor: DDBridgeMeshDescriptor?
    let parts: [DDBridgeMeshPart]
    let indexData: Data?
    let vertexData: [Data]
    var instanceTransforms: DDBridgeChainedFloat4x4? // array of float4x4
    let instanceTransformsCount: Int
    let materialPrims: [String]

    init(
        identifier: String,
        updateType: DDBridgeDataUpdateType,
        descriptor: DDBridgeMeshDescriptor?,
        parts: [DDBridgeMeshPart],
        indexData: Data?,
        vertexData: [Data],
        instanceTransforms: DDBridgeChainedFloat4x4?,
        instanceTransformsCount: Int,
        materialPrims: [String]
    ) {
        self.identifier = identifier
        self.updateType = updateType
        self.descriptor = descriptor
        self.parts = parts
        self.indexData = indexData
        self.vertexData = vertexData
        self.instanceTransforms = instanceTransforms
        self.instanceTransformsCount = instanceTransformsCount
        self.materialPrims = materialPrims
    }
}

@objc
@implementation
extension DDBridgeImageAsset {
    let data: Data?
    let width: Int
    let height: Int
    let depth: Int
    let bytesPerPixel: Int
    let textureType: MTLTextureType
    let pixelFormat: MTLPixelFormat
    let mipmapLevelCount: Int
    let arrayLength: Int
    let textureUsage: MTLTextureUsage
    let swizzle: MTLTextureSwizzleChannels

    init(
        data: Data?,
        width: Int,
        height: Int,
        depth: Int,
        bytesPerPixel: Int,
        textureType: MTLTextureType,
        pixelFormat: MTLPixelFormat,
        mipmapLevelCount: Int,
        arrayLength: Int,
        textureUsage: MTLTextureUsage,
        swizzle: MTLTextureSwizzleChannels
    ) {
        self.data = data
        self.width = width
        self.height = height
        self.depth = depth
        self.bytesPerPixel = bytesPerPixel
        self.textureType = textureType
        self.pixelFormat = pixelFormat
        self.mipmapLevelCount = mipmapLevelCount
        self.arrayLength = arrayLength
        self.textureUsage = textureUsage
        self.swizzle = swizzle
    }
}

@objc
@implementation
extension DDBridgeUpdateTexture {
    let imageAsset: DDBridgeImageAsset?
    let identifier: String
    let hashString: String

    init(
        imageAsset: DDBridgeImageAsset?,
        identifier: String,
        hashString: String
    ) {
        self.imageAsset = imageAsset
        self.identifier = identifier
        self.hashString = hashString
    }
}

@objc
@implementation
extension DDBridgeUpdateMaterial {
    let materialGraph: Data?
    let identifier: String

    init(
        materialGraph: Data?,
        identifier: String
    ) {
        self.materialGraph = materialGraph
        self.identifier = identifier
    }
}

@objc
@implementation
extension DDBridgeNode {
    let bridgeNodeType: DDBridgeNodeType
    let builtin: DDBridgeBuiltin
    let constant: DDBridgeConstantContainer

    init(
        bridgeNodeType: DDBridgeNodeType,
        builtin: DDBridgeBuiltin,
        constant: DDBridgeConstantContainer
    ) {
        self.bridgeNodeType = bridgeNodeType
        self.builtin = builtin
        self.constant = constant
    }
}

@objc
@implementation
extension DDBridgeInputOutput {
    let type: DDBridgeDataType
    let name: String

    init(
        type: DDBridgeDataType,
        name: String
    ) {
        self.type = type
        self.name = name
    }
}

@objc
@implementation
extension DDBridgePrimvar {
    let name: String
    let referencedGeomPropName: String
    let attributeFormat: UInt

    init(
        name: String,
        referencedGeomPropName: String,
        attributeFormat: UInt
    ) {
        self.name = name
        self.referencedGeomPropName = referencedGeomPropName
        self.attributeFormat = attributeFormat
    }
}

@objc
@implementation
extension DDBridgeConstantContainer {
    let constant: DDBridgeConstant
    let constantValues: [DDValueString]
    let name: String

    init(
        constant: DDBridgeConstant,
        constantValues: [DDValueString],
        name: String
    ) {
        self.constant = constant
        self.constantValues = constantValues
        self.name = name
    }
}

@objc
@implementation
extension DDBridgeBuiltin {
    let definition: String
    let name: String

    init(
        definition: String,
        name: String
    ) {
        self.definition = definition
        self.name = name
    }
}

@objc
@implementation
extension DDBridgeEdge {
    let upstreamNodeIndex: Int
    let downstreamNodeIndex: Int
    let upstreamOutputName: String
    let downstreamInputName: String

    init(
        upstreamNodeIndex: Int,
        downstreamNodeIndex: Int,
        upstreamOutputName: String,
        downstreamInputName: String
    ) {
        self.upstreamNodeIndex = upstreamNodeIndex
        self.downstreamNodeIndex = downstreamNodeIndex
        self.upstreamOutputName = upstreamOutputName
        self.downstreamInputName = downstreamInputName
    }
}

@objc
@implementation
extension DDBridgeMaterialGraph {
    let nodes: [DDBridgeNode]
    let edges: [DDBridgeEdge]
    let inputs: [DDBridgeInputOutput]
    let outputs: [DDBridgeInputOutput]
    let primvars: [DDBridgePrimvar]

    init(
        nodes: [DDBridgeNode],
        edges: [DDBridgeEdge],
        inputs: [DDBridgeInputOutput],
        outputs: [DDBridgeInputOutput],
        primvars: [DDBridgePrimvar],
    ) {
        self.nodes = nodes
        self.edges = edges
        self.inputs = inputs
        self.outputs = outputs
        self.primvars = primvars
    }
}

@objc
@implementation
extension DDValueString {
    let number: NSNumber
    let string: String

    init(
        string: String
    ) {
        self.number = NSNumber(value: 0)
        self.string = string
    }

    init(
        number: NSNumber
    ) {
        self.number = number
        self.string = ""
    }
}

extension MTLCaptureDescriptor {
    fileprivate convenience init(from device: MTLDevice?) {
        self.init()

        captureObject = device
        destination = .gpuTraceDocument
        let now = Date()
        let dateFormatter = DateFormatter()
        dateFormatter.timeZone = .current
        dateFormatter.dateFormat = "yyyy-MM-dd-HH-mm-ss-SSSS"
        let dateString = dateFormatter.string(from: now)

        outputURL = URL.temporaryDirectory.appending(path: "capture_\(dateString).gputrace").standardizedFileURL
    }
}

nonisolated func mapSemantic(_ semantic: Int) -> _Proto_LowLevelMeshResource_v1.VertexSemantic {
    switch semantic {
    case 0: return .position
    case 1: return .color
    case 2: return .normal
    case 3: return .tangent
    case 4: return .bitangent
    case 5: return .uv0
    case 6: return .uv1
    case 7: return .uv2
    case 8: return .uv3
    case 9: return .uv4
    case 10: return .uv5
    case 11: return .uv6
    case 12: return .uv7
    default:
        return .unspecified
    }
}

extension _Proto_LowLevelMeshResource_v1.Descriptor {
    nonisolated static func fromLlmDescriptor(_ llmDescriptor: DDBridgeMeshDescriptor) -> Self {
        var descriptor = Self.init()
        descriptor.vertexCapacity = Int(llmDescriptor.vertexCapacity)
        descriptor.vertexAttributes = llmDescriptor.vertexAttributes.map { attribute in
            .init(
                semantic: mapSemantic(attribute.semantic),
                format: MTLVertexFormat(rawValue: UInt(attribute.format)) ?? .invalid,
                layoutIndex: attribute.layoutIndex,
                offset: attribute.offset
            )
        }
        descriptor.vertexLayouts = llmDescriptor.vertexLayouts.map { layout in
            .init(bufferIndex: layout.bufferIndex, bufferOffset: layout.bufferOffset, bufferStride: layout.bufferStride)
        }
        descriptor.indexCapacity = llmDescriptor.indexCapacity
        descriptor.indexType = llmDescriptor.indexType

        return descriptor
    }
}

extension _Proto_LowLevelTextureResource_v1.Descriptor {
    static func from(_ texture: MTLTexture) -> _Proto_LowLevelTextureResource_v1.Descriptor {
        var descriptor = _Proto_LowLevelTextureResource_v1.Descriptor()
        descriptor.width = texture.width
        descriptor.height = texture.height
        descriptor.depth = texture.depth
        descriptor.mipmapLevelCount = texture.mipmapLevelCount
        descriptor.arrayLength = texture.arrayLength
        descriptor.pixelFormat = texture.pixelFormat
        descriptor.textureType = texture.textureType
        descriptor.textureUsage = texture.usage
        descriptor.swizzle = texture.swizzle

        return descriptor
    }
}

class Helper {
    static fileprivate func isNonZero(value: Float) -> Bool {
        abs(value) > Float.ulpOfOne
    }

    static fileprivate func isNonZero(_ vector: simd_float4) -> Bool {
        isNonZero(value: vector[0]) || isNonZero(value: vector[1]) || isNonZero(value: vector[2]) || isNonZero(value: vector[3])
    }

    static fileprivate func isNonZero(matrix: simd_float4x4) -> Bool {
        isNonZero(_: matrix.columns.0) || isNonZero(_: matrix.columns.1) || isNonZero(_: matrix.columns.2) || isNonZero(_: matrix.columns.3)
    }

    static fileprivate func makeTextureFromImageAsset(
        _ imageAsset: DDBridgeImageAsset,
        device: MTLDevice,
        context: _Proto_LowLevelResourceContext_v1,
        commandQueue: MTLCommandQueue
    ) -> _Proto_LowLevelTextureResource_v1? {
        guard let imageAssetData = imageAsset.data else {
            logError("no image data")
            return nil
        }
        do {
            logError(
                "imageAssetData = \(imageAssetData)  -  width = \(imageAsset.width)  -  height = \(imageAsset.height)  -  bytesPerPixel = \(imageAsset.bytesPerPixel) imageAsset.pixelFormat:  \(imageAsset.pixelFormat)"
            )

            var pixelFormat = imageAsset.pixelFormat
            switch imageAsset.bytesPerPixel {
            case 1:
                pixelFormat = .r8Unorm
            case 2:
                pixelFormat = .rg8Unorm
            case 4:
                pixelFormat = .rgba8Unorm
            default:
                pixelFormat = .rgba8Unorm
            }
            let textureDescriptor = MTLTextureDescriptor.texture2DDescriptor(
                pixelFormat: pixelFormat,
                width: imageAsset.width,
                height: imageAsset.height,
                mipmapped: false
            )

            guard let mtlTexture = device.makeTexture(descriptor: textureDescriptor) else {
                logError("failed to device.makeTexture")
                return nil
            }

            try unsafe imageAssetData.bytes.withUnsafeBytes { textureBytes in
                guard let textureBytesBaseAddress = textureBytes.baseAddress else {
                    return
                }
                if imageAsset.bytesPerPixel == 0 {
                    logError("bytesPerPixel == 0")
                    fatalError()
                }
                unsafe mtlTexture.replace(
                    region: MTLRegionMake2D(0, 0, imageAsset.width, imageAsset.height),
                    mipmapLevel: 0,
                    withBytes: textureBytesBaseAddress,
                    bytesPerRow: imageAsset.width * imageAsset.bytesPerPixel
                )
            }

            let commandBuffer = commandQueue.makeCommandBuffer()!
            let blitEncoder = commandBuffer.makeBlitCommandEncoder()!
            let descriptor = _Proto_LowLevelTextureResource_v1.Descriptor.from(mtlTexture)
            if let textureResource = try? context.makeTextureResource(descriptor: descriptor) {
                let outTexture = textureResource.replace(using: commandBuffer)
                blitEncoder.copy(from: mtlTexture, to: outTexture)

                blitEncoder.endEncoding()
                commandBuffer.commit()
                return textureResource
            }
        } catch {
            logError("failed to make texture from image \(error)")
        }

        return nil
    }

    static fileprivate func makeParameters(
        parameterMapping: _Proto_LowLevelMaterialParameterMapping_v1,
        stageDescriptor: _Proto_LowLevelMaterialParameterTable_v1.Descriptor.Stage,
        textureResources: [String: _Proto_LowLevelTextureResource_v1] = [:],
        resourceContext: _Proto_LowLevelResourceContext_v1
    ) throws -> (buffers: [_Proto_LowLevelBufferResource_v1], textures: [_Proto_LowLevelTextureResource_v1]) {
        var optTextures: [_Proto_LowLevelTextureResource_v1?] = stageDescriptor.textures.map({ _ in nil })
        for parameter in parameterMapping.textures {
            guard let textureResource = textureResources[parameter.name] else {
                fatalError("Failed to find texture resource \(parameter.name)")
            }
            optTextures[parameter.textureIndex] = textureResource
        }
        let textures = optTextures.map({ $0! })

        let buffers: [_Proto_LowLevelBufferResource_v1] = try stageDescriptor.buffers.map { bufferRequirements in
            let capacity = (bufferRequirements.size + 16 - 1) / 16 * 16
            let buffer = try resourceContext.makeBufferResource(descriptor: .init(capacity: capacity))
            buffer.replace { span in
                for byteOffset in span.byteOffsets {
                    span.storeBytes(of: 0, toByteOffset: byteOffset, as: UInt8.self)
                }
            }
            return buffer
        }

        return (buffers: buffers, textures: textures)
    }
}

extension Logger {
    fileprivate static let modelGPU = Logger(subsystem: "com.apple.WebKit", category: "model")
}

nonisolated func logError(_ error: String) {
    Logger.modelGPU.error("\(error)")
}

nonisolated func logInfo(_ info: String) {
    Logger.modelGPU.info("\(info)")
}

extension _Proto_LowLevelMeshResource_v1 {
    nonisolated func replaceVertexData(_ vertexData: [Data]) {
        for (vertexBufferIndex, vertexData) in vertexData.enumerated() {
            let bufferSizeInByte = vertexData.bytes.byteCount
            self.replaceVertices(at: vertexBufferIndex) { vertexBytes in
                unsafe vertexBytes.withUnsafeMutableBytes { ptr in
                    guard let baseAddress = ptr.baseAddress else {
                        return
                    }
                    unsafe vertexData.copyBytes(to: baseAddress.assumingMemoryBound(to: UInt8.self), count: bufferSizeInByte)
                }
            }
        }
    }

    nonisolated func replaceIndexData(_ indexData: Data?) {
        if let indexData = indexData {
            self.replaceIndices { indicesBytes in
                unsafe indicesBytes.withUnsafeMutableBytes { ptr in
                    guard let baseAddress = ptr.baseAddress else {
                        return
                    }
                    unsafe indexData.copyBytes(to: baseAddress.assumingMemoryBound(to: UInt8.self), count: ptr.count)
                }
            }
        }
    }

    nonisolated func replaceData(replaceParts: [DDBridgeMeshPart], replaceIndexData: Data?, replaceVertexData: [Data]) {
        let partCount = replaceParts.count

        // Copy mesh parts
        self.modifyParts { parts in
            // why are there 8 parts?
            for i in 0..<partCount {
                let part = replaceParts[i]
                parts[i] = .init(
                    indexOffset: part.indexOffset,
                    indexCount: part.indexCount,
                    primitive: part.topology,
                    windingOrder: .counterClockwise,
                    boundsMin: part.boundsMin,
                    boundsMax: part.boundsMax
                )
            }
        }

        // Copy index data
        self.replaceIndexData(replaceIndexData)

        // Copy vertex data
        self.replaceVertexData(replaceVertexData)
    }
}

@objc
@implementation
extension DDBridgeReceiver {
    fileprivate let device: MTLDevice

    @nonobjc
    fileprivate let resourceContext: _Proto_LowLevelResourceContext_v1
    @nonobjc
    fileprivate let scene: _Proto_LowLevelScene_v1
    @nonobjc
    fileprivate var meshInstances: [String: [(_Proto_LowLevelMeshInstance_v1, simd_float4x4)]] = [:]
    @nonobjc
    fileprivate var meshResources: [String: _Proto_LowLevelMeshResource_v1] = [:]
    @nonobjc
    fileprivate var camera: _Proto_LowLevelCamera_v1?
    @nonobjc
    fileprivate var renderer: _Proto_LowLevelRenderer_v1?
    @nonobjc
    fileprivate var materialInstances: [_Proto_ResourceId: _Proto_LowLevelMaterialInstance_v1] = [:]
    @nonobjc
    fileprivate var textureResources: [String: _Proto_LowLevelTextureResource_v1] = [:]
    @nonobjc
    fileprivate var textureData: [_Proto_ResourceId: (MTLTexture, String)] = [:]
    @nonobjc
    fileprivate let commandQueue: MTLCommandQueue?
    @nonobjc
    fileprivate let captureManager: MTLCaptureManager
    @nonobjc
    fileprivate var textureFromPath: [String: String] = [:]

    @nonobjc
    fileprivate var modelTransform: simd_float4x4
    @nonobjc
    fileprivate var modelDistance: Float

    @nonobjc
    private let dispatchSerialQueue: DispatchSerialQueue

    init(
        device: MTLDevice
    ) {
        self.device = device
        self.dispatchSerialQueue = DispatchSerialQueue(label: "USDModelGPUProcess", qos: .userInteractive)
        modelTransform = matrix_identity_float4x4
        modelDistance = 1.0

        resourceContext = _Proto_LowLevelResourceContext_v1(device: device)
        scene = _Proto_LowLevelScene_v1()
        commandQueue = device.makeCommandQueue()
        captureManager = MTLCaptureManager.shared()
    }

    @objc(initRenderer:completionHandler:)
    func initRenderer(_ texture: MTLTexture) async {
        do {
            renderer = try await _Proto_LowLevelRenderer_v1(configuration: .init(device: device, commandQueue: commandQueue))
        } catch {
            logError("failed to create renderer")
        }
    }

    @objc(renderWithTexture:)
    func render(with texture: MTLTexture) {
        let instancesAndTransforms = meshInstances.flatMap(\.value)

        // Add instances to scene
        for (instance, _) in instancesAndTransforms {
            scene.meshInstances.insert(instance)
        }

        for instance in scene.meshInstances {
            guard let (_, originalTransform) = instancesAndTransforms.first(where: { $0.0 === instance }) else {
                continue
            }
            instance.transform = .single(modelTransform * originalTransform)
        }

        // render
        let captureDescriptor = MTLCaptureDescriptor(from: device)
        do {
            try captureManager.startCapture(with: captureDescriptor)
            print("Capture started at \(captureDescriptor.outputURL?.absoluteString ?? "")")
        } catch {
            logInfo("failed to start gpu capture \(error)")
        }

        let aspect = Float(texture.width) / Float(texture.height)
        let projection = _Proto_LowLevelCamera_v1.Projection.perspective(
            fovYRadians: 90 * .pi / 180,
            aspectRatio: aspect,
            nearZ: 0.01,
            farZ: 100.0,
            reverseZ: true
        )

        if camera == nil {
            camera = _Proto_LowLevelCamera_v1(
                target: .texture(color: texture),
                pose: .init(
                    translation: [0, 0, 1],
                    rotation: simd_quatf(angle: 0, axis: [0, 0, 1])
                ),
                projection: projection
            )
        }
        guard let camera else { fatalError("Camera not initialized") }
        guard let renderer else { fatalError("Renderer not initialized") }

        camera.target = .texture(color: texture)
        camera.projection = projection
        camera.pose = .init(
            translation: [0, 0, modelDistance],
            rotation: simd_quatf(angle: 0, axis: [0, 0, 1])
        )

        renderer.render(configuration: .init(scene: scene, cameras: [camera]))

        captureManager.stopCapture()
    }

    fileprivate func updateTextureAsync(_ data: DDBridgeUpdateTexture) {
        do {
            guard let asset = data.imageAsset else {
                logError("Image asset was nil")
                return
            }

            let textureHash = data.hashString
            if textureResources[textureHash] != nil {
                logError("Texture already exists")
                return
            }

            if let commandQueue {
                if let newTexture = try Helper.makeTextureFromImageAsset(
                    asset,
                    device: device,
                    context: resourceContext,
                    commandQueue: commandQueue
                ) {
                    textureResources[textureHash] = newTexture
                }
            }
        } catch {
            return
        }
    }

    @objc(updateTexture:)
    func updateTexture(_ request: DDBridgeUpdateTexture) {
        self.dispatchSerialQueue.async {
            self.updateTextureAsync(request)
        }
    }

    fileprivate func updateMaterialAsync(_ data: DDBridgeUpdateMaterial) {
        let identifier = data.identifier
        let materialSourceArchive = data.materialGraph
        do {
            let materialSource = try ShaderGraphService.sourceFromArchive(data: materialSourceArchive)
            let sgMaterial = try ShaderGraphService.materialFromSource(materialSource, functionConstantValues: .init())

            let materialResource = try _Proto_LowLevelMaterialResource_v1.makeShaderGraphMaterial(sgMaterial, device: device)
            let (vertexBuffers, vertexTextures) = try Helper.makeParameters(
                parameterMapping: materialResource.vertexParameterMapping,
                stageDescriptor: materialResource.parameterTableDescriptor.vertex,
                textureResources: self.textureResources,
                resourceContext: self.resourceContext
            )
            let (fragmentBuffers, fragmentTextures) = try Helper.makeParameters(
                parameterMapping: materialResource.fragmentParameterMapping,
                stageDescriptor: materialResource.parameterTableDescriptor.fragment,
                textureResources: self.textureResources,
                resourceContext: self.resourceContext
            )

            let materialParameterTable = try resourceContext.makeMaterialParameterTable(
                materialResource: materialResource,
                vertexBuffers: vertexBuffers,
                vertexTextures: vertexTextures,
                fragmentBuffers: fragmentBuffers,
                fragmentTextures: fragmentTextures
            )

            let materialInstance = try _Proto_LowLevelMaterialInstance_v1(resource: materialResource, parameters: materialParameterTable)
            self.materialInstances[identifier] = materialInstance
        } catch {
            fatalError(error.localizedDescription)
        }
    }

    @objc(updateMaterial:)
    func updateMaterial(_ request: DDBridgeUpdateMaterial) {
        self.dispatchSerialQueue.async {
            self.updateMaterialAsync(request)
        }
    }

    fileprivate func updateMeshAsync(_ data: DDBridgeUpdateMesh) {
        let identifier = data.identifier

        var meshResource: _Proto_LowLevelMeshResource_v1?

        // Create mesh resource outside the lock if needed
        var newMeshResource: _Proto_LowLevelMeshResource_v1?
        if data.updateType == .initial || data.descriptor != nil {
            guard let meshDescriptor = data.descriptor else {
                return
            }
            let descriptor = _Proto_LowLevelMeshResource_v1.Descriptor.fromLlmDescriptor(meshDescriptor)
            if let resource = try? resourceContext.makeMeshResource(descriptor: descriptor) {
                try? resource.setPartCount(data.parts.count)
                resource.replaceData(replaceParts: data.parts, replaceIndexData: data.indexData, replaceVertexData: data.vertexData)
                newMeshResource = resource
            }
        }

        if let resource = newMeshResource {
            meshResources[identifier] = resource
        } else {
            guard let cachedMeshResource = meshResources[identifier] else {
                fatalError("Mesh resource should already be created from previous update")
            }

            if !data.parts.isEmpty || data.indexData != nil || !data.vertexData.isEmpty {
                cachedMeshResource.replaceData(
                    replaceParts: data.parts,
                    replaceIndexData: data.indexData,
                    replaceVertexData: data.vertexData
                )
            }
        }

        meshResource = meshResources[identifier]

        if data.instanceTransforms != nil {
            let capturedMeshResource = meshResource
            // Make new instances
            if meshInstances[identifier] == nil {
                meshInstances[identifier] = []
                var instanceTransformPointer = data.instanceTransforms
                while instanceTransformPointer != nil {
                    var instanceTransform = instanceTransformPointer?.transform ?? simd_float4x4()
                    for partIndex in 0..<data.parts.count {
                        guard let capturedMeshResourceNonNil = capturedMeshResource else {
                            continue
                        }
                        let materialIdentifier = data.materialPrims[partIndex]
                        guard let materialInstance = materialInstances[materialIdentifier] else {
                            fatalError("Failed to get material instance \(materialIdentifier)")
                        }
                        let meshInstance = _Proto_LowLevelMeshInstance_v1.init(
                            resource: capturedMeshResourceNonNil,
                            material: materialInstance,
                            partIndex: partIndex,
                            transform: .single(instanceTransform)
                        )
                        meshInstances[identifier]?.append((meshInstance, instanceTransform))
                    }

                    instanceTransformPointer = instanceTransformPointer?.next
                }
            } else {
                // Update transforms otherwise
                guard var nonNilMeshInstances = meshInstances[identifier] else {
                    return
                }
                let partCount = nonNilMeshInstances.count / data.instanceTransformsCount
                var instanceIndex: Int = 0
                var instanceTransformPointer = data.instanceTransforms
                while instanceTransformPointer != nil {
                    var instanceTransform = instanceTransformPointer?.transform ?? simd_float4x4()
                    for partIndex in 0..<partCount {
                        nonNilMeshInstances[instanceIndex * data.parts.count + partIndex].1 = instanceTransform
                    }

                    instanceTransformPointer = instanceTransformPointer?.next
                }
            }
        }
    }

    @objc(updateMesh:)
    func updateMesh(_ request: DDBridgeUpdateMesh) {
        logError("(update mesh) Material ids \(request.materialPrims)")
        self.dispatchSerialQueue.async {
            self.updateMeshAsync(request)
        }
    }

    fileprivate func setTransformAsync(_ transform: simd_float4x4) {
        modelTransform = transform
    }

    @objc(setTransform:)
    func setTransform(_ transform: simd_float4x4) {
        self.dispatchSerialQueue.async {
            self.setTransformAsync(transform)
        }
    }

    @objc
    func setCameraDistance(_ distance: Float) {
        modelDistance = distance
    }

    @objc
    func setPlaying(_ play: Bool) {
        // resourceContext.setEnableModelRotation(play)
    }
}

final class Converter {
    static func toWebImageAsset(_ asset: LowLevelTexture.Descriptor, data: Data) -> DDBridgeImageAsset {
        DDBridgeImageAsset(
            data: data,
            width: asset.width,
            height: asset.height,
            depth: asset.depth,
            bytesPerPixel: 0, // client calculates this
            textureType: asset.textureType,
            pixelFormat: asset.pixelFormat,
            mipmapLevelCount: asset.mipmapLevelCount,
            arrayLength: asset.arrayLength,
            textureUsage: asset.textureUsage,
            swizzle: asset.swizzle
        )
    }

    static func convertSemantic(_ semantic: LowLevelMesh.VertexSemantic) -> Int {
        switch semantic {
        case .position: return 0
        case .color: return 1
        case .normal: return 2
        case .tangent: return 3
        case .bitangent: return 4
        case .uv0: return 5
        case .uv1: return 6
        case .uv2: return 7
        case .uv3: return 8
        case .uv4: return 9
        case .uv5: return 10
        case .uv6: return 11
        case .uv7: return 12
        default: return 13
        }
    }

    static func webAttributesFromAttributes(_ attributes: [LowLevelMesh.Attribute]) -> [DDBridgeVertexAttributeFormat] {
        attributes.map({ a in
            DDBridgeVertexAttributeFormat(
                semantic: Converter.convertSemantic(a.semantic),
                format: a.format.rawValue,
                layoutIndex: a.layoutIndex,
                offset: a.offset
            )
        })
    }

    static func webLayoutsFromLayouts(_ attributes: [LowLevelMesh.Layout]) -> [DDBridgeVertexLayout] {
        attributes.map({ a in
            DDBridgeVertexLayout(bufferIndex: a.bufferIndex, bufferOffset: a.bufferOffset, bufferStride: a.bufferStride)
        })
    }

    static func webMeshDescriptorFromMeshDescriptor(_ request: LowLevelMesh.Descriptor) -> DDBridgeMeshDescriptor {
        DDBridgeMeshDescriptor(
            vertexBufferCount: request.vertexBufferCount,
            vertexCapacity: request.vertexCapacity,
            vertexAttributes: Converter.webAttributesFromAttributes(request.vertexAttributes),
            vertexLayouts: Converter.webLayoutsFromLayouts(request.vertexLayouts),
            indexCapacity: request.indexCapacity,
            indexType: request.indexType
        )
    }

    static func webPartsFromParts(_ parts: [LowLevelMesh.Part]) -> [DDBridgeMeshPart] {
        parts.map({ a in
            DDBridgeMeshPart(
                indexOffset: a.indexOffset,
                indexCount: a.indexCount,
                topology: a.topology,
                materialIndex: a.materialIndex,
                boundsMin: a.bounds.min,
                boundsMax: a.bounds.max
            )
        })
    }

    static func convert(_ m: _Proto_DataUpdateType_v1) -> DDBridgeDataUpdateType {
        if m == .initial {
            return .initial
        }
        return .delta
    }

    static func webUpdateTextureRequestFromUpdateTextureRequest(_ request: _Proto_TextureDataUpdate_v1) -> DDBridgeUpdateTexture {
        DDBridgeUpdateTexture(
            imageAsset: toWebImageAsset(request.descriptor!, data: request.data!),
            identifier: request.identifier,
            hashString: request.hashString
        )
    }

    static func convert4x4(_ floatArray: [Float]) -> simd_float4x4 {
        simd_float4x4(
            simd_float4(floatArray[0], floatArray[1], floatArray[2], floatArray[3]),
            simd_float4(floatArray[4], floatArray[5], floatArray[6], floatArray[7]),
            simd_float4(floatArray[8], floatArray[9], floatArray[10], floatArray[11]),
            simd_float4(floatArray[12], floatArray[13], floatArray[14], floatArray[15])
        )
    }

    static func webUpdateMeshRequestFromUpdateMeshRequest(
        _ request: _Proto_MeshDataUpdate_v1
    ) -> DDBridgeUpdateMesh {
        var webRequestInstanceTransforms: DDBridgeChainedFloat4x4?

        if request.instanceTransformsCount() > 0 {
            let countMinusOne = request.instanceTransformsCount() - 1
            webRequestInstanceTransforms = DDBridgeChainedFloat4x4(transform: Converter.convert4x4(request.instanceTransform(0)))
            var instanceTransforms = webRequestInstanceTransforms
            if countMinusOne > 0 {
                for i in 1...countMinusOne {
                    instanceTransforms?.next = DDBridgeChainedFloat4x4(transform: Converter.convert4x4(request.instanceTransform(i)))
                    instanceTransforms = instanceTransforms?.next
                }
            }
        }

        var descriptor: DDBridgeMeshDescriptor?
        if let requestDescriptor = request.descriptor {
            descriptor = Converter.webMeshDescriptorFromMeshDescriptor(requestDescriptor)
        }

        return DDBridgeUpdateMesh(
            identifier: request.identifier,
            updateType: Converter.convert(request.updateType),
            descriptor: descriptor,
            parts: Converter.webPartsFromParts(request.parts),
            indexData: request.indexData,
            vertexData: request.vertexData,
            instanceTransforms: webRequestInstanceTransforms,
            instanceTransformsCount: request.instanceTransformsCount(),
            materialPrims: request.materialPrims
        )
    }

    static func webUpdateMaterialRequestFromUpdateMaterialRequest(
        _ request: _Proto_MaterialDataUpdate_v1
    ) -> DDBridgeUpdateMaterial {
        DDBridgeUpdateMaterial(materialGraph: request.materialSourceArchive, identifier: request.identifier)
    }
}

final class USDModelLoader: _Proto_UsdStageSession_v1.Delegate {
    fileprivate let usdLoader: _Proto_UsdStageSession_v1
    private let objcLoader: DDBridgeModelLoader

    @nonobjc
    private let dispatchSerialQueue: DispatchSerialQueue

    @nonobjc
    fileprivate var time: TimeInterval = 0

    @nonobjc
    fileprivate var startTime: TimeInterval = 0
    @nonobjc
    fileprivate var endTime: TimeInterval = 1
    @nonobjc
    fileprivate var timeCodePerSecond: TimeInterval = 1

    init(objcInstance: DDBridgeModelLoader) {
        objcLoader = objcInstance
        usdLoader = .init()
        dispatchSerialQueue = DispatchSerialQueue(label: "USDModelWebProcess", qos: .userInteractive)
        usdLoader.delegate = self
    }

    func meshUpdated(data: consuming sending _Proto_MeshDataUpdate_v1) {
        let identifier = data.identifier
        if data.updateType == .initial || data.descriptor != nil {
            self.dispatchSerialQueue.async {
                self.objcLoader.updateMesh(webRequest: Converter.webUpdateMeshRequestFromUpdateMeshRequest(data))
            }
        }
    }

    func meshDestroyed(identifier: String) {
        //
    }

    func materialUpdated(data: consuming sending _Proto_MaterialDataUpdate_v1) {
        let identifier = data.identifier
        self.dispatchSerialQueue.async {
            self.objcLoader.updateMaterial(webRequest: Converter.webUpdateMaterialRequestFromUpdateMaterialRequest(data))
        }
    }

    func materialDestroyed(identifier: String) {
        //
    }

    func textureUpdated(data: consuming sending _Proto_TextureDataUpdate_v1) {
        // WebDDImageAsset(path: asset.path, utType: asset.utType, data: asset.data, semantic: toWebSemantic(asset.semantic), identifier: id)
        let identifier = data.identifier
        self.dispatchSerialQueue.async {
            self.objcLoader.updateTexture(webRequest: Converter.webUpdateTextureRequestFromUpdateTextureRequest(data))
        }
    }

    func textureDestroyed(identifier: String) {
        //
    }

    func loadModel(from url: Foundation.URL) {
        do {
            let stage = try UsdStage(contentsOf: url)
            self.timeCodePerSecond = stage.timeCodesPerSecond
            self.startTime = stage.startTimeCode
            self.endTime = stage.endTimeCode
            self.usdLoader.loadStage(stage)
        } catch {
            fatalError(error.localizedDescription)
        }
    }

    func loadModel(from data: Data) {
        // usdLoader.loadModel(from: data)
    }

    func update(deltaTime: TimeInterval) {
        usdLoader.update(time: time)

        time = fmod(deltaTime * self.timeCodePerSecond + time - startTime, max(endTime - startTime, 1)) + startTime
    }
}

@objc
@implementation
extension DDBridgeModelLoader {
    @nonobjc
    var loader: USDModelLoader?
    @nonobjc
    var modelUpdated: ((DDBridgeUpdateMesh) -> (Void))?
    @nonobjc
    var textureUpdatedCallback: ((DDBridgeUpdateTexture) -> (Void))?
    @nonobjc
    var materialUpdatedCallback: ((DDBridgeUpdateMaterial) -> (Void))?

    @nonobjc
    fileprivate var retainedRequests: Set<NSObject> = []

    override init() {
        super.init()

        self.loader = USDModelLoader(objcInstance: self)
    }

    @objc(
        setCallbacksWithModelUpdatedCallback:
        textureUpdatedCallback:
        materialUpdatedCallback:
    )
    func setCallbacksWithModelUpdatedCallback(
        _ modelUpdatedCallback: @escaping ((DDBridgeUpdateMesh) -> (Void)),
        textureUpdatedCallback: @escaping ((DDBridgeUpdateTexture) -> (Void)),
        materialUpdatedCallback: @escaping ((DDBridgeUpdateMaterial) -> (Void))
    ) {
        self.modelUpdated = modelUpdatedCallback
        self.textureUpdatedCallback = textureUpdatedCallback
        self.materialUpdatedCallback = materialUpdatedCallback
    }

    @objc
    func loadModel(from url: Foundation.URL) {
        self.loader?.loadModel(from: url)
    }

    @objc
    func update(_ deltaTime: Double) {
        self.loader?.update(deltaTime: deltaTime)
    }

    @objc
    func requestCompleted(_ request: NSObject) {
        retainedRequests.remove(request)
    }

    fileprivate func updateMesh(webRequest: DDBridgeUpdateMesh) {
        if let modelUpdated {
            retainedRequests.insert(webRequest)
            modelUpdated(webRequest)
        }
    }

    fileprivate func updateTexture(webRequest: DDBridgeUpdateTexture) {
        if let textureUpdatedCallback {
            retainedRequests.insert(webRequest)
            textureUpdatedCallback(webRequest)
        }
    }

    fileprivate func updateMaterial(webRequest: DDBridgeUpdateMaterial) {
        if let materialUpdatedCallback {
            retainedRequests.insert(webRequest)
            materialUpdatedCallback(webRequest)
        }
    }
}

#endif
