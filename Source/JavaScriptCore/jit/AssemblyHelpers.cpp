/*
 * Copyright (C) 2011-2023 Apple Inc. All rights reserved.
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
#include "AssemblyHelpers.h"

WTF_ALLOW_UNSAFE_BUFFER_USAGE_BEGIN

#if ENABLE(JIT)

#include "AccessCase.h"
#include "AssemblyHelpersSpoolers.h"
#include "BaselineJITCode.h"
#include "JITOperations.h"
#include "JSArrayBufferView.h"
#include "JSCJSValueInlines.h"
#include "JSDataView.h"
#include "LinkBuffer.h"
#include "MaxFrameExtentForSlowPathCall.h"
#include "MegamorphicCache.h"
#include "SuperSampler.h"
#include "ThunkGenerators.h"
#include "UnlinkedCodeBlock.h"

#if ENABLE(WEBASSEMBLY)
#include "JSWebAssemblyInstance.h"
#include "WasmContext.h"
#include "WasmMemoryInformation.h"
#endif

namespace JSC {

namespace AssemblyHelpersInternal {
constexpr bool dumpVerbose = false;
}

AssemblyHelpers::Jump AssemblyHelpers::branchIfFastTypedArray(GPRReg baseGPR)
{
    return branch8(
        Equal,
        Address(baseGPR, JSArrayBufferView::offsetOfMode()),
        TrustedImm32(FastTypedArray));
}

AssemblyHelpers::Jump AssemblyHelpers::branchIfNotFastTypedArray(GPRReg baseGPR)
{
    return branch8(
        NotEqual,
        Address(baseGPR, JSArrayBufferView::offsetOfMode()),
        TrustedImm32(FastTypedArray));
}

void AssemblyHelpers::incrementSuperSamplerCount()
{
    add32(TrustedImm32(1), AbsoluteAddress(std::bit_cast<const void*>(&g_superSamplerCount)));
}

void AssemblyHelpers::decrementSuperSamplerCount()
{
    sub32(TrustedImm32(1), AbsoluteAddress(std::bit_cast<const void*>(&g_superSamplerCount)));
}

void AssemblyHelpers::purifyNaN(FPRReg inputFPR, FPRReg resultFPR)
{
    ASSERT(inputFPR != fpTempRegister);
#if CPU(ADDRESS64)
    move64ToDouble(TrustedImm64(std::bit_cast<uint64_t>(PNaN)), fpTempRegister);
    moveDoubleConditionallyDouble(DoubleEqualAndOrdered, inputFPR, inputFPR, inputFPR, fpTempRegister, resultFPR);
#else
    moveDouble(inputFPR, resultFPR);
    auto notNaN = branchIfNotNaN(resultFPR);
    move64ToDouble(TrustedImm64(std::bit_cast<uint64_t>(PNaN)), resultFPR);
    notNaN.link(this);
#endif
}

#if ENABLE(SAMPLING_FLAGS)
void AssemblyHelpers::setSamplingFlag(int32_t flag)
{
    ASSERT(flag >= 1);
    ASSERT(flag <= 32);
    or32(TrustedImm32(1u << (flag - 1)), AbsoluteAddress(SamplingFlags::addressOfFlags()));
}

void AssemblyHelpers::clearSamplingFlag(int32_t flag)
{
    ASSERT(flag >= 1);
    ASSERT(flag <= 32);
    and32(TrustedImm32(~(1u << (flag - 1))), AbsoluteAddress(SamplingFlags::addressOfFlags()));
}
#endif

#if ASSERT_ENABLED
#if USE(JSVALUE64)
void AssemblyHelpers::jitAssertIsInt32(GPRReg gpr)
{
    if (!Options::useJITAsserts())
        return;
#if CPU(X86_64) || CPU(ARM64)
    Jump checkInt32 = branch64(BelowOrEqual, gpr, TrustedImm64(static_cast<uintptr_t>(0xFFFFFFFFu)));
    abortWithReason(AHIsNotInt32);
    checkInt32.link(this);
#else
    UNUSED_PARAM(gpr);
#endif
}

void AssemblyHelpers::jitAssertIsJSInt32(GPRReg gpr)
{
    if (!Options::useJITAsserts())
        return;
    Jump checkJSInt32 = branch64(AboveOrEqual, gpr, GPRInfo::numberTagRegister);
    abortWithReason(AHIsNotJSInt32);
    checkJSInt32.link(this);
}

void AssemblyHelpers::jitAssertIsJSNumber(GPRReg gpr)
{
    if (!Options::useJITAsserts())
        return;
    Jump checkJSNumber = branchTest64(MacroAssembler::NonZero, gpr, GPRInfo::numberTagRegister);
    abortWithReason(AHIsNotJSNumber);
    checkJSNumber.link(this);
}

void AssemblyHelpers::jitAssertIsJSDouble(GPRReg gpr)
{
    if (!Options::useJITAsserts())
        return;
    Jump checkJSInt32 = branch64(AboveOrEqual, gpr, GPRInfo::numberTagRegister);
    Jump checkJSNumber = branchTest64(MacroAssembler::NonZero, gpr, GPRInfo::numberTagRegister);
    checkJSInt32.link(this);
    abortWithReason(AHIsNotJSDouble);
    checkJSNumber.link(this);
}

void AssemblyHelpers::jitAssertIsCell(GPRReg gpr)
{
    if (!Options::useJITAsserts())
        return;
    Jump checkCell = branchTest64(MacroAssembler::Zero, gpr, GPRInfo::notCellMaskRegister);
    abortWithReason(AHIsNotCell);
    checkCell.link(this);
}

void AssemblyHelpers::jitAssertTagsInPlace()
{
    if (!Options::useJITAsserts())
        return;
    Jump ok = branch64(Equal, GPRInfo::numberTagRegister, TrustedImm64(JSValue::NumberTag));
    abortWithReason(AHNumberTagNotInPlace);
    breakpoint();
    ok.link(this);

    ok = branch64(Equal, GPRInfo::notCellMaskRegister, TrustedImm64(JSValue::NotCellMask));
    abortWithReason(AHNotCellMaskNotInPlace);
    ok.link(this);
}
#elif USE(JSVALUE32_64)
void AssemblyHelpers::jitAssertIsInt32(GPRReg gpr)
{
    if (!Options::useJITAsserts())
        return;
    UNUSED_PARAM(gpr);
}

void AssemblyHelpers::jitAssertIsJSInt32(GPRReg gpr)
{
    if (!Options::useJITAsserts())
        return;
    Jump checkJSInt32 = branch32(Equal, gpr, TrustedImm32(JSValue::Int32Tag));
    abortWithReason(AHIsNotJSInt32);
    checkJSInt32.link(this);
}

void AssemblyHelpers::jitAssertIsJSNumber(GPRReg gpr)
{
    if (!Options::useJITAsserts())
        return;
    Jump checkJSInt32 = branch32(Equal, gpr, TrustedImm32(JSValue::Int32Tag));
    Jump checkJSDouble = branch32(Below, gpr, TrustedImm32(JSValue::LowestTag));
    abortWithReason(AHIsNotJSNumber);
    checkJSInt32.link(this);
    checkJSDouble.link(this);
}

void AssemblyHelpers::jitAssertIsJSDouble(GPRReg gpr)
{
    if (!Options::useJITAsserts())
        return;
    Jump checkJSDouble = branch32(Below, gpr, TrustedImm32(JSValue::LowestTag));
    abortWithReason(AHIsNotJSDouble);
    checkJSDouble.link(this);
}

void AssemblyHelpers::jitAssertIsCell(GPRReg gpr)
{
    if (!Options::useJITAsserts())
        return;
    Jump checkCell = branchIfCell(gpr);
    abortWithReason(AHIsNotCell);
    checkCell.link(this);
}

void AssemblyHelpers::jitAssertTagsInPlace()
{
    if (!Options::useJITAsserts())
        return;
}
#endif // USE(JSVALUE32_64)

void AssemblyHelpers::jitAssertHasValidCallFrame()
{
    if (!Options::useJITAsserts())
        return;
    Jump checkCFR = branchTestPtr(Zero, GPRInfo::callFrameRegister, TrustedImm32(7));
    abortWithReason(AHCallFrameMisaligned);
    checkCFR.link(this);
}

void AssemblyHelpers::jitAssertIsNull(GPRReg gpr)
{
    if (!Options::useJITAsserts())
        return;
    Jump checkNull = branchTestPtr(Zero, gpr);
    abortWithReason(AHIsNotNull);
    checkNull.link(this);
}

void AssemblyHelpers::jitAssertArgumentCountSane()
{
    if (!Options::useJITAsserts())
        return;
    Jump ok = branch32(Below, payloadFor(CallFrameSlot::argumentCountIncludingThis), TrustedImm32(10000000));
    abortWithReason(AHInsaneArgumentCount);
    ok.link(this);
}

void AssemblyHelpers::jitAssertCodeBlockOnCallFrameWithType(GPRReg scratchGPR, JITType type)
{
    if (!Options::useJITAsserts())
        return;
    JIT_COMMENT(*this, "jitAssertCodeBlockOnCallFrameWithType | ", scratchGPR, " = callFrame->codeBlock->jitCode->jitType == ", type);
    emitGetFromCallFrameHeaderPtr(CallFrameSlot::codeBlock, scratchGPR);
    loadPtr(Address(scratchGPR, CodeBlock::jitCodeOffset()), scratchGPR);
    load8(Address(scratchGPR, JITCode::offsetOfJITType()), scratchGPR);
    Jump ok = branch32(Equal, scratchGPR, TrustedImm32(static_cast<unsigned>(type)));
    abortWithReason(AHInvalidCodeBlock);
    ok.link(this);
}

void AssemblyHelpers::jitAssertCodeBlockMatchesCurrentCalleeCodeBlockOnCallFrame(GPRReg scratchGPR, GPRReg scratchGPR2, UnlinkedCodeBlock& block)
{
    if (!Options::useJITAsserts())
        return;
    if (block.codeType() != FunctionCode)
        return;
    auto kind = block.isConstructor() ? CodeSpecializationKind::CodeForConstruct : CodeSpecializationKind::CodeForCall;
    JIT_COMMENT(*this, "jitAssertCodeBlockMatchesCurrentCalleeCodeBlockOnCallFrame with code block type: ", kind, " | ", scratchGPR, " = callFrame->callee->executableOrRareData");

    emitGetFromCallFrameHeaderPtr(CallFrameSlot::callee, scratchGPR);
    loadPtr(Address(scratchGPR, JSFunction::offsetOfExecutableOrRareData()), scratchGPR);
    auto hasExecutable = branchTestPtr(Zero, scratchGPR, TrustedImm32(JSFunction::rareDataTag));
    loadPtr(Address(scratchGPR, FunctionRareData::offsetOfExecutable() - JSFunction::rareDataTag), scratchGPR);
    hasExecutable.link(this);
    JIT_COMMENT(*this, scratchGPR, " = (", scratchGPR, ": Executable)->codeBlock");
    loadPtr(Address(scratchGPR, FunctionExecutable::offsetOfCodeBlockFor(kind)), scratchGPR);

    JIT_COMMENT(*this, scratchGPR2, " = callFrame->codeBlock");
    emitGetFromCallFrameHeaderPtr(CallFrameSlot::codeBlock, scratchGPR2);
    Jump ok = branch32(Equal, scratchGPR, scratchGPR2);
    abortWithReason(AHInvalidCodeBlock);
    ok.link(this);
}

void AssemblyHelpers::jitAssertCodeBlockOnCallFrameIsOptimizingJIT(GPRReg scratchGPR)
{
    if (!Options::useJITAsserts())
        return;
    emitGetFromCallFrameHeaderPtr(CallFrameSlot::codeBlock, scratchGPR);
    loadPtr(Address(scratchGPR, CodeBlock::jitCodeOffset()), scratchGPR);
    load8(Address(scratchGPR, JITCode::offsetOfJITType()), scratchGPR);
    JumpList ok;
    ok.append(branch32(Equal, scratchGPR, TrustedImm32(static_cast<unsigned>(JITType::DFGJIT))));
    ok.append(branch32(Equal, scratchGPR, TrustedImm32(static_cast<unsigned>(JITType::FTLJIT))));
    abortWithReason(AHInvalidCodeBlock);
    ok.link(this);
}

#endif // ASSERT_ENABLED

void AssemblyHelpers::jitReleaseAssertNoException(VM& vm)
{
    Jump noException;
#if USE(JSVALUE64)
    noException = branchTest64(Zero, AbsoluteAddress(vm.addressOfException()));
#elif USE(JSVALUE32_64)
    noException = branch32(Equal, AbsoluteAddress(vm.addressOfException()), TrustedImm32(0));
#endif
    abortWithReason(JITUncaughtExceptionAfterCall);
    noException.link(this);
}

void AssemblyHelpers::callExceptionFuzz(VM& vm, GPRReg exceptionReg)
{
    RELEASE_ASSERT(Options::useExceptionFuzz());

    EncodedJSValue* buffer = vm.exceptionFuzzingBuffer(sizeof(EncodedJSValue) * (GPRInfo::numberOfRegisters + FPRInfo::numberOfRegisters));

    for (unsigned i = 0; i < GPRInfo::numberOfRegisters; ++i) {
#if USE(JSVALUE64)
        store64(GPRInfo::toRegister(i), buffer + i);
#else
        store32(GPRInfo::toRegister(i), buffer + i);
#endif
    }
    for (unsigned i = 0; i < FPRInfo::numberOfRegisters; ++i) {
        move(TrustedImmPtr(buffer + GPRInfo::numberOfRegisters + i), GPRInfo::regT0);
        storeDouble(FPRInfo::toRegister(i), Address(GPRInfo::regT0));
    }

    // Set up one argument.
    move(TrustedImmPtr(&vm), GPRInfo::argumentGPR0);
    move(TrustedImmPtr(tagCFunction<OperationPtrTag>(operationExceptionFuzzWithCallFrame)), GPRInfo::nonPreservedNonReturnGPR);
    prepareCallOperation(vm);
    call(GPRInfo::nonPreservedNonReturnGPR, OperationPtrTag);

    for (unsigned i = 0; i < FPRInfo::numberOfRegisters; ++i) {
        move(TrustedImmPtr(buffer + GPRInfo::numberOfRegisters + i), GPRInfo::regT0);
        loadDouble(Address(GPRInfo::regT0), FPRInfo::toRegister(i));
    }
    for (unsigned i = 0; i < GPRInfo::numberOfRegisters; ++i) {
#if USE(JSVALUE64)
        load64(buffer + i, GPRInfo::toRegister(i));
#else
        load32(buffer + i, GPRInfo::toRegister(i));
#endif
    }

    if (exceptionReg != InvalidGPRReg)
        loadPtr(vm.addressOfException(), exceptionReg);
}

AssemblyHelpers::Jump AssemblyHelpers::emitJumpIfException(VM& vm)
{
    return emitExceptionCheck(vm, NormalExceptionCheck);
}

AssemblyHelpers::Jump AssemblyHelpers::emitExceptionCheck(VM& vm, ExceptionCheckKind kind, ExceptionJumpWidth width, GPRReg exceptionReg)
{
    if (Options::useExceptionFuzz()) [[unlikely]]
        callExceptionFuzz(vm, exceptionReg);

    if (width == FarJumpWidth)
        kind = (kind == NormalExceptionCheck ? InvertedExceptionCheck : NormalExceptionCheck);

    Jump result;
    if (exceptionReg != InvalidGPRReg) {
#if ASSERT_ENABLED
        JIT_COMMENT(*this, "Exception validation");
        Jump ok = branchPtr(Equal, AbsoluteAddress(vm.addressOfException()), exceptionReg);
        breakpoint();
        ok.link(this);
#endif
        JIT_COMMENT(*this, "Exception check from operation result register");
        result = branchTestPtr(kind == NormalExceptionCheck ? NonZero : Zero, exceptionReg);
    } else {
        JIT_COMMENT(*this, "Exception check from vm");
        result = branchTestPtr(kind == NormalExceptionCheck ? NonZero : Zero, AbsoluteAddress(vm.addressOfException()));
    }

    if (width == NormalJumpWidth)
        return result;

    PatchableJump realJump = patchableJump();
    result.link(this);

    return realJump.m_jump;
}

AssemblyHelpers::Jump AssemblyHelpers::emitNonPatchableExceptionCheck(VM& vm, GPRReg exceptionReg)
{
    return emitExceptionCheck(vm, NormalExceptionCheck, NormalJumpWidth, exceptionReg);
}

void AssemblyHelpers::emitStoreStructureWithTypeInfo(AssemblyHelpers& jit, TrustedImmPtr structure, RegisterID dest)
{
    const Structure* structurePtr = reinterpret_cast<const Structure*>(structure.m_value);
#if USE(JSVALUE64)
    jit.store64(TrustedImm64(static_cast<uint64_t>(structurePtr->id().bits()) | (static_cast<uint64_t>(structurePtr->typeInfoBlob()) << 32)), MacroAssembler::Address(dest, JSCell::structureIDOffset()));
    if (ASSERT_ENABLED) {
        Jump correctStructure = jit.branch32(Equal, MacroAssembler::Address(dest, JSCell::structureIDOffset()), TrustedImm32(structurePtr->id().bits()));
        jit.abortWithReason(AHStructureIDIsValid);
        correctStructure.link(&jit);

        Jump correctIndexingType = jit.branch8(Equal, MacroAssembler::Address(dest, JSCell::indexingTypeAndMiscOffset()), TrustedImm32(structurePtr->indexingModeIncludingHistory()));
        jit.abortWithReason(AHIndexingTypeIsValid);
        correctIndexingType.link(&jit);

        Jump correctType = jit.branch8(Equal, MacroAssembler::Address(dest, JSCell::typeInfoTypeOffset()), TrustedImm32(structurePtr->typeInfo().type()));
        jit.abortWithReason(AHTypeInfoIsValid);
        correctType.link(&jit);

        Jump correctFlags = jit.branch8(Equal, MacroAssembler::Address(dest, JSCell::typeInfoFlagsOffset()), TrustedImm32(structurePtr->typeInfo().inlineTypeFlags()));
        jit.abortWithReason(AHTypeInfoInlineTypeFlagsAreValid);
        correctFlags.link(&jit);
    }
#else
    // Do a 32-bit wide store to initialize the cell's fields.
    jit.store32(TrustedImm32(structurePtr->typeInfoBlob()), MacroAssembler::Address(dest, JSCell::indexingTypeAndMiscOffset()));
    jit.storePtr(structure, MacroAssembler::Address(dest, JSCell::structureIDOffset()));
#endif
}

void AssemblyHelpers::loadProperty(GPRReg object, GPRReg offset, JSValueRegs result)
{
    ASSERT(noOverlap(offset, result));
    Jump isInline = branch32(LessThan, offset, TrustedImm32(firstOutOfLineOffset));

    loadPtr(Address(object, JSObject::butterflyOffset()), result.payloadGPR());
    neg32(offset);
    signExtend32ToPtr(offset, offset);
    Jump ready = jump();

    isInline.link(this);
    addPtr(
        TrustedImm32(
            static_cast<int32_t>(sizeof(JSObject)) -
            (static_cast<int32_t>(firstOutOfLineOffset) - 2) * static_cast<int32_t>(sizeof(EncodedJSValue))),
        object, result.payloadGPR());

    ready.link(this);

    loadValue(
        BaseIndex(
            result.payloadGPR(), offset, TimesEight, (firstOutOfLineOffset - 2) * sizeof(EncodedJSValue)),
        result);
}

void AssemblyHelpers::storeProperty(JSValueRegs value, GPRReg object, GPRReg offset, GPRReg scratch)
{
    // Actually, object can be the same to scratch.
    ASSERT(noOverlap(offset, scratch));
    ASSERT(noOverlap(value, scratch));
    Jump isInline = branch32(LessThan, offset, TrustedImm32(firstOutOfLineOffset));

    loadPtr(Address(object, JSObject::butterflyOffset()), scratch);
    neg32(offset);
    signExtend32ToPtr(offset, offset);
    Jump ready = jump();

    isInline.link(this);
    addPtr(
        TrustedImm32(
            static_cast<int32_t>(sizeof(JSObject)) -
            (static_cast<int32_t>(firstOutOfLineOffset) - 2) * static_cast<int32_t>(sizeof(EncodedJSValue))),
        object, scratch);

    ready.link(this);

    storeValue(value, BaseIndex(scratch, offset, TimesEight, (firstOutOfLineOffset - 2) * sizeof(EncodedJSValue)));
}

#if USE(JSVALUE64)
AssemblyHelpers::JumpList AssemblyHelpers::loadMegamorphicProperty(VM& vm, GPRReg baseGPR, GPRReg uidGPR, UniquedStringImpl* uid, GPRReg resultGPR, GPRReg scratch1GPR, GPRReg scratch2GPR, GPRReg scratch3GPR)
{
    // uidGPR can be InvalidGPRReg if uid is non-nullptr.

    if (!uid)
        ASSERT(uidGPR != InvalidGPRReg);

    JumpList primaryFail;
    JumpList slowCases;

    load32(Address(baseGPR, JSCell::structureIDOffset()), scratch1GPR);
#if CPU(ARM64)
    extractUnsignedBitfield32(scratch1GPR, TrustedImm32(MegamorphicCache::structureIDHashShift1), TrustedImm32(32 - MegamorphicCache::structureIDHashShift1), scratch2GPR);
    xorUnsignedRightShift32(scratch2GPR, scratch1GPR, TrustedImm32(MegamorphicCache::structureIDHashShift2), scratch3GPR);
#else
    urshift32(scratch1GPR, TrustedImm32(MegamorphicCache::structureIDHashShift1), scratch2GPR);
    urshift32(scratch1GPR, TrustedImm32(MegamorphicCache::structureIDHashShift2), scratch3GPR);
    xor32(scratch2GPR, scratch3GPR);
#endif
    if (uid)
        add32(TrustedImm32(uid->hash()), scratch3GPR);
    else {
        // Note that we don't test if the hash is zero here. AtomStringImpl's can't have a zero
        // hash, however, a SymbolImpl may. But, because this is a cache, we don't care. We only
        // ever load the result from the cache if the cache entry matches what we are querying for.
        // So we either get super lucky and use zero for the hash and somehow collide with the entity
        // we're looking for, or we realize we're comparing against another entity, and go to the
        // slow path anyways.
        load32(Address(uidGPR, UniquedStringImpl::flagsOffset()), scratch2GPR);
        urshift32(TrustedImm32(StringImpl::s_flagCount), scratch2GPR);
        add32(scratch2GPR, scratch3GPR);
    }

    and32(TrustedImm32(MegamorphicCache::loadCachePrimaryMask), scratch3GPR);
    if (hasOneBitSet(sizeof(MegamorphicCache::LoadEntry))) // is a power of 2
        lshift32(TrustedImm32(getLSBSet(sizeof(MegamorphicCache::LoadEntry))), scratch3GPR);
    else
        mul32(TrustedImm32(sizeof(MegamorphicCache::LoadEntry)), scratch3GPR, scratch3GPR);
    auto& cache = vm.ensureMegamorphicCache();
    move(TrustedImmPtr(&cache), scratch2GPR);
    static_assert(!MegamorphicCache::offsetOfLoadCachePrimaryEntries());
    addPtr(scratch2GPR, scratch3GPR);

    load16(Address(scratch2GPR, MegamorphicCache::offsetOfEpoch()), scratch2GPR);

    primaryFail.append(branch32(NotEqual, scratch1GPR, Address(scratch3GPR, MegamorphicCache::LoadEntry::offsetOfStructureID())));
    if (uid)
        primaryFail.append(branchPtr(NotEqual, Address(scratch3GPR, MegamorphicCache::LoadEntry::offsetOfUid()), TrustedImmPtr(uid)));
    else
        primaryFail.append(branchPtr(NotEqual, Address(scratch3GPR, MegamorphicCache::LoadEntry::offsetOfUid()), uidGPR));
    // We already hit StructureID and uid. And we get stale epoch for this entry.
    // Since all entries in the secondary cache has stale epoch for this StructureID and uid pair, we should just go to the slow case.
    slowCases.append(branch32WithMemory16(NotEqual, Address(scratch3GPR, MegamorphicCache::LoadEntry::offsetOfEpoch()), scratch2GPR));

    // Cache hit!
    Label cacheHit = label();
    loadPtr(Address(scratch3GPR, MegamorphicCache::LoadEntry::offsetOfHolder()), scratch2GPR);
    auto missed = branchTestPtr(Zero, scratch2GPR);
    moveConditionally64(Equal, scratch2GPR, TrustedImm32(std::bit_cast<uintptr_t>(JSCell::seenMultipleCalleeObjects())), baseGPR, scratch2GPR, scratch1GPR);
    load16(Address(scratch3GPR, MegamorphicCache::LoadEntry::offsetOfOffset()), scratch2GPR);
    loadProperty(scratch1GPR, scratch2GPR, JSValueRegs { resultGPR });
    auto done = jump();

    // Secondary cache lookup. Now,
    //   1. scratch1GPR holds StructureID.
    //   2. scratch2GPR holds global epoch.
    primaryFail.link(this);
    if (uid)
        add32(TrustedImm32(static_cast<uint32_t>(std::bit_cast<uintptr_t>(uid))), scratch1GPR, scratch3GPR);
    else
        add32(uidGPR, scratch1GPR, scratch3GPR);
    addUnsignedRightShift32(scratch3GPR, scratch3GPR, TrustedImm32(MegamorphicCache::structureIDHashShift3), scratch3GPR);
    and32(TrustedImm32(MegamorphicCache::loadCacheSecondaryMask), scratch3GPR);
    if constexpr (hasOneBitSet(sizeof(MegamorphicCache::LoadEntry))) // is a power of 2
        lshift32(TrustedImm32(getLSBSet(sizeof(MegamorphicCache::LoadEntry))), scratch3GPR);
    else
        mul32(TrustedImm32(sizeof(MegamorphicCache::LoadEntry)), scratch3GPR, scratch3GPR);
    addPtr(TrustedImmPtr(std::bit_cast<uint8_t*>(&cache) + MegamorphicCache::offsetOfLoadCacheSecondaryEntries()), scratch3GPR);

    slowCases.append(branch32(NotEqual, scratch1GPR, Address(scratch3GPR, MegamorphicCache::LoadEntry::offsetOfStructureID())));
    if (uid)
        slowCases.append(branchPtr(NotEqual, Address(scratch3GPR, MegamorphicCache::LoadEntry::offsetOfUid()), TrustedImmPtr(uid)));
    else
        slowCases.append(branchPtr(NotEqual, Address(scratch3GPR, MegamorphicCache::LoadEntry::offsetOfUid()), uidGPR));
    slowCases.append(branch32WithMemory16(NotEqual, Address(scratch3GPR, MegamorphicCache::LoadEntry::offsetOfEpoch()), scratch2GPR));
    jump().linkTo(cacheHit, this);

    missed.link(this);
    moveTrustedValue(jsUndefined(), JSValueRegs { resultGPR });

    done.link(this);

    return slowCases;
}

std::tuple<AssemblyHelpers::JumpList, AssemblyHelpers::JumpList> AssemblyHelpers::storeMegamorphicProperty(VM& vm, GPRReg baseGPR, GPRReg uidGPR, UniquedStringImpl* uid, GPRReg valueGPR, GPRReg scratch1GPR, GPRReg scratch2GPR, GPRReg scratch3GPR)
{
    // uidGPR can be InvalidGPRReg if uid is non-nullptr.

    if (!uid)
        ASSERT(uidGPR != InvalidGPRReg);

    JumpList primaryFail;
    JumpList slowCases;
    JumpList reallocatingCases;

    load32(Address(baseGPR, JSCell::structureIDOffset()), scratch1GPR);
#if CPU(ARM64)
    extractUnsignedBitfield32(scratch1GPR, TrustedImm32(MegamorphicCache::structureIDHashShift1), TrustedImm32(32 - MegamorphicCache::structureIDHashShift1), scratch2GPR);
    xorUnsignedRightShift32(scratch2GPR, scratch1GPR, TrustedImm32(MegamorphicCache::structureIDHashShift4), scratch3GPR);
#else
    urshift32(scratch1GPR, TrustedImm32(MegamorphicCache::structureIDHashShift1), scratch2GPR);
    urshift32(scratch1GPR, TrustedImm32(MegamorphicCache::structureIDHashShift4), scratch3GPR);
    xor32(scratch2GPR, scratch3GPR);
#endif

    if (uid)
        add32(TrustedImm32(uid->hash()), scratch3GPR);
    else {
        // Note that we don't test if the hash is zero here. AtomStringImpl's can't have a zero
        // hash, however, a SymbolImpl may. But, because this is a cache, we don't care. We only
        // ever load the result from the cache if the cache entry matches what we are querying for.
        // So we either get super lucky and use zero for the hash and somehow collide with the entity
        // we're looking for, or we realize we're comparing against another entity, and go to the
        // slow path anyways.
        load32(Address(uidGPR, UniquedStringImpl::flagsOffset()), scratch2GPR);
        urshift32(TrustedImm32(StringImpl::s_flagCount), scratch2GPR);
        add32(scratch2GPR, scratch3GPR);
    }

    and32(TrustedImm32(MegamorphicCache::storeCachePrimaryMask), scratch3GPR);
    if (hasOneBitSet(sizeof(MegamorphicCache::StoreEntry))) // is a power of 2
        lshift32(TrustedImm32(getLSBSet(sizeof(MegamorphicCache::StoreEntry))), scratch3GPR);
    else
        mul32(TrustedImm32(sizeof(MegamorphicCache::StoreEntry)), scratch3GPR, scratch3GPR);
    auto& cache = vm.ensureMegamorphicCache();
    move(TrustedImmPtr(&cache), scratch2GPR);
    addPtr(scratch2GPR, scratch3GPR);
    addPtr(TrustedImmPtr(MegamorphicCache::offsetOfStoreCachePrimaryEntries()), scratch3GPR);

    load16(Address(scratch2GPR, MegamorphicCache::offsetOfEpoch()), scratch2GPR);

    primaryFail.append(branch32(NotEqual, scratch1GPR, Address(scratch3GPR, MegamorphicCache::StoreEntry::offsetOfOldStructureID())));
    if (uid)
        primaryFail.append(branchPtr(NotEqual, Address(scratch3GPR, MegamorphicCache::StoreEntry::offsetOfUid()), TrustedImmPtr(uid)));
    else
        primaryFail.append(branchPtr(NotEqual, Address(scratch3GPR, MegamorphicCache::StoreEntry::offsetOfUid()), uidGPR));
    // We already hit StructureID and uid. And we get stale epoch for this entry.
    // Since all entries in the secondary cache has stale epoch for this StructureID and uid pair, we should just go to the slow case.
    slowCases.append(branch32WithMemory16(NotEqual, Address(scratch3GPR, MegamorphicCache::StoreEntry::offsetOfEpoch()), scratch2GPR));

    // Cache hit!
    Label cacheHit = label();
    reallocatingCases.append(branchTest8(NonZero, Address(scratch3GPR, MegamorphicCache::StoreEntry::offsetOfReallocating())));
    load32(Address(scratch3GPR, MegamorphicCache::StoreEntry::offsetOfNewStructureID()), scratch2GPR);
    load16(Address(scratch3GPR, MegamorphicCache::StoreEntry::offsetOfOffset()), scratch3GPR);
    auto replaceCase = branch32(Equal, scratch2GPR, scratch1GPR);

    // We only support non-allocating transition. This means we do not need to nuke Structure* for transition here.
    store32(scratch2GPR, Address(baseGPR, JSCell::structureIDOffset()));

    replaceCase.link(this);
    storeProperty(JSValueRegs { valueGPR }, baseGPR, scratch3GPR, scratch1GPR);
    auto done = jump();

    // Secondary cache lookup
    primaryFail.link(this);
    if (uid)
        add32(TrustedImm32(static_cast<uint32_t>(std::bit_cast<uintptr_t>(uid))), scratch1GPR, scratch3GPR);
    else
        add32(uidGPR, scratch1GPR, scratch3GPR);
    addUnsignedRightShift32(scratch3GPR, scratch3GPR, TrustedImm32(MegamorphicCache::structureIDHashShift5), scratch3GPR);
    and32(TrustedImm32(MegamorphicCache::storeCacheSecondaryMask), scratch3GPR);
    if constexpr (hasOneBitSet(sizeof(MegamorphicCache::StoreEntry))) // is a power of 2
        lshift32(TrustedImm32(getLSBSet(sizeof(MegamorphicCache::StoreEntry))), scratch3GPR);
    else
        mul32(TrustedImm32(sizeof(MegamorphicCache::StoreEntry)), scratch3GPR, scratch3GPR);
    addPtr(TrustedImmPtr(std::bit_cast<uint8_t*>(&cache) + MegamorphicCache::offsetOfStoreCacheSecondaryEntries()), scratch3GPR);

    slowCases.append(branch32(NotEqual, scratch1GPR, Address(scratch3GPR, MegamorphicCache::StoreEntry::offsetOfOldStructureID())));
    if (uid)
        slowCases.append(branchPtr(NotEqual, Address(scratch3GPR, MegamorphicCache::StoreEntry::offsetOfUid()), TrustedImmPtr(uid)));
    else
        slowCases.append(branchPtr(NotEqual, Address(scratch3GPR, MegamorphicCache::StoreEntry::offsetOfUid()), uidGPR));
    slowCases.append(branch32WithMemory16(NotEqual, Address(scratch3GPR, MegamorphicCache::StoreEntry::offsetOfEpoch()), scratch2GPR));
    jump().linkTo(cacheHit, this);

    done.link(this);

    return std::tuple { slowCases, reallocatingCases };
}

AssemblyHelpers::JumpList AssemblyHelpers::hasMegamorphicProperty(VM& vm, GPRReg baseGPR, GPRReg uidGPR, UniquedStringImpl* uid, GPRReg resultGPR, GPRReg scratch1GPR, GPRReg scratch2GPR, GPRReg scratch3GPR)
{
    // uidGPR can be InvalidGPRReg if uid is non-nullptr.

    if (!uid)
        ASSERT(uidGPR != InvalidGPRReg);

    JumpList primaryFail;
    JumpList slowCases;

    load32(Address(baseGPR, JSCell::structureIDOffset()), scratch1GPR);
#if CPU(ARM64)
    extractUnsignedBitfield32(scratch1GPR, TrustedImm32(MegamorphicCache::structureIDHashShift1), TrustedImm32(32 - MegamorphicCache::structureIDHashShift1), scratch2GPR);
    xorUnsignedRightShift32(scratch2GPR, scratch1GPR, TrustedImm32(MegamorphicCache::structureIDHashShift6), scratch3GPR);
#else
    urshift32(scratch1GPR, TrustedImm32(MegamorphicCache::structureIDHashShift1), scratch2GPR);
    urshift32(scratch1GPR, TrustedImm32(MegamorphicCache::structureIDHashShift6), scratch3GPR);
    xor32(scratch2GPR, scratch3GPR);
#endif
    if (uid)
        add32(TrustedImm32(uid->hash()), scratch3GPR);
    else {
        // Note that we don't test if the hash is zero here. AtomStringImpl's can't have a zero
        // hash, however, a SymbolImpl may. But, because this is a cache, we don't care. We only
        // ever load the result from the cache if the cache entry matches what we are querying for.
        // So we either get super lucky and use zero for the hash and somehow collide with the entity
        // we're looking for, or we realize we're comparing against another entity, and go to the
        // slow path anyways.
        load32(Address(uidGPR, UniquedStringImpl::flagsOffset()), scratch2GPR);
        urshift32(TrustedImm32(StringImpl::s_flagCount), scratch2GPR);
        add32(scratch2GPR, scratch3GPR);
    }

    and32(TrustedImm32(MegamorphicCache::hasCachePrimaryMask), scratch3GPR);
    if (hasOneBitSet(sizeof(MegamorphicCache::HasEntry))) // is a power of 2
        lshift32(TrustedImm32(getLSBSet(sizeof(MegamorphicCache::HasEntry))), scratch3GPR);
    else
        mul32(TrustedImm32(sizeof(MegamorphicCache::HasEntry)), scratch3GPR, scratch3GPR);
    auto& cache = vm.ensureMegamorphicCache();
    move(TrustedImmPtr(&cache), scratch2GPR);
    addPtr(scratch2GPR, scratch3GPR);
    addPtr(TrustedImmPtr(MegamorphicCache::offsetOfHasCachePrimaryEntries()), scratch3GPR);

    load16(Address(scratch2GPR, MegamorphicCache::offsetOfEpoch()), scratch2GPR);

    primaryFail.append(branch32(NotEqual, scratch1GPR, Address(scratch3GPR, MegamorphicCache::HasEntry::offsetOfStructureID())));
    if (uid)
        primaryFail.append(branchPtr(NotEqual, Address(scratch3GPR, MegamorphicCache::HasEntry::offsetOfUid()), TrustedImmPtr(uid)));
    else
        primaryFail.append(branchPtr(NotEqual, Address(scratch3GPR, MegamorphicCache::HasEntry::offsetOfUid()), uidGPR));
    // We already hit StructureID and uid. And we get stale epoch for this entry.
    // Since all entries in the secondary cache has stale epoch for this StructureID and uid pair, we should just go to the slow case.
    slowCases.append(branch32WithMemory16(NotEqual, Address(scratch3GPR, MegamorphicCache::HasEntry::offsetOfEpoch()), scratch2GPR));

    // Cache hit!
    Label cacheHit = label();
    load16(Address(scratch3GPR, MegamorphicCache::HasEntry::offsetOfResult()), scratch2GPR);
    boxBoolean(scratch2GPR, JSValueRegs { resultGPR });
    auto done = jump();

    // Secondary cache lookup. Now,
    //   1. scratch1GPR holds StructureID.
    //   2. scratch2GPR holds global epoch.
    primaryFail.link(this);
    if (uid)
        add32(TrustedImm32(static_cast<uint32_t>(std::bit_cast<uintptr_t>(uid))), scratch1GPR, scratch3GPR);
    else
        add32(uidGPR, scratch1GPR, scratch3GPR);
    addUnsignedRightShift32(scratch3GPR, scratch3GPR, TrustedImm32(MegamorphicCache::structureIDHashShift7), scratch3GPR);
    and32(TrustedImm32(MegamorphicCache::hasCacheSecondaryMask), scratch3GPR);
    if constexpr (hasOneBitSet(sizeof(MegamorphicCache::HasEntry))) // is a power of 2
        lshift32(TrustedImm32(getLSBSet(sizeof(MegamorphicCache::HasEntry))), scratch3GPR);
    else
        mul32(TrustedImm32(sizeof(MegamorphicCache::HasEntry)), scratch3GPR, scratch3GPR);
    addPtr(TrustedImmPtr(std::bit_cast<uint8_t*>(&cache) + MegamorphicCache::offsetOfHasCacheSecondaryEntries()), scratch3GPR);

    slowCases.append(branch32(NotEqual, scratch1GPR, Address(scratch3GPR, MegamorphicCache::HasEntry::offsetOfStructureID())));
    if (uid)
        slowCases.append(branchPtr(NotEqual, Address(scratch3GPR, MegamorphicCache::HasEntry::offsetOfUid()), TrustedImmPtr(uid)));
    else
        slowCases.append(branchPtr(NotEqual, Address(scratch3GPR, MegamorphicCache::HasEntry::offsetOfUid()), uidGPR));
    slowCases.append(branch32WithMemory16(NotEqual, Address(scratch3GPR, MegamorphicCache::HasEntry::offsetOfEpoch()), scratch2GPR));
    jump().linkTo(cacheHit, this);

    done.link(this);

    return slowCases;
}
#endif

void AssemblyHelpers::emitNonNullDecodeZeroExtendedStructureID(RegisterID source, RegisterID dest)
{
#if ENABLE(STRUCTURE_ID_WITH_SHIFT)
    lshift64(source, TrustedImm32(StructureID::encodeShiftAmount), dest);
#elif CPU(ADDRESS64)
    // This could use BFI on arm64 but that only helps if the start of structure heap is encodable as a mov and not as an immediate in the add so it's probably not super important.
    if constexpr (structureHeapAddressSize >= 4 * GB) {
        ASSERT(structureHeapAddressSize == 4 * GB);
        move(source, dest);
    } else {
        static_assert(static_cast<uint32_t>(StructureID::structureIDMask) == StructureID::structureIDMask);
        and32(TrustedImm32(static_cast<uint32_t>(StructureID::structureIDMask)), source, dest);
    }
    or64(TrustedImm64(startOfStructureHeap()), dest);
#else // not CPU(ADDRESS64)
    move(source, dest);
#endif
}

void AssemblyHelpers::emitLoadStructure(RegisterID source, RegisterID dest)
{
    load32(MacroAssembler::Address(source, JSCell::structureIDOffset()), dest);
    emitNonNullDecodeZeroExtendedStructureID(dest, dest);
}

void AssemblyHelpers::emitLoadStructure(VM&, RegisterID source, RegisterID dest)
{
    emitLoadStructure(source, dest);
}

void AssemblyHelpers::emitEncodeStructureID(RegisterID source, RegisterID dest)
{
#if ENABLE(STRUCTURE_ID_WITH_SHIFT)
    urshift64(source, TrustedImm32(StructureID::encodeShiftAmount), dest);
#elif CPU(ADDRESS64)
    static_assert(StructureID::structureIDMask <= UINT32_MAX);
    // We don't guarantee the upper bits are cleared, since generally only
    // the bottom 32 bits of the register are observed as the structure ID.
    // So, we don't want to bother masking the register unless it's
    // observable within those 32 bits.
    if (StructureID::structureIDMask < UINT32_MAX)
        and64(TrustedImm32(static_cast<uint32_t>(StructureID::structureIDMask)), source, dest);
    else
        move(source, dest);
#else
    move(source, dest);
#endif
}

void AssemblyHelpers::emitLoadPrototype(VM& vm, GPRReg objectGPR, JSValueRegs resultRegs, JumpList& slowPath)
{
    ASSERT(resultRegs.payloadGPR() != objectGPR);

    slowPath.append(branchTest8(MacroAssembler::NonZero, MacroAssembler::Address(objectGPR, JSObject::typeInfoFlagsOffset()), TrustedImm32(OverridesGetPrototype)));

    emitLoadStructure(vm, objectGPR, resultRegs.payloadGPR());
    loadValue(MacroAssembler::Address(resultRegs.payloadGPR(), Structure::prototypeOffset()), resultRegs);
    auto hasMonoProto = branchIfNotEmpty(resultRegs);
    loadValue(MacroAssembler::Address(objectGPR, offsetRelativeToBase(knownPolyProtoOffset)), resultRegs);
    hasMonoProto.link(this);
}

void AssemblyHelpers::makeSpaceOnStackForCCall()
{
    unsigned stackOffset = WTF::roundUpToMultipleOf<stackAlignmentBytes()>(maxFrameExtentForSlowPathCall);
    if (stackOffset)
        subPtr(TrustedImm32(stackOffset), stackPointerRegister);
}

void AssemblyHelpers::reclaimSpaceOnStackForCCall()
{
    unsigned stackOffset = WTF::roundUpToMultipleOf<stackAlignmentBytes()>(maxFrameExtentForSlowPathCall);
    if (stackOffset)
        addPtr(TrustedImm32(stackOffset), stackPointerRegister);
}

#if USE(JSVALUE64)
template<typename LoadFromHigh, typename StoreToHigh, typename LoadFromLow, typename StoreToLow>
void emitRandomThunkImpl(AssemblyHelpers& jit, GPRReg scratch0, GPRReg scratch1, GPRReg scratch2, FPRReg result, const LoadFromHigh& loadFromHigh, const StoreToHigh& storeToHigh, const LoadFromLow& loadFromLow, const StoreToLow& storeToLow)
{
    // Inlined WeakRandom::advance().
    // uint64_t x = m_low;
    loadFromLow(scratch0);
    // uint64_t y = m_high;
    loadFromHigh(scratch1);
    // m_low = y;
    storeToLow(scratch1);

    // x ^= x << 23;
    jit.lshift64(scratch0, AssemblyHelpers::TrustedImm32(23), scratch2);
    jit.xor64(scratch2, scratch0);

    // x ^= x >> 17;
    jit.rshift64(scratch0, AssemblyHelpers::TrustedImm32(17), scratch2);
    jit.xor64(scratch2, scratch0);

    // x ^= y ^ (y >> 26);
    jit.rshift64(scratch1, AssemblyHelpers::TrustedImm32(26), scratch2);
    jit.xor64(scratch1, scratch2);
    jit.xor64(scratch2, scratch0);

    // m_high = x;
    storeToHigh(scratch0);

    // return x + y;
    jit.add64(scratch1, scratch0);

    // Extract random 53bit. [0, 53] bit is safe integer number ranges in double representation.
    jit.and64(AssemblyHelpers::TrustedImm64((1ULL << 53) - 1), scratch0);
    // Now, scratch0 is always in range of int64_t. Safe to convert it to double with cvtsi2sdq.
    jit.convertInt64ToDouble(scratch0, result);

    // Convert `(53bit double integer value) / (1 << 53)` to `(53bit double integer value) * (1.0 / (1 << 53))`.
    // In latter case, `1.0 / (1 << 53)` will become a double value represented as (mantissa = 0 & exp = 970, it means 1e-(2**54)).
    static constexpr double scale = 1.0 / (1ULL << 53);

    // Multiplying 1e-(2**54) with the double integer does not change anything of the mantissa part of the double integer.
    // It just reduces the exp part of the given 53bit double integer.
    // (Except for 0.0. This is specially handled and in this case, exp just becomes 0.)
    // Now we get 53bit precision random double value in [0, 1).
    jit.move(AssemblyHelpers::TrustedImmPtr(&scale), scratch1);
    jit.mulDouble(AssemblyHelpers::Address(scratch1), result);
}

void AssemblyHelpers::emitRandomThunk(JSGlobalObject* globalObject, GPRReg scratch0, GPRReg scratch1, GPRReg scratch2, FPRReg result)
{
    void* lowAddress = reinterpret_cast<uint8_t*>(globalObject) + JSGlobalObject::weakRandomOffset() + WeakRandom::lowOffset();
    void* highAddress = reinterpret_cast<uint8_t*>(globalObject) + JSGlobalObject::weakRandomOffset() + WeakRandom::highOffset();

    auto loadFromHigh = [&](GPRReg high) {
        load64(highAddress, high);
    };
    auto storeToHigh = [&](GPRReg high) {
        store64(high, highAddress);
    };
    auto loadFromLow = [&](GPRReg low) {
        load64(lowAddress, low);
    };
    auto storeToLow = [&](GPRReg low) {
        store64(low, lowAddress);
    };

    emitRandomThunkImpl(*this, scratch0, scratch1, scratch2, result, loadFromHigh, storeToHigh, loadFromLow, storeToLow);
}

void AssemblyHelpers::emitRandomThunk(VM& vm, GPRReg scratch0, GPRReg scratch1, GPRReg scratch2, GPRReg scratch3, FPRReg result)
{
    emitGetFromCallFrameHeaderPtr(CallFrameSlot::callee, scratch3);
    emitLoadStructure(vm, scratch3, scratch3);
    loadPtr(Address(scratch3, Structure::globalObjectOffset()), scratch3);
    // Now, scratch3 holds JSGlobalObject*.

    auto loadFromHigh = [&](GPRReg high) {
        load64(Address(scratch3, JSGlobalObject::weakRandomOffset() + WeakRandom::highOffset()), high);
    };
    auto storeToHigh = [&](GPRReg high) {
        store64(high, Address(scratch3, JSGlobalObject::weakRandomOffset() + WeakRandom::highOffset()));
    };
    auto loadFromLow = [&](GPRReg low) {
        load64(Address(scratch3, JSGlobalObject::weakRandomOffset() + WeakRandom::lowOffset()), low);
    };
    auto storeToLow = [&](GPRReg low) {
        store64(low, Address(scratch3, JSGlobalObject::weakRandomOffset() + WeakRandom::lowOffset()));
    };

    emitRandomThunkImpl(*this, scratch0, scratch1, scratch2, result, loadFromHigh, storeToHigh, loadFromLow, storeToLow);
}
#endif

void AssemblyHelpers::emitAllocateWithNonNullAllocator(GPRReg resultGPR, const JITAllocator& allocator, GPRReg allocatorGPR, GPRReg scratchGPR, JumpList& slowPath, SlowAllocationResult slowAllocationResult)
{
    if (Options::forceGCSlowPaths()) {
        slowPath.append(jump());
        return;
    }

    // NOTE, some invariants of this function:
    // - When going to the slow path, we must leave resultGPR with zero in it.
    // - We *can not* use RegisterSetBuilder::macroScratchRegisters on x86.
    // - We *can* use RegisterSetBuilder::macroScratchRegisters on ARM.

    Jump popPath;
    Jump zeroPath;
    Jump done;

    if (allocator.isConstant())
        move(TrustedImmPtr(allocator.allocator().localAllocator()), allocatorGPR);

#if CPU(ARM) || CPU(ARM64)
    auto dataTempRegister = getCachedDataTempRegisterIDAndInvalidate();
#endif

#if CPU(ARM64)
    // On ARM64, we can leverage instructions like load-pair and shifted-add to make loading from the free list
    // and extracting interval information use less instructions.

    // Assert that we can use loadPairPtr for the interval bounds and nextInterval/secret.
    static_assert(FreeList::offsetOfIntervalEnd() - FreeList::offsetOfIntervalStart() == sizeof(uintptr_t));
    static_assert(FreeList::offsetOfNextInterval() - FreeList::offsetOfIntervalEnd() == sizeof(uintptr_t));
    static_assert(FreeList::offsetOfSecret() - FreeList::offsetOfNextInterval() == sizeof(uintptr_t));

    JIT_COMMENT(*this, "Bump allocation (fast path)");
    loadPairPtr(allocatorGPR, TrustedImm32(LocalAllocator::offsetOfFreeList() + FreeList::offsetOfIntervalStart()), resultGPR, scratchGPR);
    popPath = branchPtr(RelationalCondition::AboveOrEqual, resultGPR, scratchGPR);
    auto bumpLabel = label();
    if (allocator.isConstant())
        addPtr(TrustedImm32(allocator.allocator().cellSize()), resultGPR, scratchGPR);
    else {
        load32(Address(allocatorGPR, LocalAllocator::offsetOfCellSize()), scratchGPR);
        addPtr(resultGPR, scratchGPR);
    }
    storePtr(scratchGPR, Address(allocatorGPR, LocalAllocator::offsetOfFreeList() + FreeList::offsetOfIntervalStart()));
    done = jump();

    JIT_COMMENT(*this, "Get next interval (slower path)");
    popPath.link(this);
    loadPairPtr(allocatorGPR, TrustedImm32(LocalAllocator::offsetOfFreeList() + FreeList::offsetOfNextInterval()), resultGPR, scratchGPR);
    zeroPath = branchTestPtr(ResultCondition::NonZero, resultGPR, TrustedImm32(1));
    xor64(Address(resultGPR, FreeCell::offsetOfScrambledBits()), scratchGPR);
    addSignExtend64(resultGPR, scratchGPR, dataTempRegister);
    addUnsignedRightShift64(resultGPR, scratchGPR, TrustedImm32(32), scratchGPR);
    storePairPtr(scratchGPR, dataTempRegister, allocatorGPR, TrustedImm32(LocalAllocator::offsetOfFreeList() + FreeList::offsetOfIntervalEnd()));
    jump(bumpLabel);
#elif CPU(X86_64)
    // On x86_64, we can leverage better support for memory operands to directly interact with the free 
    // list instead of relying on registers as much.

    JIT_COMMENT(*this, "Bump allocation (fast path)");
    loadPtr(Address(allocatorGPR, LocalAllocator::offsetOfFreeList() + FreeList::offsetOfIntervalStart()), resultGPR);
    popPath = branchPtr(RelationalCondition::AboveOrEqual, resultGPR, Address(allocatorGPR, LocalAllocator::offsetOfFreeList() + FreeList::offsetOfIntervalEnd()));
    auto bumpLabel = label();
    if (allocator.isConstant())
        add64(TrustedImm32(allocator.allocator().cellSize()), Address(allocatorGPR, LocalAllocator::offsetOfFreeList() + FreeList::offsetOfIntervalStart()));
    else {
        load32(Address(allocatorGPR, LocalAllocator::offsetOfCellSize()), scratchGPR);
        add64(scratchGPR, Address(allocatorGPR, LocalAllocator::offsetOfFreeList() + FreeList::offsetOfIntervalStart()));
    }
    done = jump();

    JIT_COMMENT(*this, "Get next interval (slower path)");
    popPath.link(this);
    loadPtr(Address(allocatorGPR, LocalAllocator::offsetOfFreeList() + FreeList::offsetOfNextInterval()), resultGPR);
    zeroPath = branchTestPtr(ResultCondition::NonZero, resultGPR, TrustedImm32(1));
    load32(Address(allocatorGPR, LocalAllocator::offsetOfFreeList() + FreeList::offsetOfSecret()), scratchGPR);
    xor32(Address(resultGPR, FreeCell::offsetOfScrambledBits()), scratchGPR); // Lower 32 bits -> offset to next interval
    add64(scratchGPR, Address(allocatorGPR, LocalAllocator::offsetOfFreeList() + FreeList::offsetOfNextInterval()));
    load32(Address(allocatorGPR, LocalAllocator::offsetOfFreeList() + FreeList::offsetOfSecret() + 4), scratchGPR);
    xor32(Address(resultGPR, FreeCell::offsetOfScrambledBits() + 4), scratchGPR); // Upper 32 bits -> size of interval
    storePtr(resultGPR, Address(allocatorGPR, LocalAllocator::offsetOfFreeList() + FreeList::offsetOfIntervalStart()));
    addPtr(resultGPR, scratchGPR);
    storePtr(scratchGPR, Address(allocatorGPR, LocalAllocator::offsetOfFreeList() + FreeList::offsetOfIntervalEnd()));
    jump(bumpLabel);
#else
    // Otherwise, we have a fairly general case for all other architectures here.

    // Bump allocation (fast path)
    loadPtr(Address(allocatorGPR, LocalAllocator::offsetOfFreeList() + FreeList::offsetOfIntervalStart()), resultGPR);
    loadPtr(Address(allocatorGPR, LocalAllocator::offsetOfFreeList() + FreeList::offsetOfIntervalEnd()), scratchGPR);
    popPath = branchPtr(RelationalCondition::AboveOrEqual, resultGPR, scratchGPR);
    auto bumpLabel = label();
    if (allocator.isConstant()) {
        move(resultGPR, scratchGPR);
        addPtr(TrustedImm32(allocator.allocator().cellSize()), scratchGPR);
    } else {
        load32(Address(allocatorGPR, LocalAllocator::offsetOfCellSize()), scratchGPR);
        addPtr(resultGPR, scratchGPR);
    }
    storePtr(scratchGPR, Address(allocatorGPR, LocalAllocator::offsetOfFreeList() + FreeList::offsetOfIntervalStart()));
    done = jump();

    // Get next interval (slower path)
    popPath.link(this);
    loadPtr(Address(allocatorGPR, LocalAllocator::offsetOfFreeList() + FreeList::offsetOfNextInterval()), resultGPR);
    zeroPath = branchTestPtr(ResultCondition::NonZero, resultGPR, TrustedImm32(1));
    load32(Address(allocatorGPR, LocalAllocator::offsetOfFreeList() + FreeList::offsetOfSecret()), scratchGPR);
    xor32(Address(resultGPR, FreeCell::offsetOfScrambledBits()), scratchGPR); // Lower 32 bits -> offset to next interval
    loadPtr(Address(allocatorGPR, LocalAllocator::offsetOfFreeList() + FreeList::offsetOfNextInterval()), dataTempRegister);
    addPtr(scratchGPR, dataTempRegister);
    storePtr(dataTempRegister, Address(allocatorGPR, LocalAllocator::offsetOfFreeList() + FreeList::offsetOfNextInterval()));
    load32(Address(allocatorGPR, LocalAllocator::offsetOfFreeList() + FreeList::offsetOfSecret() + 4), scratchGPR);
    xor32(Address(resultGPR, FreeCell::offsetOfScrambledBits() + 4), scratchGPR); // Upper 32 bits -> size of interval
    addPtr(resultGPR, scratchGPR);
    storePtr(scratchGPR, Address(allocatorGPR, LocalAllocator::offsetOfFreeList() + FreeList::offsetOfIntervalEnd()));
    jump(bumpLabel);
#endif

    if (slowAllocationResult == SlowAllocationResult::ClearToNull) {
        zeroPath.link(this);
        move(TrustedImm32(0), resultGPR);
        slowPath.append(jump());
    } else
        slowPath.append(zeroPath);

    done.link(this);
}

void AssemblyHelpers::emitAllocate(GPRReg resultGPR, const JITAllocator& allocator, GPRReg allocatorGPR, GPRReg scratchGPR, JumpList& slowPath, SlowAllocationResult slowAllocationResult)
{
    switch (allocator.kind()) {
    case JITAllocator::Constant:
        if (!allocator.allocator()) {
            slowPath.append(jump());
            return;
        }
        break;
    case JITAllocator::Variable:
        slowPath.append(branchTestPtr(Zero, allocatorGPR));
        break;
    case JITAllocator::VariableNonNull:
        break;
    }

    emitAllocateWithNonNullAllocator(resultGPR, allocator, allocatorGPR, scratchGPR, slowPath, slowAllocationResult);
}

void AssemblyHelpers::emitAllocateVariableSized(GPRReg resultGPR, const JITAllocator& allocator, Address subspaceAllocatorsBase, GPRReg allocationSize, GPRReg scratchGPR1, GPRReg scratchGPR2, JumpList& slowPath, SlowAllocationResult slowAllocationResult)
{
    static_assert(!(MarkedSpace::sizeStep & (MarkedSpace::sizeStep - 1)), "MarkedSpace::sizeStep must be a power of two.");

    unsigned stepShift = getLSBSet(MarkedSpace::sizeStep);

    addPtr(TrustedImm32(MarkedSpace::sizeStep - 1), allocationSize, scratchGPR1);
    urshiftPtr(TrustedImm32(stepShift), scratchGPR1);
    slowPath.append(branchPtr(Above, scratchGPR1, TrustedImmPtr(MarkedSpace::largeCutoff >> stepShift)));
    JIT_COMMENT(*this, "Load the Allocator from the CompleteSubspace's buffer");
    loadPtr(subspaceAllocatorsBase.indexedBy(scratchGPR1, ScalePtr), scratchGPR1);

    emitAllocate(resultGPR, allocator, scratchGPR1, scratchGPR2, slowPath, slowAllocationResult);
}

void AssemblyHelpers::emitAllocateVariableSized(GPRReg resultGPR, CompleteSubspace& subspace, GPRReg allocationSize, GPRReg scratchGPR1, GPRReg scratchGPR2, JumpList& slowPath, SlowAllocationResult slowAllocationResult)
{
    JIT_COMMENT(*this, "Materialize Allocator buffer base");
    move(TrustedImmPtr(subspace.allocatorsForSizeSteps().data()), scratchGPR2);
    emitAllocateVariableSized(resultGPR, JITAllocator::variable(), Address(scratchGPR2, 0), allocationSize, scratchGPR1, scratchGPR2, slowPath, slowAllocationResult);
}

void AssemblyHelpers::restoreCalleeSavesFromEntryFrameCalleeSavesBuffer(EntryFrame*& topEntryFrame)
{
#if NUMBER_OF_CALLEE_SAVES_REGISTERS > 0
    RegisterAtOffsetList* allCalleeSaves = RegisterSetBuilder::vmCalleeSaveRegisterOffsets();
    auto dontRestoreRegisters = RegisterSetBuilder::stackRegisters();
    unsigned registerCount = allCalleeSaves->registerCount();
    if constexpr (AssemblyHelpersInternal::dumpVerbose)
        JIT_COMMENT(*this, "restoreCalleeSavesFromEntryFrameCalleeSavesBuffer ", *allCalleeSaves, " skip: ", dontRestoreRegisters);
    else
        JIT_COMMENT(*this, "restoreCalleeSavesFromEntryFrameCalleeSavesBuffer");

    GPRReg scratch = InvalidGPRReg;
    unsigned scratchGPREntryIndex = 0;
#if CPU(ARM64)
    // We don't need a second scratch GPR, but we'll also defer restoring this
    // GPR (in the next slot after the scratch) so that we can restore them together
    // later using a loadPair64.
    GPRReg unusedNextSlotGPR = InvalidGPRReg;
#endif

    // Use the first GPR entry's register as our baseGPR.
    for (unsigned i = 0; i < registerCount; i++) {
        RegisterAtOffset entry = allCalleeSaves->at(i);
        if (dontRestoreRegisters.contains(entry.reg(), IgnoreVectors))
            continue;
        if (entry.reg().isGPR()) {
#if CPU(ARM64)
            if (i + 1 < registerCount) {
                RegisterAtOffset entry2 = allCalleeSaves->at(i + 1);
                if (!dontRestoreRegisters.contains(entry2.reg(), IgnoreVectors)
                    && entry2.reg().isGPR()
                    && entry2.offset() == entry.offset() + static_cast<ptrdiff_t>(sizeof(CPURegister))) {
                    scratchGPREntryIndex = i;
                    scratch = entry.reg().gpr();
                    unusedNextSlotGPR = entry2.reg().gpr();
                    break;
                }
            }
#else
            scratchGPREntryIndex = i;
            scratch = entry.reg().gpr();
            break;
#endif
        }
    }
    ASSERT(scratch != InvalidGPRReg);

    RegisterSet skipList;
    skipList.merge(dontRestoreRegisters);

    // Skip the scratch register(s). We'll restore them later.
    skipList.add(scratch, IgnoreVectors);
#if CPU(ARM64)
    RELEASE_ASSERT(unusedNextSlotGPR != InvalidGPRReg);
    skipList.add(unusedNextSlotGPR, IgnoreVectors);
#endif

    loadPtr(&topEntryFrame, scratch);
    restoreCalleeSavesFromVMEntryFrameCalleeSavesBufferImpl(scratch, skipList);

    // Restore the callee save value of the scratch.
    RegisterAtOffset entry = allCalleeSaves->at(scratchGPREntryIndex);
    ASSERT(!dontRestoreRegisters.contains(entry.reg(), IgnoreVectors));
    ASSERT(entry.reg().isGPR());
    ASSERT(scratch == entry.reg().gpr());
#if CPU(ARM64)
    RegisterAtOffset entry2 = allCalleeSaves->at(scratchGPREntryIndex + 1);
    ASSERT_UNUSED(entry2, !dontRestoreRegisters.contains(entry2.reg(), IgnoreVectors));
    ASSERT(entry2.reg().isGPR());
    ASSERT(unusedNextSlotGPR == entry2.reg().gpr());
    loadPair64(scratch, TrustedImm32(entry.offset()), scratch, unusedNextSlotGPR);
#else
    loadPtr(Address(scratch, entry.offset()), scratch);
#endif

#else
    UNUSED_PARAM(topEntryFrame);
#endif // NUMBER_OF_CALLEE_SAVES_REGISTERS > 0
}

void AssemblyHelpers::restoreCalleeSavesFromVMEntryFrameCalleeSavesBuffer(GPRReg vmGPR, GPRReg scratchGPR)
{
#if NUMBER_OF_CALLEE_SAVES_REGISTERS > 0
    loadPtr(Address(vmGPR, VM::topEntryFrameOffset()), scratchGPR);
    restoreCalleeSavesFromVMEntryFrameCalleeSavesBufferImpl(scratchGPR, RegisterSetBuilder::stackRegisters());
#else
    UNUSED_PARAM(vmGPR);
#endif // NUMBER_OF_CALLEE_SAVES_REGISTERS > 0
}

void AssemblyHelpers::restoreCalleeSavesFromVMEntryFrameCalleeSavesBufferImpl(GPRReg entryFrameGPR, const RegisterSet& skipList)
{
#if NUMBER_OF_CALLEE_SAVES_REGISTERS > 0
    addPtr(TrustedImm32(EntryFrame::calleeSaveRegistersBufferOffset()), entryFrameGPR);

    RegisterAtOffsetList* allCalleeSaves = RegisterSetBuilder::vmCalleeSaveRegisterOffsets();
    if constexpr (AssemblyHelpersInternal::dumpVerbose)
        JIT_COMMENT(*this, "restoreCalleeSavesFromVMEntryFrameCalleeSavesBufferImpl ", entryFrameGPR, " callee saves: ", *allCalleeSaves, " skip: ", skipList);
    else
        JIT_COMMENT(*this, "restoreCalleeSavesFromVMEntryFrameCalleeSavesBufferImpl");
    unsigned registerCount = allCalleeSaves->registerCount();

    LoadRegSpooler spooler(*this, entryFrameGPR);

    // Restore all callee saves except for the scratch.
    unsigned i = 0;
    for (; i < registerCount; i++) {
        RegisterAtOffset entry = allCalleeSaves->at(i);
        if (skipList.contains(entry.reg(), IgnoreVectors))
            continue;
        if (!entry.reg().isGPR())
            break;
        spooler.loadGPR(entry);
    }
    spooler.finalizeGPR();
    for (; i < registerCount; i++) {
        RegisterAtOffset entry = allCalleeSaves->at(i);
        if (skipList.contains(entry.reg(), IgnoreVectors))
            continue;
        ASSERT(!entry.reg().isGPR());
        spooler.loadFPR(entry);
    }
    spooler.finalizeFPR();

#else
    UNUSED_PARAM(vmGPR);
    UNUSED_PARAM(skipList);
#endif // NUMBER_OF_CALLEE_SAVES_REGISTERS > 0
}

void AssemblyHelpers::emitVirtualCall(VM& vm, CallLinkInfo* info)
{
    move(TrustedImmPtr(info), GPRInfo::regT2);
    emitVirtualCallWithoutMovingGlobalObject(vm, GPRInfo::regT2, info->callMode());
}

void AssemblyHelpers::emitVirtualCallWithoutMovingGlobalObject(VM& vm, GPRReg callLinkInfoGPR, CallMode callMode)
{
    move(callLinkInfoGPR, GPRInfo::regT2);
    nearCallThunk(CodeLocationLabel<JITStubRoutinePtrTag> { vm.getCTIVirtualCall(callMode).code() });
}

#if USE(JSVALUE64)
void AssemblyHelpers::wangsInt64Hash(GPRReg inputAndResult, GPRReg scratch)
{
    GPRReg input = inputAndResult;
    // key += ~(key << 32);
    lshift64(input, TrustedImm32(32), scratch);
    not64(scratch);
    add64(scratch, input);
    // key ^= (key >> 22);
    urshift64(input, TrustedImm32(22), scratch);
    xor64(scratch, input);
    // key += ~(key << 13);
    lshift64(input, TrustedImm32(13), scratch);
    not64(scratch);
    add64(scratch, input);
    // key ^= (key >> 8);
    urshift64(input, TrustedImm32(8), scratch);
    xor64(scratch, input);
    // key += (key << 3);
    lshift64(input, TrustedImm32(3), scratch);
    add64(scratch, input);
    // key ^= (key >> 15);
    urshift64(input, TrustedImm32(15), scratch);
    xor64(scratch, input);
    // key += ~(key << 27);
    lshift64(input, TrustedImm32(27), scratch);
    not64(scratch);
    add64(scratch, input);
    // key ^= (key >> 31);
    urshift64(input, TrustedImm32(31), scratch);
    xor64(scratch, input);

    // return static_cast<unsigned>(result)
    void* mask = std::bit_cast<void*>(static_cast<uintptr_t>(UINT_MAX));
    and64(TrustedImmPtr(mask), inputAndResult);
}
#endif // USE(JSVALUE64)

void AssemblyHelpers::emitConvertValueToBoolean(VM& vm, JSValueRegs value, GPRReg result, GPRReg scratchIfShouldCheckMasqueradesAsUndefined, FPRReg valueAsFPR, FPRReg tempFPR, bool shouldCheckMasqueradesAsUndefined, JSGlobalObject* globalObject, bool invert)
{
    // Implements the following control flow structure:
    // if (value is cell) {
    //     if (value is string or value is HeapBigInt)
    //         result = !!value->length
    //     else {
    //         do evil things for masquerades-as-undefined
    //         result = true
    //     }
    // } else if (value is int32) {
    //     result = !!unboxInt32(value)
    // } else if (value is number) {
    //     result = !!unboxDouble(value)
    // } else if (value is BigInt32) {
    //     result = !!unboxBigInt32(value)
    // } else {
    //     result = value == jsTrue
    // }

    JumpList done;

    auto notCell = branchIfNotCell(value);
    auto isString = branchIfString(value.payloadGPR());
    auto isHeapBigInt = branchIfHeapBigInt(value.payloadGPR());

    if (shouldCheckMasqueradesAsUndefined) {
        ASSERT(scratchIfShouldCheckMasqueradesAsUndefined != InvalidGPRReg);
        JumpList isNotMasqueradesAsUndefined;
        isNotMasqueradesAsUndefined.append(branchTest8(Zero, Address(value.payloadGPR(), JSCell::typeInfoFlagsOffset()), TrustedImm32(MasqueradesAsUndefined)));
        emitLoadStructure(vm, value.payloadGPR(), result);
        move(TrustedImmPtr(globalObject), scratchIfShouldCheckMasqueradesAsUndefined);
        isNotMasqueradesAsUndefined.append(branchPtr(NotEqual, Address(result, Structure::globalObjectOffset()), scratchIfShouldCheckMasqueradesAsUndefined));

        // We act like we are "undefined" here.
        move(invert ? TrustedImm32(1) : TrustedImm32(0), result);
        done.append(jump());
        isNotMasqueradesAsUndefined.link(this);
    }
    move(invert ? TrustedImm32(0) : TrustedImm32(1), result);
    done.append(jump());

    isString.link(this);
    move(TrustedImmPtr(jsEmptyString(vm)), result);
    comparePtr(invert ? Equal : NotEqual, value.payloadGPR(), result, result);
    done.append(jump());

    isHeapBigInt.link(this);
    load32(Address(value.payloadGPR(), JSBigInt::offsetOfLength()), result);
    compare32(invert ? Equal : NotEqual, result, TrustedImm32(0), result);
    done.append(jump());

    notCell.link(this);
    auto notInt32 = branchIfNotInt32(value);
    compare32(invert ? Equal : NotEqual, value.payloadGPR(), TrustedImm32(0), result);
    done.append(jump());

    notInt32.link(this);
    auto notDouble = branchIfNotDoubleKnownNotInt32(value);
#if USE(JSVALUE64)
    unboxDouble(value.gpr(), result, valueAsFPR);
#else
    unboxDouble(value, valueAsFPR);
#endif
    move(invert ? TrustedImm32(1) : TrustedImm32(0), result);
    done.append(branchDoubleZeroOrNaN(valueAsFPR, tempFPR));
    move(invert ? TrustedImm32(0) : TrustedImm32(1), result);
    done.append(jump());

    notDouble.link(this);
#if USE(BIGINT32)
    auto isNotBigInt32 = branchIfNotBigInt32(value.gpr(), result);
    move(value.gpr(), result);
    urshift64(TrustedImm32(16), result);
    compare32(invert ? Equal : NotEqual, result, TrustedImm32(0), result);
    done.append(jump());

    isNotBigInt32.link(this);
#endif // USE(BIGINT32)
#if USE(JSVALUE64)
    compare64(invert ? NotEqual : Equal, value.gpr(), TrustedImm32(JSValue::ValueTrue), result);
#else
    move(invert ? TrustedImm32(1) : TrustedImm32(0), result);
    done.append(branchIfNotBoolean(value, InvalidGPRReg));
    compare32(invert ? Equal : NotEqual, value.payloadGPR(), TrustedImm32(0), result);
#endif

    done.link(this);
}

AssemblyHelpers::JumpList AssemblyHelpers::branchIfValue(VM& vm, JSValueRegs value, GPRReg scratch, GPRReg scratchIfShouldCheckMasqueradesAsUndefined, FPRReg valueAsFPR, FPRReg tempFPR, bool shouldCheckMasqueradesAsUndefined, Variant<JSGlobalObject*, GPRReg, LazyGlobalObjectLoadTag> globalObject, bool invert)
{
    // Implements the following control flow structure:
    // if (value is cell) {
    //     if (value is string or value is HeapBigInt)
    //         result = !!value->length
    //     else {
    //         do evil things for masquerades-as-undefined
    //         result = true
    //     }
    // } else if (value is int32) {
    //     result = !!unboxInt32(value)
    // } else if (value is number) {
    //     result = !!unboxDouble(value)
    // } else if (value is BigInt32) {
    //     result = !!unboxBigInt32(value)
    // } else {
    //     result = value == jsTrue
    // }

    JumpList done;
    JumpList truthy;

    auto notCell = branchIfNotCell(value);
    auto isString = branchIfString(value.payloadGPR());
    auto isHeapBigInt = branchIfHeapBigInt(value.payloadGPR());

    if (shouldCheckMasqueradesAsUndefined) {
        ASSERT(scratchIfShouldCheckMasqueradesAsUndefined != InvalidGPRReg);
        JumpList isNotMasqueradesAsUndefined;
        isNotMasqueradesAsUndefined.append(branchTest8(Zero, Address(value.payloadGPR(), JSCell::typeInfoFlagsOffset()), TrustedImm32(MasqueradesAsUndefined)));
        emitLoadStructure(vm, value.payloadGPR(), scratch);
        if (std::holds_alternative<JSGlobalObject*>(globalObject))
            move(TrustedImmPtr(std::get<JSGlobalObject*>(globalObject)), scratchIfShouldCheckMasqueradesAsUndefined);
        else if (std::holds_alternative<GPRReg>(globalObject))
            move(std::get<GPRReg>(globalObject), scratchIfShouldCheckMasqueradesAsUndefined);
        else
            loadPtr(Address(GPRInfo::jitDataRegister, BaselineJITData::offsetOfGlobalObject()), scratchIfShouldCheckMasqueradesAsUndefined);
        isNotMasqueradesAsUndefined.append(branchPtr(NotEqual, Address(scratch, Structure::globalObjectOffset()), scratchIfShouldCheckMasqueradesAsUndefined));

        // We act like we are "undefined" here.
        if (invert)
            truthy.append(jump());
        else
            done.append(jump());

        if (invert)
            done.append(isNotMasqueradesAsUndefined);
        else
            truthy.append(isNotMasqueradesAsUndefined);
    } else {
        if (invert)
            done.append(jump());
        else
            truthy.append(jump());
    }

    isString.link(this);
    truthy.append(branchPtr(invert ? Equal : NotEqual, value.payloadGPR(), TrustedImmPtr(jsEmptyString(vm))));
    done.append(jump());

    isHeapBigInt.link(this);
    truthy.append(branchTest32(invert ? Zero : NonZero, Address(value.payloadGPR(), JSBigInt::offsetOfLength())));
    done.append(jump());

    notCell.link(this);
    auto notInt32 = branchIfNotInt32(value);
    truthy.append(branchTest32(invert ? Zero : NonZero, value.payloadGPR()));
    done.append(jump());

    notInt32.link(this);
    auto notDouble = branchIfNotDoubleKnownNotInt32(value);
#if USE(JSVALUE64)
    unboxDouble(value.gpr(), scratch, valueAsFPR);
#else
    unboxDouble(value, valueAsFPR);
#endif
    if (invert) {
        truthy.append(branchDoubleZeroOrNaN(valueAsFPR, tempFPR));
        done.append(jump());
    } else {
        done.append(branchDoubleZeroOrNaN(valueAsFPR, tempFPR));
        truthy.append(jump());
    }

    notDouble.link(this);
#if USE(BIGINT32)
    auto isNotBigInt32 = branchIfNotBigInt32(value.gpr(), scratch);
    move(value.gpr(), scratch);
    urshift64(TrustedImm32(16), scratch);
    truthy.append(branchTest32(invert ? Zero : NonZero, scratch));
    done.append(jump());

    isNotBigInt32.link(this);
#endif // USE(BIGINT32)
#if USE(JSVALUE64)
    truthy.append(branch64(invert ? NotEqual : Equal, value.gpr(), TrustedImm64(JSValue::encode(jsBoolean(true)))));
#else
    auto notBoolean = branchIfNotBoolean(value, InvalidGPRReg);
    if (invert)
        truthy.append(notBoolean);
    else
        done.append(notBoolean);
    truthy.append(branch32(invert ? Equal : NotEqual, value.payloadGPR(), TrustedImm32(0)));
#endif

    done.link(this);

    return truthy;
}

#if ENABLE(WEBASSEMBLY)
void AssemblyHelpers::storeWasmContextInstance(GPRReg src)
{
    JIT_COMMENT(*this, "Store wasm context instance from", src);
    move(src, GPRInfo::wasmContextInstancePointer);
    JIT_COMMENT(*this, "Store wasm context instance done");
}

void AssemblyHelpers::prepareWasmCallOperation(GPRReg instanceGPR)
{
    UNUSED_PARAM(instanceGPR);
#if !USE(BUILTIN_FRAME_ADDRESS) || ASSERT_ENABLED
    storePtr(GPRInfo::callFrameRegister, Address(instanceGPR, JSWebAssemblyInstance::offsetOfTemporaryCallFrame()));
#endif
}

#endif // ENABLE(WEBASSEMBLY)

void AssemblyHelpers::copyCalleeSavesToEntryFrameCalleeSavesBufferImpl(GPRReg calleeSavesBuffer)
{
    JIT_COMMENT(*this, "copyCalleeSavesToEntryFrameCalleeSavesBufferImpl ", calleeSavesBuffer);
#if NUMBER_OF_CALLEE_SAVES_REGISTERS > 0
    addPtr(TrustedImm32(EntryFrame::calleeSaveRegistersBufferOffset()), calleeSavesBuffer);

    RegisterAtOffsetList* allCalleeSaves = RegisterSetBuilder::vmCalleeSaveRegisterOffsets();
    auto dontCopyRegisters = RegisterSetBuilder::stackRegisters();
    unsigned registerCount = allCalleeSaves->registerCount();

    StoreRegSpooler spooler(*this, calleeSavesBuffer);

    unsigned i = 0;
    for (; i < registerCount; i++) {
        RegisterAtOffset entry = allCalleeSaves->at(i);
        if (dontCopyRegisters.contains(entry.reg(), IgnoreVectors))
            continue;
        if (!entry.reg().isGPR())
            break;
        spooler.storeGPR(entry);
    }
    spooler.finalizeGPR();
    for (; i < registerCount; i++) {
        RegisterAtOffset entry = allCalleeSaves->at(i);
        if (dontCopyRegisters.contains(entry.reg(), IgnoreVectors))
            continue;
        spooler.storeFPR(entry);
    }
    spooler.finalizeFPR();

#else
    UNUSED_PARAM(calleeSavesBuffer);
#endif
}

void AssemblyHelpers::cage(Gigacage::Kind kind, GPRReg storage)
{
#if GIGACAGE_ENABLED
    if (!Gigacage::isEnabled(kind))
        return;
    andPtr(TrustedImmPtr(Gigacage::mask(kind)), storage);
    addPtr(TrustedImmPtr(Gigacage::basePtr(kind)), storage);
#else
    UNUSED_PARAM(kind);
    UNUSED_PARAM(storage);
#endif
}

// length may be the same register as scratch.
void AssemblyHelpers::cageConditionally(Gigacage::Kind kind, GPRReg storage, GPRReg length, GPRReg scratch)
{
#if GIGACAGE_ENABLED
    if (Gigacage::isEnabled(kind)) {
        if (kind != Gigacage::Primitive || Gigacage::disablingPrimitiveGigacageIsForbidden())
            cage(kind, storage);
        else {
            JumpList done;
            done.append(branchTest8(NonZero, AbsoluteAddress(&Gigacage::disablePrimitiveGigacageRequested)));

            loadPtr(Gigacage::addressOfBasePtr(kind), scratch);
            done.append(branchTest64(Zero, scratch));
            andPtr(TrustedImmPtr(Gigacage::mask(kind)), storage);
            addPtr(scratch, storage);
            done.link(this);
        }
    }
#endif

    UNUSED_PARAM(kind);
    UNUSED_PARAM(storage);
    UNUSED_PARAM(length);
    UNUSED_PARAM(scratch);
}

void AssemblyHelpers::emitSave(const RegisterAtOffsetList& list)
{
    if constexpr (AssemblyHelpersInternal::dumpVerbose)
        JIT_COMMENT(*this, "emitSave ", list);
    StoreRegSpooler spooler(*this, framePointerRegister);

    size_t registerCount = list.registerCount();
    size_t i = 0;
    for (; i < registerCount; i++) {
        auto entry = list.at(i);
        if (!entry.reg().isGPR())
            break;
        spooler.storeGPR(entry);
    }
    spooler.finalizeGPR();

    for (; i < registerCount; i++)
        spooler.storeFPR(list.at(i));
    spooler.finalizeFPR();
}

void AssemblyHelpers::emitRestore(const RegisterAtOffsetList& list, GPRReg baseGPR)
{
    if constexpr (AssemblyHelpersInternal::dumpVerbose)
        JIT_COMMENT(*this, "emitRestore ", list);
    LoadRegSpooler spooler(*this, baseGPR);

    size_t registerCount = list.registerCount();
    size_t i = 0;
    for (; i < registerCount; i++) {
        auto entry = list.at(i);
        if (!entry.reg().isGPR())
            break;
        spooler.loadGPR(entry);
    }
    spooler.finalizeGPR();

    for (; i < registerCount; i++)
        spooler.loadFPR(list.at(i));
    spooler.finalizeFPR();
}

void AssemblyHelpers::emitSaveCalleeSavesFor(const RegisterAtOffsetList* calleeSaves)
{
    auto dontSaveRegisters = RegisterSetBuilder::stackRegisters();
    unsigned registerCount = calleeSaves->registerCount();
    if constexpr (AssemblyHelpersInternal::dumpVerbose)
        JIT_COMMENT(*this, "emitSaveCalleeSavesFor ", *calleeSaves, " dontRestore: ", dontSaveRegisters);
    else
        JIT_COMMENT(*this, "emitSaveCalleeSavesFor");

    StoreRegSpooler spooler(*this, framePointerRegister);

    unsigned i = 0;
    for (; i < registerCount; i++) {
        RegisterAtOffset entry = calleeSaves->at(i);
        if (entry.reg().isFPR())
            break;
        if (dontSaveRegisters.contains(entry.reg(), IgnoreVectors))
            continue;
        spooler.storeGPR(entry);
    }
    spooler.finalizeGPR();
    for (; i < registerCount; i++) {
        RegisterAtOffset entry = calleeSaves->at(i);
        if (dontSaveRegisters.contains(entry.reg(), IgnoreVectors))
            continue;
        spooler.storeFPR(entry);
    }
    spooler.finalizeFPR();
}

void AssemblyHelpers::emitRestoreCalleeSavesFor(const RegisterAtOffsetList* calleeSaves)
{
    auto dontRestoreRegisters = RegisterSetBuilder::stackRegisters();
    unsigned registerCount = calleeSaves->registerCount();
    if constexpr (AssemblyHelpersInternal::dumpVerbose)
        JIT_COMMENT(*this, "emitRestoreCalleeSavesFor ", *calleeSaves, " dontSave: ", dontRestoreRegisters);
    else
        JIT_COMMENT(*this, "emitRestoreCalleeSavesFor");

    LoadRegSpooler spooler(*this, framePointerRegister);

    unsigned i = 0;
    for (; i < registerCount; i++) {
        RegisterAtOffset entry = calleeSaves->at(i);
        if (entry.reg().isFPR())
            break;
        if (dontRestoreRegisters.contains(entry.reg(), IgnoreVectors))
            continue;
        spooler.loadGPR(entry);
    }
    spooler.finalizeGPR();
    for (; i < registerCount; i++) {
        RegisterAtOffset entry = calleeSaves->at(i);
        if (dontRestoreRegisters.contains(entry.reg(), IgnoreVectors))
            continue;
        spooler.loadFPR(entry);
    }
    spooler.finalizeFPR();
}

void AssemblyHelpers::copyLLIntBaselineCalleeSavesFromFrameOrRegisterToEntryFrameCalleeSavesBuffer(EntryFrame*& topEntryFrame, const RegisterSet& usedRegisters)
{
#if NUMBER_OF_CALLEE_SAVES_REGISTERS > 0
    // Copy saved calleeSaves on stack or unsaved calleeSaves in register to vm calleeSave buffer
    ScratchRegisterAllocator allocator(usedRegisters);
    GPRReg destBufferGPR = allocator.allocateScratchGPR();
    GPRReg temp1 = allocator.allocateScratchGPR();
    FPRReg fpTemp1 = allocator.allocateScratchFPR();
    GPRReg temp2 = allocator.allocateScratchGPR();
    FPRReg fpTemp2 = allocator.allocateScratchFPR();
    RELEASE_ASSERT(!allocator.didReuseRegisters());

    loadPtr(&topEntryFrame, destBufferGPR);
    addPtr(TrustedImm32(EntryFrame::calleeSaveRegistersBufferOffset()), destBufferGPR);

    CopySpooler spooler(*this, framePointerRegister, destBufferGPR, temp1, temp2, fpTemp1, fpTemp2);

    RegisterAtOffsetList* allCalleeSaves = RegisterSetBuilder::vmCalleeSaveRegisterOffsets();
    const RegisterAtOffsetList* currentCalleeSaves = &RegisterAtOffsetList::llintBaselineCalleeSaveRegisters();
    auto dontCopyRegisters = RegisterSetBuilder::stackRegisters();
    unsigned registerCount = allCalleeSaves->registerCount();

    unsigned i = 0;
    for (; i < registerCount; i++) {
        RegisterAtOffset entry = allCalleeSaves->at(i);
        if (dontCopyRegisters.contains(entry.reg(), IgnoreVectors))
            continue;
        RegisterAtOffset* currentFrameEntry = currentCalleeSaves->find(entry.reg());

        if (!entry.reg().isGPR())
            break;
        if (currentFrameEntry)
            spooler.loadGPR(currentFrameEntry->offset());
        else
            spooler.copyGPR(entry.reg().gpr());
        spooler.storeGPR(entry.offset());
    }
    spooler.finalizeGPR();

    for (; i < registerCount; i++) {
        RegisterAtOffset entry = allCalleeSaves->at(i);
        if (dontCopyRegisters.contains(entry.reg(), IgnoreVectors))
            continue;
        RegisterAtOffset* currentFrameEntry = currentCalleeSaves->find(entry.reg());

        RELEASE_ASSERT(entry.reg().isFPR());
        if (currentFrameEntry)
            spooler.loadFPR(currentFrameEntry->offset());
        else
            spooler.copyFPR(entry.reg().fpr());
        spooler.storeFPR(entry.offset());
    }
    spooler.finalizeFPR();

#else
    UNUSED_PARAM(topEntryFrame);
    UNUSED_PARAM(usedRegisters);
#endif
}

void AssemblyHelpers::emitSaveOrCopyLLIntBaselineCalleeSavesFor(CodeBlock* codeBlock, VirtualRegister offsetVirtualRegister, RestoreTagRegisterMode tagRegisterMode, GPRReg temp1, GPRReg temp2, GPRReg temp3)
{
    ASSERT_UNUSED(codeBlock, codeBlock);
    ASSERT(JITCode::isBaselineCode(codeBlock->jitType()));
    ASSERT(codeBlock->jitCode()->calleeSaveRegisters() == &RegisterAtOffsetList::llintBaselineCalleeSaveRegisters());

    const RegisterAtOffsetList* calleeSaves = &RegisterAtOffsetList::llintBaselineCalleeSaveRegisters();
    auto dontSaveRegisters = RegisterSetBuilder::stackRegisters();
    unsigned registerCount = calleeSaves->registerCount();

    GPRReg dstBufferGPR = temp1;
    addPtr(TrustedImm32(offsetVirtualRegister.offsetInBytes()), framePointerRegister, dstBufferGPR);

    CopySpooler spooler(*this, framePointerRegister, dstBufferGPR, temp2, temp3);

    for (unsigned i = 0; i < registerCount; i++) {
        RegisterAtOffset entry = calleeSaves->at(i);
        if (dontSaveRegisters.contains(entry.reg(), IgnoreVectors))
            continue;
        RELEASE_ASSERT(entry.reg().isGPR());

#if USE(JSVALUE32_64)
        UNUSED_PARAM(tagRegisterMode);
#else
        if (tagRegisterMode == CopyBaselineCalleeSavedRegistersFromBaseFrame)
            spooler.loadGPR(entry.offset());
        else
#endif
            spooler.copyGPR(entry.reg().gpr());
        spooler.storeGPR(entry.offset());
    }
    spooler.finalizeGPR();
}

void AssemblyHelpers::getArityPadding(VM& vm, unsigned numberOfParameters, GPRReg argumentCountIncludingThisGPR, GPRReg paddingOutputGPR, GPRReg scratchGPR0, GPRReg scratchGPR1, JumpList& stackOverflow)
{
    ASSERT(argumentCountIncludingThisGPR != paddingOutputGPR);
    ASSERT(numberOfParameters >= 1);

    // padding is `align2(numParameters + CallFrame::headerSizeInRegisters) - (argumentCountIncludingThis + CallFrame::headerSizeInRegisters)`
    //
    // 1. If `align2(numParameters + CallFrame::headerSizeInRegisters)` == argumentCountIncludingThis + CallFrame::headerSizeInRegisters, then
    //    padding is `numParameters + CallFrame::headerSizeInRegisters - argumentCountIncludingThis - CallFrame::headerSizeInRegisters` == `numParameters - argumentCountIncludingThis`.
    //    Since we already checked `numParameters > argumentCountIncludingThis`, it will be 1~.
    // 2. If `align2(numParameters + CallFrame::headerSizeInRegisters)` == argumentCountIncludingThis + CallFrame::headerSizeInRegisters + 1, then
    //    padding is `numParameters + CallFrame::headerSizeInRegisters + 1 - argumentCountIncludingThis - CallFrame::headerSizeInRegisters` == `numParameters + 1 - argumentCountIncludingThis`.
    //    Since we already checked `numParameters > argumentCountIncludingThis`, it will be 2~.
    //
    //  Then, we return align2(padding).
    static_assert(stackAlignmentRegisters() == 2);
    if (WTF::roundUpToMultipleOf(stackAlignmentRegisters(), numberOfParameters + CallFrame::headerSizeInRegisters) == numberOfParameters + CallFrame::headerSizeInRegisters)
        move(TrustedImm32(numberOfParameters), paddingOutputGPR);
    else
        move(TrustedImm32(numberOfParameters + 1), paddingOutputGPR);
    sub32(paddingOutputGPR, argumentCountIncludingThisGPR, paddingOutputGPR);

    add32(TrustedImm32(1), paddingOutputGPR, scratchGPR0);
    and32(TrustedImm32(~1U), scratchGPR0);
    lshiftPtr(TrustedImm32(3), scratchGPR0);
    subPtr(stackPointerRegister, scratchGPR0, scratchGPR1);
    stackOverflow.append(branchPtr(GreaterThan, AbsoluteAddress(vm.addressOfSoftStackLimit()), scratchGPR1));
}

#if USE(JSVALUE64)

AssemblyHelpers::JumpList AssemblyHelpers::branchIfResizableOrGrowableSharedTypedArrayIsOutOfBounds(GPRReg baseGPR, GPRReg scratchGPR, GPRReg scratch2GPR, std::optional<TypedArrayType> typedArrayType)
{
    ASSERT(scratchGPR != scratch2GPR);
    // valueGPR can be baseGPR

    JumpList outOfBounds;

    outOfBounds.append(branchTestPtr(MacroAssembler::Zero, MacroAssembler::Address(baseGPR, JSArrayBufferView::offsetOfVector())));

    load8(Address(baseGPR, JSArrayBufferView::offsetOfMode()), scratchGPR);
    and32(TrustedImm32(resizabilityAndAutoLengthMask), scratchGPR, scratch2GPR);
    auto canUseRawFieldsDirectly = branch32(BelowOrEqual, scratch2GPR, TrustedImm32(isGrowableSharedMode));

    // Three cases come here.
    //     GrowableSharedAutoLengthWastefulTypedArray
    //     ResizableNonSharedWastefulTypedArray
    //     ResizableNonSharedAutoLengthWastefulTypedArray

    loadPtr(Address(baseGPR, JSObject::butterflyOffset()), scratch2GPR);
    loadPtr(Address(scratch2GPR, Butterfly::offsetOfArrayBuffer()), scratch2GPR);

    auto isGrowableShared = branchTest32(NonZero, scratchGPR, TrustedImm32(isGrowableSharedMode));
#if USE(LARGE_TYPED_ARRAYS)
    load64(Address(scratch2GPR, ArrayBuffer::offsetOfSizeInBytes()), scratch2GPR);
#else
    load32(Address(scratch2GPR, ArrayBuffer::offsetOfSizeInBytes()), scratch2GPR);
#endif
    auto loadedFromResizable = jump();

    isGrowableShared.link(this);
    loadPtr(Address(scratch2GPR, ArrayBuffer::offsetOfShared()), scratch2GPR);
#if USE(LARGE_TYPED_ARRAYS)
    atomicLoad64(Address(scratch2GPR, SharedArrayBufferContents::offsetOfSizeInBytes()), scratch2GPR);
#else
    atomicLoad32(Address(scratch2GPR, SharedArrayBufferContents::offsetOfSizeInBytes()), scratch2GPR);
#endif

    loadedFromResizable.link(this);

    // It never overflows since it is already checked at the construction.
#if USE(LARGE_TYPED_ARRAYS)
    if (typedArrayType) {
        load64(Address(baseGPR, JSArrayBufferView::offsetOfLength()), scratchGPR);
        if (elementSize(typedArrayType.value()) > 1)
            lshift64(TrustedImm32(logElementSize(typedArrayType.value())), scratchGPR);
    } else {
        load8(Address(baseGPR, JSObject::typeInfoTypeOffset()), scratchGPR);
        addPtr(TrustedImmPtr(logElementSizes), scratchGPR);
        load8(Address(scratchGPR, -FirstTypedArrayType), scratchGPR);
        lshift64(Address(baseGPR, JSArrayBufferView::offsetOfLength()), scratchGPR, scratchGPR);
    }
    add64(Address(baseGPR, JSArrayBufferView::offsetOfByteOffset()), scratchGPR);
    outOfBounds.append(branch64(Above, scratchGPR, scratch2GPR));
#else
    if (typedArrayType) {
        load32(Address(baseGPR, JSArrayBufferView::offsetOfLength()), scratchGPR);
        if (elementSize(typedArrayType.value()) > 1)
            lshift32(TrustedImm32(logElementSize(typedArrayType.value())), scratchGPR);
    } else {
        load8(Address(baseGPR, JSObject::typeInfoTypeOffset()), scratchGPR);
        addPtr(TrustedImmPtr(logElementSizes), scratchGPR);
        load8(Address(scratchGPR, -FirstTypedArrayType), scratchGPR);
        lshift32(Address(baseGPR, JSArrayBufferView::offsetOfLength()), scratchGPR, scratchGPR);
    }
    add32(Address(baseGPR, JSArrayBufferView::offsetOfByteOffset()), scratchGPR);
    outOfBounds.append(branch32(Above, scratchGPR, scratch2GPR));
#endif

    canUseRawFieldsDirectly.link(this);

    return outOfBounds;
}

std::tuple<AssemblyHelpers::Jump, AssemblyHelpers::JumpList> AssemblyHelpers::loadTypedArrayByteLengthImpl(GPRReg baseGPR, GPRReg valueGPR, GPRReg scratchGPR, GPRReg scratch2GPR, std::optional<TypedArrayType> typedArrayType, TypedArrayField field)
{
    ASSERT(scratchGPR != scratch2GPR);
    ASSERT(scratchGPR != valueGPR);
    // scratch2GPR can be valueGPR
    // valueGPR can be baseGPR

    load8(Address(baseGPR, JSArrayBufferView::offsetOfMode()), scratchGPR);
    and32(TrustedImm32(resizabilityAndAutoLengthMask), scratchGPR, scratch2GPR);
    auto canUseRawFieldsDirectly = branch32(BelowOrEqual, scratch2GPR, TrustedImm32(isGrowableSharedMode));

    // Three cases come here.
    //     GrowableSharedAutoLengthWastefulTypedArray
    //     ResizableNonSharedWastefulTypedArray
    //     ResizableNonSharedAutoLengthWastefulTypedArray

    if (typedArrayType && typedArrayType.value() == TypeDataView)
        loadPtr(Address(baseGPR, JSDataView::offsetOfBuffer()), scratch2GPR);
    else {
        loadPtr(Address(baseGPR, JSObject::butterflyOffset()), scratch2GPR);
        loadPtr(Address(scratch2GPR, Butterfly::offsetOfArrayBuffer()), scratch2GPR);
    }

    auto isGrowableShared = branchTest32(NonZero, scratchGPR, TrustedImm32(isGrowableSharedMode));
#if USE(LARGE_TYPED_ARRAYS)
    load64(Address(scratch2GPR, ArrayBuffer::offsetOfSizeInBytes()), scratch2GPR);
#else
    load32(Address(scratch2GPR, ArrayBuffer::offsetOfSizeInBytes()), scratch2GPR);
#endif
    auto loadedFromResizable = jump();

    isGrowableShared.link(this);
    loadPtr(Address(scratch2GPR, ArrayBuffer::offsetOfShared()), scratch2GPR);
#if USE(LARGE_TYPED_ARRAYS)
    atomicLoad64(Address(scratch2GPR, SharedArrayBufferContents::offsetOfSizeInBytes()), scratch2GPR);
#else
    atomicLoad32(Address(scratch2GPR, SharedArrayBufferContents::offsetOfSizeInBytes()), scratch2GPR);
#endif

    loadedFromResizable.link(this);

    // It never overflows since it is already checked at the construction.
#if USE(LARGE_TYPED_ARRAYS)
    if (typedArrayType) {
        load64(Address(baseGPR, JSArrayBufferView::offsetOfLength()), scratchGPR);
        if (elementSize(typedArrayType.value()) > 1)
            lshift64(TrustedImm32(logElementSize(typedArrayType.value())), scratchGPR);
    } else {
        load8(Address(baseGPR, JSObject::typeInfoTypeOffset()), scratchGPR);
        addPtr(TrustedImmPtr(logElementSizes), scratchGPR);
        load8(Address(scratchGPR, -FirstTypedArrayType), scratchGPR);
        lshift64(Address(baseGPR, JSArrayBufferView::offsetOfLength()), scratchGPR, scratchGPR);
    }
    add64(Address(baseGPR, JSArrayBufferView::offsetOfByteOffset()), scratchGPR);
    auto outOfBounds = branch64(Above, scratchGPR, scratch2GPR);
#else
    if (typedArrayType) {
        load32(Address(baseGPR, JSArrayBufferView::offsetOfLength()), scratchGPR);
        if (elementSize(typedArrayType.value()) > 1)
            lshift32(TrustedImm32(logElementSize(typedArrayType.value())), scratchGPR);
    } else {
        load8(Address(baseGPR, JSObject::typeInfoTypeOffset()), scratchGPR);
        addPtr(TrustedImmPtr(logElementSizes), scratchGPR);
        load8(Address(scratchGPR, -FirstTypedArrayType), scratchGPR);
        lshift32(Address(baseGPR, JSArrayBufferView::offsetOfLength()), scratchGPR, scratchGPR);
    }
    add32(Address(baseGPR, JSArrayBufferView::offsetOfByteOffset()), scratchGPR);
    auto outOfBounds = branch32(Above, scratchGPR, scratch2GPR);
#endif

    auto nonAutoLength = branchTest8(Zero, Address(baseGPR, JSArrayBufferView::offsetOfMode()), TrustedImm32(isAutoLengthMode));

    JumpList doneCases;

#if USE(LARGE_TYPED_ARRAYS)
    if (field == TypedArrayField::ByteLength) {
        sub64(scratch2GPR, scratchGPR, valueGPR);
        ASSERT(typedArrayType);
        if (elementSize(typedArrayType.value()) > 1)
            and64(TrustedImm64(~static_cast<uint64_t>(elementSize(typedArrayType.value()) - 1)), valueGPR);
    } else {
        if (typedArrayType) {
            sub64(scratch2GPR, scratchGPR, valueGPR);
            if (elementSize(typedArrayType.value()) > 1)
                urshift64(TrustedImm32(logElementSize(typedArrayType.value())), valueGPR);
        } else {
            sub64(scratch2GPR, scratchGPR, scratch2GPR); // Do not change valueGPR until we use baseGPR.
            load8(Address(baseGPR, JSObject::typeInfoTypeOffset()), scratchGPR);
            addPtr(TrustedImmPtr(logElementSizes), scratchGPR);
            load8(Address(scratchGPR, -FirstTypedArrayType), scratchGPR);
            urshift64(scratch2GPR, scratchGPR, valueGPR);
        }
    }
#else
    if (field == TypedArrayField::ByteLength) {
        sub32(scratch2GPR, scratchGPR, valueGPR);
        ASSERT(typedArrayType);
        if (elementSize(typedArrayType.value()) > 1)
            and32(TrustedImm32(~static_cast<uint32_t>(elementSize(typedArrayType.value()) - 1)), valueGPR);
    } else {
        if (typedArrayType) {
            sub32(scratch2GPR, scratchGPR, valueGPR);
            if (elementSize(typedArrayType.value()) > 1)
                urshift32(TrustedImm32(logElementSize(typedArrayType.value())), valueGPR);
        } else {
            sub32(scratch2GPR, scratchGPR, scratch2GPR); // Do not change valueGPR until we use baseGPR.
            load8(Address(baseGPR, JSObject::typeInfoTypeOffset()), scratchGPR);
            addPtr(TrustedImmPtr(logElementSizes), scratchGPR);
            load8(Address(scratchGPR, -FirstTypedArrayType), scratchGPR);
            urshift32(scratch2GPR, scratchGPR, valueGPR);
        }
    }
#endif
    doneCases.append(jump());

    nonAutoLength.link(this);
    canUseRawFieldsDirectly.link(this);
#if USE(LARGE_TYPED_ARRAYS)
    load64(MacroAssembler::Address(baseGPR, JSArrayBufferView::offsetOfLength()), valueGPR);
    if (field == TypedArrayField::ByteLength) {
        ASSERT(typedArrayType);
        if (elementSize(typedArrayType.value()) > 1)
            lshift64(TrustedImm32(logElementSize(typedArrayType.value())), valueGPR);
    }
#else
    load32(MacroAssembler::Address(baseGPR, JSArrayBufferView::offsetOfLength()), valueGPR);
    if (field == TypedArrayField::ByteLength) {
        ASSERT(typedArrayType);
        if (elementSize(typedArrayType.value()) > 1)
            lshift32(TrustedImm32(logElementSize(typedArrayType.value())), valueGPR);
    }
#endif
    doneCases.append(jump());

    return { outOfBounds, doneCases };
}

void AssemblyHelpers::loadTypedArrayByteLengthCommonImpl(GPRReg baseGPR, GPRReg valueGPR, GPRReg scratchGPR, GPRReg scratch2GPR, std::optional<TypedArrayType> typedArrayType, TypedArrayField field)
{
    auto [outOfBounds, doneCases] = loadTypedArrayByteLengthImpl(baseGPR, valueGPR, scratchGPR, scratch2GPR, typedArrayType, field);
    outOfBounds.link(this);
    move(TrustedImm32(0), valueGPR);
    doneCases.link(this);
}

void AssemblyHelpers::loadTypedArrayByteLength(GPRReg baseGPR, GPRReg valueGPR, GPRReg scratchGPR, GPRReg scratch2GPR, TypedArrayType typedArrayType)
{
    loadTypedArrayByteLengthCommonImpl(baseGPR, valueGPR, scratchGPR, scratch2GPR, typedArrayType, TypedArrayField::ByteLength);
}

std::tuple<AssemblyHelpers::Jump, AssemblyHelpers::JumpList> AssemblyHelpers::loadDataViewByteLength(GPRReg baseGPR, GPRReg valueGPR, GPRReg scratchGPR, GPRReg scratch2GPR, TypedArrayType typedArrayType)
{
    return loadTypedArrayByteLengthImpl(baseGPR, valueGPR, scratchGPR, scratch2GPR, typedArrayType, TypedArrayField::ByteLength);
}

void AssemblyHelpers::loadTypedArrayLength(GPRReg baseGPR, GPRReg valueGPR, GPRReg scratchGPR, GPRReg scratch2GPR, std::optional<TypedArrayType> typedArrayType)
{
    loadTypedArrayByteLengthCommonImpl(baseGPR, valueGPR, scratchGPR, scratch2GPR, typedArrayType, TypedArrayField::Length);
}

#endif // ENABLE(JSVALUE64)

#if ENABLE(WEBASSEMBLY)
#if CPU(ARM64) || CPU(X86_64) || CPU(RISCV64) || CPU(ARM)
AssemblyHelpers::JumpList AssemblyHelpers::checkWasmStackOverflow(GPRReg instanceGPR, TrustedImm32 checkSize, GPRReg framePointerGPR)
{
#if CPU(ARM64)
    loadPtr(Address(instanceGPR, JSWebAssemblyInstance::offsetOfSoftStackLimit()), getCachedMemoryTempRegisterIDAndInvalidate());
    JumpList overflow;
    // Because address is within 48bit, this addition never causes overflow.
    addPtr(checkSize, memoryTempRegister); // TrustedImm32 would use dataTempRegister. Thus let's have limit in memoryTempRegister.
    overflow.append(branchPtr(LessThan, framePointerGPR, memoryTempRegister));
    return overflow;
#elif CPU(X86_64) || CPU(ARM)
    loadPtr(Address(instanceGPR, JSWebAssemblyInstance::offsetOfSoftStackLimit()), scratchRegister());
    JumpList overflow;
    // Because address is within 48bit, this addition never causes overflow.
    addPtr(checkSize, scratchRegister());
    overflow.append(branchPtr(LessThan, framePointerGPR, scratchRegister()));
    return overflow;
#elif CPU(RISCV64)
    loadPtr(Address(instanceGPR, JSWebAssemblyInstance::offsetOfSoftStackLimit()), memoryTempRegister);
    JumpList overflow;
    // Because address is within 48bit, this addition never causes overflow.
    addPtr(checkSize, memoryTempRegister); // TrustedImm32 would use dataTempRegister. Thus let's have limit in memoryTempRegister.
    overflow.append(branchPtr(LessThan, framePointerGPR, memoryTempRegister));
    return overflow;
#endif
}
#endif
#endif // ENABLE(WEBASSEMBLY)

} // namespace JSC

#endif // ENABLE(JIT)

WTF_ALLOW_UNSAFE_BUFFER_USAGE_END
