/*
 * Copyright (C) 2023-2024 Apple Inc. All rights reserved.
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

#if ENABLE(DFG_JIT)

#include "DFGAbstractInterpreter.h"
#include "DFGAbstractValue.h"
#include "DFGArrayifySlowPathGenerator.h"
#include "DFGBackwardsDominators.h"
#include "DFGCFG.h"
#include "DFGCallArrayAllocatorSlowPathGenerator.h"
#include "DFGCallCreateDirectArgumentsSlowPathGenerator.h"
#include "DFGControlEquivalenceAnalysis.h"
#include "DFGDominators.h"
#include "DFGFlowMap.h"
#include "DFGInPlaceAbstractState.h"
#include "DFGNaturalLoops.h"
#include "DFGSaneStringGetByValSlowPathGenerator.h"
#include "DFGSlowPathGenerator.h"
#include <wtf/TZoneMallocInlines.h>

namespace JSC { namespace DFG {

WTF_MAKE_SEQUESTERED_ARENA_ALLOCATED_IMPL(ArrayifySlowPathGenerator);
WTF_MAKE_SEQUESTERED_ARENA_ALLOCATED_IMPL(BackwardsCFG);
WTF_MAKE_SEQUESTERED_ARENA_ALLOCATED_IMPL(BackwardsDominators);
WTF_MAKE_SEQUESTERED_ARENA_ALLOCATED_IMPL(CFG);
WTF_MAKE_SEQUESTERED_ARENA_ALLOCATED_IMPL(CallArrayAllocatorSlowPathGenerator);
WTF_MAKE_SEQUESTERED_ARENA_ALLOCATED_IMPL(CallArrayAllocatorWithVariableSizeSlowPathGenerator);
WTF_MAKE_SEQUESTERED_ARENA_ALLOCATED_IMPL(CallArrayAllocatorWithVariableStructureVariableSizeSlowPathGenerator);
WTF_MAKE_SEQUESTERED_ARENA_ALLOCATED_IMPL(CallCreateDirectArgumentsSlowPathGenerator);
WTF_MAKE_SEQUESTERED_ARENA_ALLOCATED_IMPL(ControlEquivalenceAnalysis);
WTF_MAKE_SEQUESTERED_ARENA_ALLOCATED_IMPL(SaneStringGetByValSlowPathGenerator);

} } // namespace JSC::DFG

#endif // ENABLE(DFG_JIT)
