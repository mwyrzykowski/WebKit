/*
 * Copyright (C) 2014 Frédéric Wang (fred.wang@free.fr). All rights reserved.
 * Copyright (C) 2016 Igalia S.L.
 * Copyright (C) 2016-2021 Apple Inc. All rights reserved.
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
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "config.h"
#include "RenderMathMLToken.h"

#if ENABLE(MATHML)

#include "MathMLElement.h"
#include "MathMLNames.h"
#include "MathMLTokenElement.h"
#include "PaintInfo.h"
#include "RenderBoxInlines.h"
#include "RenderBoxModelObjectInlines.h"
#include "RenderElement.h"
#include "RenderIterator.h"
#include "RenderStyleInlines.h"
#include <wtf/StdLibExtras.h>
#include <wtf/TZoneMallocInlines.h>

namespace WebCore {

using namespace MathMLNames;

WTF_MAKE_TZONE_OR_ISO_ALLOCATED_IMPL(RenderMathMLToken);

RenderMathMLToken::RenderMathMLToken(Type type, MathMLTokenElement& element, RenderStyle&& style)
    : RenderMathMLBlock(type, element, WTFMove(style))
{
}

RenderMathMLToken::RenderMathMLToken(Type type, Document& document, RenderStyle&& style)
    : RenderMathMLBlock(type, document, WTFMove(style))
{
}

RenderMathMLToken::~RenderMathMLToken() = default;

MathMLTokenElement& RenderMathMLToken::element()
{
    return static_cast<MathMLTokenElement&>(nodeForNonAnonymous());
}

void RenderMathMLToken::updateTokenContent()
{
    RenderMathMLBlock::updateFromElement();
    setMathVariantGlyphDirty();
}

// Entries for the mathvariant lookup tables.
// 'key' represents the Unicode character to be transformed and is used for searching the tables.
// 'replacement' represents the mapped mathvariant Unicode character.
struct MathVariantMapping {
    uint32_t key;
    uint32_t replacement;
};
static inline char32_t ExtractKey(const MathVariantMapping* entry) { return entry->key; }
static char32_t MathVariantMappingSearch(uint32_t key, const MathVariantMapping* table, size_t tableLength)
{
    if (const auto* entry = tryBinarySearch<const MathVariantMapping, char32_t>(table, tableLength, key, ExtractKey))
        return entry->replacement;

    return 0;
}

// Lookup tables for use with mathvariant mappings to transform a unicode character point to another unicode character that indicates the proper output.
// key represents one of two concepts.
// 1. In the Latin table it represents a hole in the mathematical alphanumeric block, where the character that should occupy that position is located elsewhere.
// 2. It represents an Arabic letter.
//  As a replacement, 0 is reserved to indicate no mapping was found.
static const MathVariantMapping arabicInitialMapTable[] = {
    { 0x628, 0x1EE21 },
    { 0x62A, 0x1EE35 },
    { 0x62B, 0x1EE36 },
    { 0x62C, 0x1EE22 },
    { 0x62D, 0x1EE27 },
    { 0x62E, 0x1EE37 },
    { 0x633, 0x1EE2E },
    { 0x634, 0x1EE34 },
    { 0x635, 0x1EE31 },
    { 0x636, 0x1EE39 },
    { 0x639, 0x1EE2F },
    { 0x63A, 0x1EE3B },
    { 0x641, 0x1EE30 },
    { 0x642, 0x1EE32 },
    { 0x643, 0x1EE2A },
    { 0x644, 0x1EE2B },
    { 0x645, 0x1EE2C },
    { 0x646, 0x1EE2D },
    { 0x647, 0x1EE24 },
    { 0x64A, 0x1EE29 }
};

static const MathVariantMapping arabicTailedMapTable[] = {
    { 0x62C, 0x1EE42 },
    { 0x62D, 0x1EE47 },
    { 0x62E, 0x1EE57 },
    { 0x633, 0x1EE4E },
    { 0x634, 0x1EE54 },
    { 0x635, 0x1EE51 },
    { 0x636, 0x1EE59 },
    { 0x639, 0x1EE4F },
    { 0x63A, 0x1EE5B },
    { 0x642, 0x1EE52 },
    { 0x644, 0x1EE4B },
    { 0x646, 0x1EE4D },
    { 0x64A, 0x1EE49 },
    { 0x66F, 0x1EE5F },
    { 0x6BA, 0x1EE5D }
};

static const MathVariantMapping arabicStretchedMapTable[] = {
    { 0x628, 0x1EE61 },
    { 0x62A, 0x1EE75 },
    { 0x62B, 0x1EE76 },
    { 0x62C, 0x1EE62 },
    { 0x62D, 0x1EE67 },
    { 0x62E, 0x1EE77 },
    { 0x633, 0x1EE6E },
    { 0x634, 0x1EE74 },
    { 0x635, 0x1EE71 },
    { 0x636, 0x1EE79 },
    { 0x637, 0x1EE68 },
    { 0x638, 0x1EE7A },
    { 0x639, 0x1EE6F },
    { 0x63A, 0x1EE7B },
    { 0x641, 0x1EE70 },
    { 0x642, 0x1EE72 },
    { 0x643, 0x1EE6A },
    { 0x645, 0x1EE6C },
    { 0x646, 0x1EE6D },
    { 0x647, 0x1EE64 },
    { 0x64A, 0x1EE69 },
    { 0x66E, 0x1EE7C },
    { 0x6A1, 0x1EE7E }
};

static const MathVariantMapping arabicLoopedMapTable[] = {
    { 0x627, 0x1EE80 },
    { 0x628, 0x1EE81 },
    { 0x62A, 0x1EE95 },
    { 0x62B, 0x1EE96 },
    { 0x62C, 0x1EE82 },
    { 0x62D, 0x1EE87 },
    { 0x62E, 0x1EE97 },
    { 0x62F, 0x1EE83 },
    { 0x630, 0x1EE98 },
    { 0x631, 0x1EE93 },
    { 0x632, 0x1EE86 },
    { 0x633, 0x1EE8E },
    { 0x634, 0x1EE94 },
    { 0x635, 0x1EE91 },
    { 0x636, 0x1EE99 },
    { 0x637, 0x1EE88 },
    { 0x638, 0x1EE9A },
    { 0x639, 0x1EE8F },
    { 0x63A, 0x1EE9B },
    { 0x641, 0x1EE90 },
    { 0x642, 0x1EE92 },
    { 0x644, 0x1EE8B },
    { 0x645, 0x1EE8C },
    { 0x646, 0x1EE8D },
    { 0x647, 0x1EE84 },
    { 0x648, 0x1EE85 },
    { 0x64A, 0x1EE89 }
};

static const MathVariantMapping arabicDoubleMapTable[] = {
    { 0x628, 0x1EEA1 },
    { 0x62A, 0x1EEB5 },
    { 0x62B, 0x1EEB6 },
    { 0x62C, 0x1EEA2 },
    { 0x62D, 0x1EEA7 },
    { 0x62E, 0x1EEB7 },
    { 0x62F, 0x1EEA3 },
    { 0x630, 0x1EEB8 },
    { 0x631, 0x1EEB3 },
    { 0x632, 0x1EEA6 },
    { 0x633, 0x1EEAE },
    { 0x634, 0x1EEB4 },
    { 0x635, 0x1EEB1 },
    { 0x636, 0x1EEB9 },
    { 0x637, 0x1EEA8 },
    { 0x638, 0x1EEBA },
    { 0x639, 0x1EEAF },
    { 0x63A, 0x1EEBB },
    { 0x641, 0x1EEB0 },
    { 0x642, 0x1EEB2 },
    { 0x644, 0x1EEAB },
    { 0x645, 0x1EEAC },
    { 0x646, 0x1EEAD },
    { 0x648, 0x1EEA5 },
    { 0x64A, 0x1EEA9 }
};

static const MathVariantMapping latinExceptionMapTable[] = {
    { 0x1D455, 0x210E },
    { 0x1D49D, 0x212C },
    { 0x1D4A0, 0x2130 },
    { 0x1D4A1, 0x2131 },
    { 0x1D4A3, 0x210B },
    { 0x1D4A4, 0x2110 },
    { 0x1D4A7, 0x2112 },
    { 0x1D4A8, 0x2133 },
    { 0x1D4AD, 0x211B },
    { 0x1D4BA, 0x212F },
    { 0x1D4BC, 0x210A },
    { 0x1D4C4, 0x2134 },
    { 0x1D506, 0x212D },
    { 0x1D50B, 0x210C },
    { 0x1D50C, 0x2111 },
    { 0x1D515, 0x211C },
    { 0x1D51D, 0x2128 },
    { 0x1D53A, 0x2102 },
    { 0x1D53F, 0x210D },
    { 0x1D545, 0x2115 },
    { 0x1D547, 0x2119 },
    { 0x1D548, 0x211A },
    { 0x1D549, 0x211D },
    { 0x1D551, 0x2124 }
};

constexpr char16_t greekUpperTheta = 0x03F4;
constexpr char16_t holeGreekUpperTheta = 0x03A2;
constexpr char16_t nabla = 0x2207;
constexpr char16_t partialDifferential = 0x2202;
constexpr char16_t greekUpperAlpha = 0x0391;
constexpr char16_t greekUpperOmega = 0x03A9;
constexpr char16_t greekLowerAlpha = 0x03B1;
constexpr char16_t greekLowerOmega = 0x03C9;
constexpr char16_t greekLunateEpsilonSymbol = 0x03F5;
constexpr char16_t greekThetaSymbol = 0x03D1;
constexpr char16_t greekKappaSymbol = 0x03F0;
constexpr char16_t greekPhiSymbol = 0x03D5;
constexpr char16_t greekRhoSymbol = 0x03F1;
constexpr char16_t greekPiSymbol = 0x03D6;
constexpr char16_t greekLetterDigamma = 0x03DC;
constexpr char32_t greekSmallLetterDigamma = 0x03DD;
constexpr char32_t mathBoldCapitalDigamma = 0x1D7CA;
constexpr char32_t mathBoldSmallDigamma = 0x1D7CB;

constexpr char16_t latinSmallLetterDotlessI = 0x0131;
constexpr char16_t latinSmallLetterDotlessJ = 0x0237;

constexpr char32_t mathItalicSmallDotlessI = 0x1D6A4;
constexpr char32_t mathItalicSmallDotlessJ = 0x1D6A5;

constexpr char32_t mathBoldUpperA = 0x1D400;
constexpr char32_t mathItalicUpperA = 0x1D434;
constexpr char32_t mathBoldSmallA = 0x1D41A;
constexpr char32_t mathBoldUpperAlpha = 0x1D6A8;
constexpr char32_t mathBoldSmallAlpha = 0x1D6C2;
constexpr char32_t mathItalicUpperAlpha = 0x1D6E2;
constexpr char32_t mathBoldDigitZero = 0x1D7CE;
constexpr char32_t mathDoubleStruckZero = 0x1D7D8;

constexpr char32_t mathBoldUpperTheta = 0x1D6B9;
constexpr char32_t mathBoldNabla = 0x1D6C1;
constexpr char32_t mathBoldPartialDifferential = 0x1D6DB;
constexpr char32_t mathBoldEpsilonSymbol = 0x1D6DC;
constexpr char32_t mathBoldThetaSymbol = 0x1D6DD;
constexpr char32_t mathBoldKappaSymbol = 0x1D6DE;
constexpr char32_t mathBoldPhiSymbol = 0x1D6DF;
constexpr char32_t mathBoldRhoSymbol = 0x1D6E0;
constexpr char32_t mathBoldPiSymbol = 0x1D6E1;

// Performs the character mapping needed to implement MathML's mathvariant attribute.
// It takes a unicode character and maps it to its appropriate mathvariant counterpart specified by mathvariant.
// The mapped character is typically located within Unicode's mathematical blocks (0x1D***, 0x1EE**) but there are exceptions which this function accounts for.
// Characters without a valid mapping or valid aMathvar value are returned
// unaltered.
// Characters already in the mathematical blocks (or are one of the exceptions) are never transformed.
// Acceptable values for mathvariant are specified in MathMLElement.h
// The transformable characters can be found at:
// http://lists.w3.org/Archives/Public/www-math/2013Sep/0012.html and
// https://en.wikipedia.org/wiki/Mathematical_Alphanumeric_Symbols
static char32_t mathVariant(char32_t codePoint, MathMLElement::MathVariant mathvariant)
{
    ASSERT(mathvariant >= MathMLElement::MathVariant::Normal && mathvariant <= MathMLElement::MathVariant::Stretched);

    if (mathvariant == MathMLElement::MathVariant::Normal)
        return codePoint; // Nothing to do here.

    // Exceptional characters with at most one possible transformation.
    if (codePoint == holeGreekUpperTheta)
        return codePoint; // Nothing at this code point is transformed
    if (codePoint == greekLetterDigamma) {
        if (mathvariant == MathMLElement::MathVariant::Bold)
            return mathBoldCapitalDigamma;
        return codePoint;
    }
    if (codePoint == greekSmallLetterDigamma) {
        if (mathvariant == MathMLElement::MathVariant::Bold)
            return mathBoldSmallDigamma;
        return codePoint;
    }
    if (codePoint == latinSmallLetterDotlessI) {
        if (mathvariant == MathMLElement::MathVariant::Italic)
            return mathItalicSmallDotlessI;
        return codePoint;
    }
    if (codePoint == latinSmallLetterDotlessJ) {
        if (mathvariant == MathMLElement::MathVariant::Italic)
            return mathItalicSmallDotlessJ;
        return codePoint;
    }

    // The Unicode mathematical blocks are divided into four segments: Latin, Greek, numbers and Arabic.
    // In the case of the first three baseChar represents the relative order in which the characters are encoded in the Unicode mathematical block, normalised to the first character of that sequence.
    char32_t baseChar = 0;
    enum CharacterType {
        Latin,
        Greekish,
        Number,
        Arabic
    };
    CharacterType varType;
    if (isASCIIUpper(codePoint)) {
        baseChar = codePoint - 'A';
        varType = Latin;
    } else if (isASCIILower(codePoint)) {
        // Lowercase characters are placed immediately after the uppercase characters in the Unicode mathematical block.
        // The constant subtraction represents the number of characters between the start of the sequence (capital A) and the first lowercase letter.
        baseChar = mathBoldSmallA - mathBoldUpperA + codePoint - 'a';
        varType = Latin;
    } else if (isASCIIDigit(codePoint)) {
        baseChar = codePoint - '0';
        varType = Number;
    } else if (greekUpperAlpha <= codePoint && codePoint <= greekUpperOmega) {
        baseChar = codePoint - greekUpperAlpha;
        varType = Greekish;
    } else if (greekLowerAlpha <= codePoint && codePoint <= greekLowerOmega) {
        // Lowercase Greek comes after uppercase Greek.
        // Note in this instance the presence of an additional character (Nabla) between the end of the uppercase Greek characters and the lowercase ones.
        baseChar = mathBoldSmallAlpha - mathBoldUpperAlpha + codePoint - greekLowerAlpha;
        varType = Greekish;
    } else if (0x0600 <= codePoint && codePoint <= 0x06FF) {
        // Arabic characters are defined within this range
        varType = Arabic;
    } else {
        switch (codePoint) {
        case greekUpperTheta:
            baseChar = mathBoldUpperTheta - mathBoldUpperAlpha;
            break;
        case nabla:
            baseChar = mathBoldNabla - mathBoldUpperAlpha;
            break;
        case partialDifferential:
            baseChar = mathBoldPartialDifferential - mathBoldUpperAlpha;
            break;
        case greekLunateEpsilonSymbol:
            baseChar = mathBoldEpsilonSymbol - mathBoldUpperAlpha;
            break;
        case greekThetaSymbol:
            baseChar = mathBoldThetaSymbol - mathBoldUpperAlpha;
            break;
        case greekKappaSymbol:
            baseChar = mathBoldKappaSymbol - mathBoldUpperAlpha;
            break;
        case greekPhiSymbol:
            baseChar = mathBoldPhiSymbol - mathBoldUpperAlpha;
            break;
        case greekRhoSymbol:
            baseChar = mathBoldRhoSymbol - mathBoldUpperAlpha;
            break;
        case greekPiSymbol:
            baseChar = mathBoldPiSymbol - mathBoldUpperAlpha;
            break;
        default:
            return codePoint;
        }
        varType = Greekish;
    }

    int8_t multiplier;
    if (varType == Number) {
        // Each possible number mathvariant is encoded in a single, contiguous block.
        // For example the beginning of the double struck number range follows immediately after the end of the bold number range.
        // multiplier represents the order of the sequences relative to the first one.
        switch (mathvariant) {
        case MathMLElement::MathVariant::Bold:
            multiplier = 0;
            break;
        case MathMLElement::MathVariant::DoubleStruck:
            multiplier = 1;
            break;
        case MathMLElement::MathVariant::SansSerif:
            multiplier = 2;
            break;
        case MathMLElement::MathVariant::BoldSansSerif:
            multiplier = 3;
            break;
        case MathMLElement::MathVariant::Monospace:
            multiplier = 4;
            break;
        default:
            // This mathvariant isn't defined for numbers or is otherwise normal.
            return codePoint;
        }
        // As the ranges are contiguous, to find the desired mathvariant range it is sufficient to
        // multiply the position within the sequence order (multiplier) with the period of the sequence (which is constant for all number sequences)
        // and to add the character point of the first character within the number mathvariant range.
        // To this the baseChar calculated earlier is added to obtain the final code point.
        return baseChar + multiplier * (mathDoubleStruckZero - mathBoldDigitZero) + mathBoldDigitZero;
    }
    if (varType == Greekish) {
        switch (mathvariant) {
        case MathMLElement::MathVariant::Bold:
            multiplier = 0;
            break;
        case MathMLElement::MathVariant::Italic:
            multiplier = 1;
            break;
        case MathMLElement::MathVariant::BoldItalic:
            multiplier = 2;
            break;
        case MathMLElement::MathVariant::BoldSansSerif:
            multiplier = 3;
            break;
        case MathMLElement::MathVariant::SansSerifBoldItalic:
            multiplier = 4;
            break;
        default:
            // This mathvariant isn't defined for Greek or is otherwise normal.
            return codePoint;
        }
        // See the Number case for an explanation of the following calculation.
        return baseChar + mathBoldUpperAlpha + multiplier * (mathItalicUpperAlpha - mathBoldUpperAlpha);
    }

    char32_t tempChar = 0;
    char32_t newChar;
    if (varType == Arabic) {
        // The Arabic mathematical block is not continuous, nor does it have a monotonic mapping to the unencoded characters, requiring the use of a lookup table.
        const MathVariantMapping* mapTable;
        size_t tableLength;
        switch (mathvariant) {
        case MathMLElement::MathVariant::Initial:
            mapTable = arabicInitialMapTable;
            tableLength = std::size(arabicInitialMapTable);
            break;
        case MathMLElement::MathVariant::Tailed:
            mapTable = arabicTailedMapTable;
            tableLength = std::size(arabicTailedMapTable);
            break;
        case MathMLElement::MathVariant::Stretched:
            mapTable = arabicStretchedMapTable;
            tableLength = std::size(arabicStretchedMapTable);
            break;
        case MathMLElement::MathVariant::Looped:
            mapTable = arabicLoopedMapTable;
            tableLength = std::size(arabicLoopedMapTable);
            break;
        case MathMLElement::MathVariant::DoubleStruck:
            mapTable = arabicDoubleMapTable;
            tableLength = std::size(arabicDoubleMapTable);
            break;
        default:
            return codePoint; // No valid transformations exist.
        }
        newChar = MathVariantMappingSearch(codePoint, mapTable, tableLength);
    } else {
        // Must be Latin
        if (mathvariant > MathMLElement::MathVariant::Monospace)
            return codePoint; // Latin doesn't support the Arabic mathvariants
        multiplier = static_cast<int>(mathvariant) - 2;
        // This is possible because the values for NS_MATHML_MATHVARIANT_* are chosen to coincide with the order in which the encoded mathvariant characters are located within their unicode block (less an offset to avoid None and Normal variants)
        // See the Number case for an explanation of the following calculation
        tempChar = baseChar + mathBoldUpperA + multiplier * (mathItalicUpperA - mathBoldUpperA);
        // There are roughly twenty characters that are located outside of the mathematical block, so the spaces where they ought to be are used as keys for a lookup table containing the correct character mappings.
        newChar = MathVariantMappingSearch(tempChar, latinExceptionMapTable, std::size(latinExceptionMapTable));
    }

    if (newChar)
        return newChar;
    if (varType == Latin)
        return tempChar;
    return codePoint; // This is an Arabic character without a corresponding mapping.
}

void RenderMathMLToken::computePreferredLogicalWidths()
{
    ASSERT(needsPreferredLogicalWidthsUpdate());

    if (m_mathVariantGlyphDirty)
        updateMathVariantGlyph();

    if (m_mathVariantCodePoint) {
        auto mathVariantGlyph = style().fontCascade().glyphDataForCharacter(m_mathVariantCodePoint.value(), m_mathVariantIsMirrored);
        if (mathVariantGlyph.font) {
            m_maxPreferredLogicalWidth = m_minPreferredLogicalWidth = mathVariantGlyph.font->widthForGlyph(mathVariantGlyph.glyph);
            adjustPreferredLogicalWidthsForBorderAndPadding();
            clearNeedsPreferredWidthsUpdate();
            return;
        }
    }

    RenderMathMLBlock::computePreferredLogicalWidths();
}

void RenderMathMLToken::updateMathVariantGlyph()
{
    ASSERT(m_mathVariantGlyphDirty);

    m_mathVariantCodePoint = std::nullopt;
    m_mathVariantGlyphDirty = false;

    // Early return if the token element contains RenderElements.
    // Note that the renderers corresponding to the children of the token element are wrapped inside an anonymous RenderBlock.
    if (const auto& block = downcast<RenderElement>(firstChild())) {
        if (childrenOfType<RenderElement>(*block).first())
            return;
    }

    const Ref tokenElement = element();
    if (auto codePoint = MathMLTokenElement::convertToSingleCodePoint(element().textContent())) {
        MathMLElement::MathVariant mathvariant = mathMLStyle().mathVariant();
        if (mathvariant == MathMLElement::MathVariant::None)
            mathvariant = tokenElement->hasTagName(MathMLNames::miTag) ? MathMLElement::MathVariant::Italic : MathMLElement::MathVariant::Normal;
        char32_t transformedCodePoint = mathVariant(codePoint.value(), mathvariant);
        if (transformedCodePoint != codePoint.value()) {
            m_mathVariantCodePoint = mathVariant(codePoint.value(), mathvariant);
            m_mathVariantIsMirrored = writingMode().isBidiRTL();
        }
    }
}

void RenderMathMLToken::styleDidChange(StyleDifference diff, const RenderStyle* oldStyle)
{
    RenderMathMLBlock::styleDidChange(diff, oldStyle);
    setMathVariantGlyphDirty();
}

void RenderMathMLToken::updateFromElement()
{
    RenderMathMLBlock::updateFromElement();
    setMathVariantGlyphDirty();
}

std::optional<LayoutUnit> RenderMathMLToken::firstLineBaseline() const
{
    if (m_mathVariantCodePoint) {
        auto mathVariantGlyph = style().fontCascade().glyphDataForCharacter(m_mathVariantCodePoint.value(), m_mathVariantIsMirrored);
        if (mathVariantGlyph.font)
            return LayoutUnit { static_cast<int>(lroundf(-mathVariantGlyph.font->boundsForGlyph(mathVariantGlyph.glyph).y())) } + borderAndPaddingBefore();
    }
    return RenderMathMLBlock::firstLineBaseline();
}

void RenderMathMLToken::layoutBlock(RelayoutChildren relayoutChildren, LayoutUnit pageLogicalHeight)
{
    ASSERT(needsLayout());

    insertPositionedChildrenIntoContainingBlock();

    if (relayoutChildren == RelayoutChildren::No && simplifiedLayout())
        return;

    layoutFloatingChildren();

    GlyphData mathVariantGlyph;
    if (m_mathVariantCodePoint)
        mathVariantGlyph = style().fontCascade().glyphDataForCharacter(m_mathVariantCodePoint.value(), m_mathVariantIsMirrored);

    if (!mathVariantGlyph.font) {
        RenderMathMLBlock::layoutBlock(relayoutChildren, pageLogicalHeight);
        return;
    }

    recomputeLogicalWidth();
    for (auto* child = firstInFlowChildBox(); child; child = child->nextInFlowSiblingBox())
        child->layoutIfNeeded();
    setLogicalWidth(LayoutUnit(mathVariantGlyph.font->widthForGlyph(mathVariantGlyph.glyph)));
    setLogicalHeight(LayoutUnit(mathVariantGlyph.font->boundsForGlyph(mathVariantGlyph.glyph).height()));

    adjustLayoutForBorderAndPadding();

    layoutOutOfFlowBoxes(relayoutChildren);

    updateScrollInfoAfterLayout();

    clearNeedsLayout();
}

void RenderMathMLToken::paint(PaintInfo& info, const LayoutPoint& paintOffset)
{
    RenderMathMLBlock::paint(info, paintOffset);

    // FIXME: Instead of using DrawGlyph, we may consider using the more general TextPainter so that we can apply mathvariant to strings with an arbitrary number of characters and preserve advanced CSS effects (text-shadow, etc).
    if (info.context().paintingDisabled() || info.phase != PaintPhase::Foreground || style().usedVisibility() != Visibility::Visible || !m_mathVariantCodePoint)
        return;

    auto mathVariantGlyph = style().fontCascade().glyphDataForCharacter(m_mathVariantCodePoint.value(), m_mathVariantIsMirrored);
    if (!mathVariantGlyph.font)
        return;

    GraphicsContextStateSaver stateSaver(info.context());
    info.context().setFillColor(style().visitedDependentColorWithColorFilter(CSSPropertyColor));

    LayoutUnit glyphAscent = static_cast<int>(lroundf(-mathVariantGlyph.font->boundsForGlyph(mathVariantGlyph.glyph).y()));
    // FIXME: If we're just drawing a single glyph, why do we need to compute an advance?
    auto advance = makeGlyphBufferAdvance(mathVariantGlyph.font->widthForGlyph(mathVariantGlyph.glyph));
    info.context().drawGlyphs(*mathVariantGlyph.font, singleElementSpan(mathVariantGlyph.glyph), singleElementSpan(advance), paintOffset + location() + LayoutPoint(borderLeft() + paddingLeft(), glyphAscent + borderAndPaddingBefore()), style().fontCascade().fontDescription().usedFontSmoothing());
}

void RenderMathMLToken::paintChildren(PaintInfo& paintInfo, const LayoutPoint& paintOffset, PaintInfo& paintInfoForChild, bool usePrintRect)
{
    if (m_mathVariantCodePoint) {
        auto mathVariantGlyph = style().fontCascade().glyphDataForCharacter(m_mathVariantCodePoint.value(), m_mathVariantIsMirrored);
        if (mathVariantGlyph.font)
            return;
    }

    RenderMathMLBlock::paintChildren(paintInfo, paintOffset, paintInfoForChild, usePrintRect);
}

}

#endif // ENABLE(MATHML)
