/*
 * Copyright (C) 2004-2022 Apple Inc. All rights reserved.
 * Copyright (C) 2010 Patrick Gansterer <paroga@paroga.com>
 * Copyright (C) 2012 Google Inc. All rights reserved.
 * Copyright (C) 2015 Yusuke Suzuki<utatane.tea@gmail.com>. All rights reserved.
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
 *
 */

#include "config.h"
#include <wtf/text/AtomStringImpl.h>

#include <wtf/Threading.h>
#include <wtf/text/AtomStringTable.h>
#include <wtf/text/StringHash.h>
#include <wtf/unicode/UTF8Conversion.h>

#if USE(WEB_THREAD)
#include <wtf/Lock.h>
#endif

namespace WTF {

using namespace Unicode;

#if USE(WEB_THREAD)

class AtomStringTableLocker : public Locker<Lock> {
    WTF_MAKE_NONCOPYABLE(AtomStringTableLocker);

    static Lock s_stringTableLock;
public:
    AtomStringTableLocker()
        : Locker<Lock>(s_stringTableLock)
    {
    }
};

Lock AtomStringTableLocker::s_stringTableLock;

#else

class AtomStringTableLocker {
    WTF_MAKE_NONCOPYABLE(AtomStringTableLocker);
public:
    AtomStringTableLocker() { }
};

#endif // USE(WEB_THREAD)

using StringTableImpl = AtomStringTable::StringTableImpl;

static ALWAYS_INLINE StringTableImpl& stringTable()
{
    return Thread::currentSingleton().atomStringTable()->table();
}

template<typename T, typename HashTranslator>
static inline Ref<AtomStringImpl> addToStringTable(AtomStringTableLocker&, StringTableImpl& atomStringTable, const T& value)
{
    auto addResult = atomStringTable.add<HashTranslator>(value);

    // If the string is newly-translated, then we need to adopt it.
    // The boolean in the pair tells us if that is so.
    if (addResult.isNewEntry)
        return adoptRef(static_cast<AtomStringImpl&>(*addResult.iterator->get()));
    return *static_cast<AtomStringImpl*>(addResult.iterator->get());
}

template<typename T, typename HashTranslator>
static inline Ref<AtomStringImpl> addToStringTable(const T& value)
{
    AtomStringTableLocker locker;
    return addToStringTable<T, HashTranslator>(locker, stringTable(), value);
}

using UCharBuffer = HashTranslatorCharBuffer<char16_t>;
struct UCharBufferTranslator {
    static unsigned hash(const UCharBuffer& buf)
    {
        return buf.hash;
    }

    static bool equal(AtomStringTable::StringEntry const& str, const UCharBuffer& buf)
    {
        return WTF::equal(str.get(), buf.characters);
    }

    static void translate(AtomStringTable::StringEntry& location, const UCharBuffer& buf, unsigned hash)
    {
        auto* pointer = &StringImpl::create8BitIfPossible(buf.characters).leakRef();
        pointer->setHash(hash);
        pointer->setIsAtom(true);
        location = pointer;
    }
};

struct HashedUTF8Characters {
    std::span<const char8_t> characters;
    Unicode::UTF16LengthWithHash length;
};

struct HashedUTF8CharactersTranslator {
    static unsigned hash(const HashedUTF8Characters& characters)
    {
        return characters.length.hash;
    }

    static bool equal(const AtomStringTable::StringEntry& passedString, const HashedUTF8Characters& characters)
    {
        auto* string = passedString.get();
        if (characters.length.lengthUTF16 != string->length())
            return false;

        // If buffer contains only ASCII characters, UTF-8 and UTF16 lengths are the same.
        if (characters.length.lengthUTF16 != characters.characters.size()) {
            if (string->is8Bit())
                return Unicode::equal(string->span8(), characters.characters);
            return Unicode::equal(string->span16(), characters.characters);
        }

        auto charactersLatin1 = byteCast<LChar>(characters.characters);
        if (string->is8Bit())
            return WTF::equal(string->span8().data(), charactersLatin1);
        return WTF::equal(string->span16().data(), charactersLatin1);
    }

    static void translate(AtomStringTable::StringEntry& location, const HashedUTF8Characters& characters, unsigned hash)
    {
        std::span<char16_t> target;
        auto newString = StringImpl::createUninitialized(characters.length.lengthUTF16, target);

        auto result = Unicode::convert(characters.characters, target);
        RELEASE_ASSERT(result.code == Unicode::ConversionResultCode::Success);

        if (result.isAllASCII)
            newString = StringImpl::create(byteCast<LChar>(characters.characters));

        auto* pointer = &newString.leakRef();
        pointer->setHash(hash);
        pointer->setIsAtom(true);
        location = pointer;
    }
};

RefPtr<AtomStringImpl> AtomStringImpl::add(std::span<const char16_t> characters)
{
    if (!characters.data())
        return nullptr;

    if (characters.empty())
        return static_cast<AtomStringImpl*>(StringImpl::empty());

    UCharBuffer buffer { characters };
    return addToStringTable<UCharBuffer, UCharBufferTranslator>(buffer);
}

RefPtr<AtomStringImpl> AtomStringImpl::add(HashTranslatorCharBuffer<char16_t>& buffer)
{
    if (!buffer.characters.data())
        return nullptr;

    if (buffer.characters.empty())
        return static_cast<AtomStringImpl*>(StringImpl::empty());

    return addToStringTable<UCharBuffer, UCharBufferTranslator>(buffer);
}

struct SubstringLocation {
    SUPPRESS_UNCOUNTED_MEMBER StringImpl* baseString;
    unsigned start;
    unsigned length;
};

struct SubstringTranslator {
    static void translate(AtomStringTable::StringEntry& location, const SubstringLocation& buffer, unsigned hash)
    {
        auto* pointer = &StringImpl::createSubstringSharingImpl(*buffer.baseString, buffer.start, buffer.length).leakRef();
        pointer->setHash(hash);
        pointer->setIsAtom(true);
        location = pointer;
    }
};

struct SubstringTranslator8 : SubstringTranslator {
    static unsigned hash(const SubstringLocation& buffer)
    {
        return StringHasher::computeHashAndMaskTop8Bits(buffer.baseString->span8().subspan(buffer.start, buffer.length));
    }

    static bool equal(AtomStringTable::StringEntry const& string, const SubstringLocation& buffer)
    {
        return WTF::equal(string.get(), buffer.baseString->span8().subspan(buffer.start, buffer.length));
    }
};

struct SubstringTranslator16 : SubstringTranslator {
    static unsigned hash(const SubstringLocation& buffer)
    {
        return StringHasher::computeHashAndMaskTop8Bits(buffer.baseString->span16().subspan(buffer.start, buffer.length));
    }

    static bool equal(AtomStringTable::StringEntry const& string, const SubstringLocation& buffer)
    {
        return WTF::equal(string.get(), buffer.baseString->span16().subspan(buffer.start, buffer.length));
    }
};

RefPtr<AtomStringImpl> AtomStringImpl::add(StringImpl* baseString, unsigned start, unsigned length)
{
    if (!baseString)
        return nullptr;

    if (!length || start >= baseString->length())
        return static_cast<AtomStringImpl*>(StringImpl::empty());

    unsigned maxLength = baseString->length() - start;
    if (length >= maxLength) {
        if (!start)
            return add(baseString);
        length = maxLength;
    }

    SubstringLocation buffer = { baseString, start, length };
    if (baseString->is8Bit())
        return addToStringTable<SubstringLocation, SubstringTranslator8>(buffer);
    return addToStringTable<SubstringLocation, SubstringTranslator16>(buffer);
}
    
using LCharBuffer = HashTranslatorCharBuffer<LChar>;
struct LCharBufferTranslator {
    static unsigned hash(const LCharBuffer& buf)
    {
        return buf.hash;
    }

    static bool equal(AtomStringTable::StringEntry const& str, const LCharBuffer& buf)
    {
        return WTF::equal(str.get(), buf.characters);
    }

    static void translate(AtomStringTable::StringEntry& location, const LCharBuffer& buf, unsigned hash)
    {
        auto* pointer = &StringImpl::create(buf.characters).leakRef();
        pointer->setHash(hash);
        pointer->setIsAtom(true);
        location = pointer;
    }
};

template<typename CharType>
struct BufferFromStaticDataTranslator {
    using Buffer = HashTranslatorCharBuffer<CharType>;
    static unsigned hash(const Buffer& buf)
    {
        return buf.hash;
    }

    static bool equal(AtomStringTable::StringEntry const& str, const Buffer& buf)
    {
        return WTF::equal(str.get(), buf.characters);
    }

    static void translate(AtomStringTable::StringEntry& location, const Buffer& buf, unsigned hash)
    {
        auto* pointer = &StringImpl::createWithoutCopying(buf.characters).leakRef();
        pointer->setHash(hash);
        pointer->setIsAtom(true);
        location = pointer;
    }
};

RefPtr<AtomStringImpl> AtomStringImpl::add(HashTranslatorCharBuffer<LChar>& buffer)
{
    if (!buffer.characters.data())
        return nullptr;

    if (buffer.characters.empty())
        return static_cast<AtomStringImpl*>(StringImpl::empty());

    return addToStringTable<LCharBuffer, LCharBufferTranslator>(buffer);
}

RefPtr<AtomStringImpl> AtomStringImpl::add(std::span<const LChar> characters)
{
    if (!characters.data())
        return nullptr;

    if (characters.empty())
        return static_cast<AtomStringImpl*>(StringImpl::empty());

    LCharBuffer buffer { characters };
    return addToStringTable<LCharBuffer, LCharBufferTranslator>(buffer);
}

Ref<AtomStringImpl> AtomStringImpl::addLiteral(std::span<const LChar> characters)
{
    ASSERT(characters.data());
    ASSERT(!characters.empty());

    LCharBuffer buffer { characters };
    return addToStringTable<LCharBuffer, BufferFromStaticDataTranslator<LChar>>(buffer);
}

static Ref<AtomStringImpl> addSymbol(AtomStringTableLocker& locker, StringTableImpl& atomStringTable, StringImpl& base)
{
    ASSERT(base.length());
    ASSERT(base.isSymbol());

    SubstringLocation buffer = { &base, 0, base.length() };
    if (base.is8Bit())
        return addToStringTable<SubstringLocation, SubstringTranslator8>(locker, atomStringTable, buffer);
    return addToStringTable<SubstringLocation, SubstringTranslator16>(locker, atomStringTable, buffer);
}

static inline Ref<AtomStringImpl> addSymbol(StringImpl& base)
{
    AtomStringTableLocker locker;
    return addSymbol(locker, stringTable(), base);
}

static Ref<AtomStringImpl> addStatic(AtomStringTableLocker& locker, StringTableImpl& atomStringTable, const StringImpl& base)
{
    ASSERT(base.length());
    ASSERT(base.isStatic());

    if (base.is8Bit()) {
        LCharBuffer buffer { base.span8(), base.hash() };
        return addToStringTable<LCharBuffer, BufferFromStaticDataTranslator<LChar>>(locker, atomStringTable, buffer);
    }
    UCharBuffer buffer { base.span16(), base.hash() };
    return addToStringTable<UCharBuffer, BufferFromStaticDataTranslator<char16_t>>(locker, atomStringTable, buffer);
}

static inline Ref<AtomStringImpl> addStatic(const StringImpl& base)
{
    AtomStringTableLocker locker;
    return addStatic(locker, stringTable(), base);
}

RefPtr<AtomStringImpl> AtomStringImpl::add(const StaticStringImpl* string)
{
    auto s = reinterpret_cast<const StringImpl*>(string);
    ASSERT(s->isStatic());
    return addStatic(*s);
}

Ref<AtomStringImpl> AtomStringImpl::addSlowCase(StringImpl& string)
{
    // This check is necessary for null symbols.
    // Their length is zero, but they are not AtomStringImpl.
    if (!string.length())
        return *static_cast<AtomStringImpl*>(StringImpl::empty());

    if (string.isStatic())
        return addStatic(string);

    if (string.isSymbol())
        return addSymbol(string);

    ASSERT_WITH_MESSAGE(!string.isAtom(), "AtomStringImpl should not hit the slow case if the string is already an atom.");

    AtomStringTableLocker locker;
    auto addResult = stringTable().add(&string);

    if (addResult.isNewEntry) {
        ASSERT(addResult.iterator->get() == &string);
        string.setIsAtom(true);
    }

    return *static_cast<AtomStringImpl*>(addResult.iterator->get());
}

Ref<AtomStringImpl> AtomStringImpl::addSlowCase(Ref<StringImpl>&& string)
{
    // This check is necessary for null symbols.
    // Their length is zero, but they are not AtomStringImpl.
    if (!string->length())
        return *static_cast<AtomStringImpl*>(StringImpl::empty());

    if (string->isStatic())
        return addStatic(WTFMove(string));

    if (string->isSymbol())
        return addSymbol(WTFMove(string));

    ASSERT_WITH_MESSAGE(!string->isAtom(), "AtomStringImpl should not hit the slow case if the string is already an atom.");

    AtomStringTableLocker locker;
    auto addResult = stringTable().add(string.ptr());

    if (addResult.isNewEntry) {
        ASSERT(addResult.iterator->get() == string.ptr());
        string->setIsAtom(true);
        return static_reference_cast<AtomStringImpl>(WTFMove(string));
    }

    return *static_cast<AtomStringImpl*>(addResult.iterator->get());
}

Ref<AtomStringImpl> AtomStringImpl::addSlowCase(AtomStringTable& stringTable, StringImpl& string)
{
    // This check is necessary for null symbols.
    // Their length is zero, but they are not AtomStringImpl.
    if (!string.length())
        return *static_cast<AtomStringImpl*>(StringImpl::empty());

    if (string.isStatic()) {
        AtomStringTableLocker locker;
        return addStatic(locker, stringTable.table(), string);
    }

    if (string.isSymbol()) {
        AtomStringTableLocker locker;
        return addSymbol(locker, stringTable.table(), string);
    }

    ASSERT_WITH_MESSAGE(!string.isAtom(), "AtomStringImpl should not hit the slow case if the string is already an atom.");

    AtomStringTableLocker locker;
    auto addResult = stringTable.table().add(&string);

    if (addResult.isNewEntry) {
        ASSERT(addResult.iterator->get() == &string);
        string.setIsAtom(true);
    }

    return *static_cast<AtomStringImpl*>(addResult.iterator->get());
}

// When removing a string from the table, we know it's already the one in the table, so no need for a string equality check.
struct AtomStringTableRemovalHashTranslator {
    static unsigned hash(const AtomStringImpl* string) { return string->hash(); }
    static bool equal(const AtomStringTable::StringEntry& a, const AtomStringImpl* b) { return a == b; }
};

void AtomStringImpl::remove(AtomStringImpl* string)
{
    ASSERT(string->isAtom());
    AtomStringTableLocker locker;
    auto& atomStringTable = stringTable();
    auto iterator = atomStringTable.find<AtomStringTableRemovalHashTranslator>(string);
    ASSERT_WITH_MESSAGE(iterator != atomStringTable.end(), "The string being removed is an atom in the string table of an other thread!");
    ASSERT(string == iterator->get());
    atomStringTable.remove(iterator);
}

RefPtr<AtomStringImpl> AtomStringImpl::lookUpSlowCase(StringImpl& string)
{
    ASSERT_WITH_MESSAGE(!string.isAtom(), "AtomStringImpl objects should return from the fast case.");

    if (!string.length())
        return static_cast<AtomStringImpl*>(StringImpl::empty());

    AtomStringTableLocker locker;
    auto& atomStringTable = stringTable();
    auto iterator = atomStringTable.find(&string);
    if (iterator != atomStringTable.end())
        return static_cast<AtomStringImpl*>(iterator->get());
    return nullptr;
}

RefPtr<AtomStringImpl> AtomStringImpl::add(std::span<const char8_t> characters)
{
    HashedUTF8Characters buffer { characters, computeUTF16LengthWithHash(characters) };
    if (!buffer.length.hash)
        return nullptr;
    return addToStringTable<HashedUTF8Characters, HashedUTF8CharactersTranslator>(buffer);
}

RefPtr<AtomStringImpl> AtomStringImpl::lookUp(std::span<const LChar> characters)
{
    AtomStringTableLocker locker;
    auto& table = stringTable();

    LCharBuffer buffer { characters };
    auto iterator = table.find<LCharBufferTranslator>(buffer);
    if (iterator != table.end())
        return static_cast<AtomStringImpl*>(iterator->get());
    return nullptr;
}

RefPtr<AtomStringImpl> AtomStringImpl::lookUp(std::span<const char16_t> characters)
{
    AtomStringTableLocker locker;
    auto& table = stringTable();

    UCharBuffer buffer { characters };
    auto iterator = table.find<UCharBufferTranslator>(buffer);
    if (iterator != table.end())
        return static_cast<AtomStringImpl*>(iterator->get());
    return nullptr;
}

#if ASSERT_ENABLED
bool AtomStringImpl::isInAtomStringTable(StringImpl* string)
{
    AtomStringTableLocker locker;
    return stringTable().contains(string);
}
#endif

} // namespace WTF
