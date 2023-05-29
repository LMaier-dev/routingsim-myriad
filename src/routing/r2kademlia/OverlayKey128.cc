//
// Copyright (C) 2022 Institute of Telematics, KIT
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//

/**
 * Use OverlayKey from OverSim as starting point
 * @file OverlayKey128.cc
 * @author Roland Bless, Jakob Thomas, Sebastian Mies, Ingmar Baumgart
 */

#include <cassert>
#include <omnetpp.h>
#include <sstream>
#include <iomanip>
#include "OverlayKey128.h"

uint32_t OverlayKey::keyLength = MAX_KEYLENGTH;

OverlayKey::overlaykey_t OverlayKey::MSB_MASK = ~((OverlayKey::overlaykey_t) 0) >> ( MAX_KEYLENGTH - keyLength );
OverlayKey::overlaykey_t OverlayKey::MSB = ((OverlayKey::overlaykey_t) 1 << (keyLength-1));
//--------------------------------------------------------------------
// constants
//--------------------------------------------------------------------

// predefined keys
const OverlayKey OverlayKey::UNSPECIFIED_KEY(INVALIDKEYVALUE);
const OverlayKey OverlayKey::ZERO(0);
const OverlayKey OverlayKey::ONE(1);


//--------------------------------------------------------------------
// construction and destruction
//--------------------------------------------------------------------


// create a key out of a buffer
OverlayKey::OverlayKey(const unsigned char* buf, uint32_t size)
{
	int trimSize, offset;
	clear();
	trimSize = (int) std::min((uint32_t) sizeof(OverlayKey::overlaykey_t), size);
	offset = (uint32_t) sizeof(OverlayKey::overlaykey_t) - trimSize;
	memcpy( ((char*)&key) + offset, buf, trimSize );
	trim();
}

// create a key out of an string with the given base
OverlayKey::OverlayKey(const std::string& str, uint32_t base)
{
	if ((base != 2) and (base != 16)) {
		throw cRuntimeError("OverlayKey::OverlayKey(): Invalid base!");
	}
	uint8_t l2base;
	switch(base) {
	case 2: l2base= 1;
		break;
	case 16:
		l2base= 4;
		break;
	default:
		throw cRuntimeError("OverlayKey::OverlayKey(): Invalid base!");
		break;
	}
	std::string s(str);
	clear();

	if (s == "<unspec>") { // for deserializing
		(*this) = OverlayKey::UNSPECIFIED_KEY;
		return;
	}

	for (uint32_t i=0; i < s.size(); i++) {
		if ((s[i] >= '0') && (s[i] <= '9')) {
			s[i] -= '0';
		} else if ((s[i] >= 'a') && (s[i] <= 'f')) {
			s[i] -= ('a' - 10);
		} else if ((s[i] >= 'A') && (s[i] <= 'F')) {
			s[i] -= ('A' - 10);
		} else {
			throw cRuntimeError("OverlayKey::OverlayKey(): "
			                    "Invalid character in string!");
		}
		key|= (overlaykey_t) s[i] << i*l2base;
	}

	trim();
}



// default destructor
// OverlayKey::~OverlayKey() {
// 	EV_STATICCONTEXT;
// 	EV_TRACE << "OverlayKey: " << shortStr() << " destructor called, addr:" << static_cast<void*>(this) << endl;
// }

//--------------------------------------------------------------------
// string representations & node key attributes
//--------------------------------------------------------------------

void OverlayKey::setKeyLength(uint32_t length)
{
	if ((length < 1) or (length > OverlayKey::keyLength) or (length > MAX_KEYLENGTH)) {
		throw cRuntimeError(
				"OverlayKey::setKeyLength(): length must be <= %i "
				"and setKeyLength() must not be called twice "
				"with different length!", MAX_KEYLENGTH);
	}

	keyLength = length;

	MSB_MASK =  ~((overlaykey_t) 0) >> ( MAX_KEYLENGTH - keyLength );
	MSB = ((OverlayKey::overlaykey_t) 1 << (keyLength-1));
	// EV_STATICCONTEXT;
	// EV_TRACE << "keylength: " << keyLength << " sizeof(mp_limb_t): " << sizeof(mp_limb_t)
	// 	 << " aSize: " << aSize << " sizeof(key): " << sizeof(key)
	// 	 << " MSB_MASK: " << std::hex << OverlayKey::GMP_MSB_MASK << std::dec << endl;
}


bool OverlayKey::isValid(const std::string& str)
{
	std::string s(str);

	if (s == "<unspec>") {
		return true;
	}

	for (uint32_t i=0; i<s.size(); i++) {
		if ((s[i] >= '0') && (s[i] <= '9')) {
		} else if ((s[i] >= 'a') && (s[i] <= 'f')) {
		} else if ((s[i] >= 'A') && (s[i] <= 'F')) {
		} else {
			return false;
		}
	}

	return true;
}


std::string OverlayKey::toString(uint32_t base) const
{
	if ((base != 2) and (base != 16)) {
		throw cRuntimeError("OverlayKey::OverlayKey(): Invalid base!");
	}

	if (isUnspecified())
		return std::string("<unspec>");
	else {
		std::ostringstream outstr;
		std::ios_base::fmtflags flags(outstr.flags());
		if (base == 16) {			 
			outstr << std::hex << std::setfill('0') << std::setw((keyLength % 64)/4) << keyw[1];
			//if (keyw[1]!=0)
			//	outstr << std::setfill('0') << std::setw(16);
			outstr << std::hex << std::setfill('0') << std::setw(16) << keyw[0];
		}
		else {
			overlaykey_t bitmask((overlaykey_t) 1 << 127);
			for (uint8_t i= 0; i < 128; i++) {
				outstr << (((key & bitmask) == 0) ? '0' : '1');
				bitmask = bitmask >> 1;
			}
		}
		outstr.flags(flags);
		return outstr.str();
	}
}


//--------------------------------------------------------------------
// additional math
//--------------------------------------------------------------------

// returns the digit at bitposition bitpos (to the left)
uint32_t OverlayKey::getBitRange(uint32_t bitpos, uint32_t bits_per_digit) const
{
	if ((bitpos + bits_per_digit > OverlayKey::keyLength) or (bits_per_digit > 32)) {
		throw cRuntimeError("OverlayKey::getBitRange: Invalid range");
	}

	overlaykey_t bitmask(0xffffffff >> (32 - bits_per_digit));
	// shift bitmask up to position bitpos, do and, then shift result down again
	overlaykey_t result( (key & (bitmask << bitpos)) >> bitpos );

	ASSERT(result <= 0xffffffff);
	return (uint32_t) result;
}


double OverlayKey::toDouble() const
{
	double result = 0;
	uint8_t range = 32, length = getLength();
	for (uint8_t i = 0; i < length; i += 32) {
		if ((length - i) < 32) range = length - i;
		result += (getBitRange(i, range) * pow(2.0, i));
	}

	return result;
}


// returns a simple hash of the key
std::size_t OverlayKey::hash() const noexcept
{
/* By Paul Hsieh (C) 2004, 2005.  Covered under the LGPL 2.1 license.
   http://www.azillionmonkeys.com/qed/hash.html */

    #define get16bits(d) (*((const uint16_t *) (d)))

    const char *data = (const char *) &key;
    int len = OverlayKey::keyLength / 8;

    std::size_t hash = len, tmp;
    int rem;

    if (len <= 0 || data == NULL) return 0;

    rem = len & 3;
    len >>= 2;

    /* Main loop */
    for (;len > 0; len--) {
        hash  += get16bits (data);
        tmp    = (get16bits (data+2) << 11) ^ hash;
        hash   = (hash << 16) ^ tmp;
        data  += 2*sizeof (uint16_t);
        hash  += hash >> 11;
    }

    /* Handle end cases */
    switch (rem) {
    case 3: hash += get16bits (data);
    hash ^= hash << 16;
    hash ^= ((signed char)data[sizeof (uint16_t)]) << 18;
    hash += hash >> 11;
    break;
    case 2: hash += get16bits (data);
    hash ^= hash << 11;
    hash += hash >> 17;
    break;
    case 1: hash += (signed char)*data;
    hash ^= hash << 10;
    hash += hash >> 1;
    }

    /* Force "avalanching" of final 127 bits */
    hash ^= hash << 3;
    hash += hash >> 5;
    hash ^= hash << 4;
    hash += hash >> 17;
    hash ^= hash << 25;
    hash += hash >> 6;

    return hash;
}


// fill suffix with random bits starting at pos
OverlayKey OverlayKey::randomSuffix(uint32_t pos, cRNG* rng) const
{	
	overlaykey_t rndsuffix;
	omnet_random(rndsuffix, rng);
	overlaykey_t bitmask(((overlaykey_t) 1 << pos)-1);
	OverlayKey newKey( (key & ~bitmask) | (rndsuffix & bitmask) );
	newKey.trim();

	return newKey;
}


// fill prefix until pos with random bits
OverlayKey OverlayKey::randomPrefix(uint32_t pos, cRNG* rng) const
{
	overlaykey_t rndprefix;
	omnet_random(rndprefix, rng);
	overlaykey_t bitmask(((overlaykey_t) 1 << pos)-1);
	OverlayKey newKey( (key & bitmask) | (rndprefix & ~bitmask) );
	newKey.trim();

	return newKey;
}


// calculate shared prefix length (in number of equal digits)
uint32_t OverlayKey::sharedPrefixLength(const OverlayKey& compKey,
				        uint32_t bitsPerDigit) const
{
	overlaykey_t diff(compKey.key ^ key);
	if (diff == 0) // all bits match
		return (bitsPerDigit == 1) ? keyLength : keyLength / bitsPerDigit;

	unsigned int length= 0;
	while (length < keyLength and (diff & MSB) == 0)
	{
		diff= diff << 1;
		length++;
	}

	// length is number of matching bits
	// return number of matching digits
	return (bitsPerDigit == 1) ? length : length / bitsPerDigit;
}


// calculate log of base 2
int OverlayKey::log_2() const
{
	// all zero: undefined
	if (key == 0)
		return -1;
	// TODO start w.r.t. correct length
	uint8_t result(127);
	overlaykey_t mask((overlaykey_t) 1 << result);
	
	do {
		if ( (key & mask) == mask )
			return result;
		result--;
		mask= mask >> 1;
	} while (result > 0);

	return result;
}

// returns true, if this key is element of the interval (keyA, keyB)
bool OverlayKey::isBetween(const OverlayKey& keyA,
				           const OverlayKey& keyB) const
{
	if (isUnspecified() or keyA.isUnspecified() or keyB.isUnspecified())
		return false;

	if (*this == keyA)
		return false;
	else if (keyA < keyB)
		return ((*this > keyA) and (*this < keyB));
	else
		return ((*this > keyA) or (*this < keyB));
}

// returns true, if this key is element of the interval (keyA, keyB]
bool OverlayKey::isBetweenR(const OverlayKey& keyA, const OverlayKey& keyB) const
{
	if (isUnspecified() or keyA.isUnspecified() or keyB.isUnspecified())
		return false;

	if ((keyA == keyB) && (*this == keyA))
		return true;
	else if (keyA <= keyB)
		return ((*this > keyA) and (*this <= keyB));
	else
		return ((*this > keyA) or (*this <= keyB));
}

// returns true, if this key is element of the interval [keyA, keyB)
bool OverlayKey::isBetweenL(const OverlayKey& keyA,
			    const OverlayKey& keyB) const
{
	if (isUnspecified() or keyA.isUnspecified() or keyB.isUnspecified())
		return false;

	if ((keyA == keyB) and (*this == keyA))
		return true;
	else
	if (keyA <= keyB)
		return ((*this >= keyA) and (*this < keyB));
	else
		return ((*this >= keyA) or (*this < keyB));
}

// returns true, if this key is element of the interval [keyA, keyB]
bool OverlayKey::isBetweenLR(const OverlayKey& keyA,
			     const OverlayKey& keyB) const
{
	if (isUnspecified() or keyA.isUnspecified() or keyB.isUnspecified())
		return false;

	if ((keyA == keyB) and (*this == keyA))
		return true;
	else
	if (keyA <= keyB)
		return ((*this >= keyA) and (*this <= keyB));
	else
		return ((*this >= keyA) or (*this <= keyB));
}


//----------------------------------------------------------------------
// statics and globals
//----------------------------------------------------------------------

// default console output
std::ostream& operator<<(std::ostream& os, const OverlayKey& c)
{
	os << c.toString(16);
	return os;
};


// generate random number
OverlayKey OverlayKey::random(cRNG* rng)
{
	OverlayKey newKey(ZERO);

	omnet_random(newKey.key, rng);

	newKey.trim();

	return newKey;
}


// generate a key=2**exponent
OverlayKey OverlayKey::pow2( uint32_t exponent )
{
	if (exponent >= keyLength) {
		throw cRuntimeError("OverlayKey::pow2(): exponent >= keyLength!");
	}

	OverlayKey newKey(ZERO);

	newKey.key= ((OverlayKey::overlaykey_t) 1) << exponent;

	return newKey;
}

