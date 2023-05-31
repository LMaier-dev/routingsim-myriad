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
 * Represesentation of an overlay key.
 * Taken from OverSim. Adapted by Jakob Thomas
 * @file OverlayKey.h
 * @author Sebastian Mies, Ingmar Baumgart
 */

#ifndef __OVERLAYKEY_128_H_
#define __OVERLAYKEY_128_H_

#include <stdint.h>

#include <omnetpp.h>
#include <crng.h>

class BinaryValue;
class OverlayKeyBit;

using namespace omnetpp;

/**
 * A common overlay key class based on int128
 *
 */
class OverlayKey : public cObject
{
public:
	typedef unsigned __int128 overlaykey_t;
	//-------------------------------------------------------------------------
	// constants
	//-------------------------------------------------------------------------

	static const OverlayKey UNSPECIFIED_KEY; /**< OverlayKey without defined key */
	static const OverlayKey ZERO; /**< OverlayKey with key initialized as 0 */
	static const OverlayKey ONE; /**< OverlayKey with key initialized as 1 */
	static constexpr overlaykey_t INVALIDKEYVALUE = (overlaykey_t) ~0; /**< all bits set to 1 */

	//-------------------------------------------------------------------------
	// construction and destruction
	//-------------------------------------------------------------------------

	/**
	 * Default constructor
	 *
	 * Contructs an unspecified overlay key
	 */
	OverlayKey();

	/**
	 * Constructs an overlay key initialized with a common integer
	 *
	 * @param num The integer to initialize this key with
	 */
	OverlayKey( overlaykey_t num );

        /**
	 * Constructs overlay key out of two 64bit integer words
	 */
	OverlayKey( std::uint64_t highword, std::uint64_t lowword);

	/**
	 * Constructs a key out of a buffer
	 * @param buffer Source buffer
	 * @param size Buffer size (in bytes)
	 */
	OverlayKey( const unsigned char* buffer, uint32_t size);

	/**
	 * Constructs a key out of a string number.
	 */
	OverlayKey( const std::string& str, uint32_t base = 16 );

	/**
	 * Copy constructor.
	 *
	 * @param rhs The key to copy.
	 */
	OverlayKey( const OverlayKey& rhs );

	/**
	 * Move constructor.
	 *
	 * @param rhs The key to copy.
	 */
	OverlayKey( OverlayKey&& rhs );

	/**
	 * override dup() function from cObject base class
	 */
	virtual OverlayKey* dup() const override { return new OverlayKey(*this); };

	/**
	 * Default destructor.
	 *
	 * Does nothing ATM.
	 */
	//~OverlayKey();

	//-------------------------------------------------------------------------
	// string representations & node key attributes
	//-------------------------------------------------------------------------

	/**
	 * Returns a string representation of this key
	 *
	 * @return String representation of this key
	 */
	std::string toString( uint32_t base = 16 ) const;

	/**
	 * Returns a string representation of this key (from cObject)
	 *
	 * @return String representation of this key
	 */
	virtual std::string str() const override { return toString(); }
	/**
	 * Returns a short string representation of this key
	 *
	 * @return Short string representation of this key
	 */
	std::string shortStr() const { return toString().substr(0,6); }

	/**
	 * Common stdc++ console output method
	 */
	friend std::ostream& operator<<(std::ostream& os, const OverlayKey& c);

	/**
	 * Returns true, if the key is unspecified
	 *
	 * @return Returns true, if key is unspecified
	 */
	bool isUnspecified() const;

	/**
	 * Returns true, if the key is a valid string representation
	 *
	 * @return Returns true, if the key is a valid string representation
	 */

	static bool isValid(const std::string& str);

	//-------------------------------------------------------------------------
	// operators
	//-------------------------------------------------------------------------

	/**
	 * compares this to a given OverlayKey
	 *
	 * @param compKey the the OverlayKey to compare this to
	 * @return true if compKey->key is smaller than this->key, else false
	 */
	bool operator< ( const OverlayKey& compKey ) const;

	/**
	 * compares this to a given OverlayKey
	 *
	 * @param compKey the the OverlayKey to compare this to
	 * @return true if compKey->key is greater than this->key, else false
	 */
	bool operator> ( const OverlayKey& compKey ) const;

	/**
	 * compares this to a given OverlayKey
	 *
	 * @param compKey the the OverlayKey to compare this to
	 * @return true if compKey->key is smaller than or equal to this->key, else false
	 */
	bool operator<=( const OverlayKey& compKey ) const;

	/**
	 * compares this to a given OverlayKey
	 *
	 * @param compKey the the OverlayKey to compare this to
	 * @return true if compKey->key is greater than or equal to this->key, else false
	 */
	bool operator>=( const OverlayKey& compKey ) const;

	/**
	 * compares this to a given OverlayKey
	 *
	 * @param compKey the the OverlayKey to compare this to
	 * @return true if compKey->key is equal to this->key, else false
	 */
	bool operator==( const OverlayKey& compKey ) const;

	/**
	 * compares this to a given OverlayKey
	 *
	 * @param compKey the the OverlayKey to compare this to
	 * @return true if compKey->key is not equal to this->key, else false
	 */
	bool operator!=( const OverlayKey& compKey ) const;

	/**
	 * assigns OverlayKey of rhs to this->key
	 *
	 * @param rhs the OverlayKey with the defined key
	 * @return this OverlayKey object
	 */
	OverlayKey& operator= ( const OverlayKey& rhs );

	/**
	 * move assignment OverlayKey of rhs to this->key
	 *
	 * @param rhs the OverlayKey with the defined key
	 * @return this OverlayKey object
	 */
	OverlayKey& operator= ( OverlayKey&& rhs );

        /**
	 * substracts 1 from this->key
	 *
	 * @return this OverlayKey object
	 */
	OverlayKey& operator--();

	/**
	 * adds 1 to this->key
	 *
	 * @return this OverlayKey object
	 */
	OverlayKey& operator++();

	/**
	 * adds rhs->key to this->key
	 *
	 * @param rhs the OverlayKey with the defined key
	 * @return this OverlayKey object
	 */
	OverlayKey& operator+=( const OverlayKey& rhs );

	/**
	 * substracts rhs->key from this->key
	 *
	 * @param rhs the OverlayKey with the defined key
	 * @return this OverlayKey object
	 */
	OverlayKey& operator-=( const OverlayKey& rhs );

	/**
	 * adds rhs->key to this->key
	 *
	 * @param rhs the OverlayKey with the defined key
	 * @return this OverlayKey object
	 */
	OverlayKey  operator+ ( const OverlayKey& rhs ) const;

	/**
	 * substracts rhs->key from this->key
	 *
	 * @param rhs the OverlayKey with the defined key
	 * @return this OverlayKey object
	 */
	OverlayKey  operator- ( const OverlayKey& rhs ) const;

	/**
	 * substracts 1 from this->key
	 *
	 * @return this OverlayKey object
	 */
	OverlayKey  operator--( int );

	/**
	 * adds 1 to this->key
	 *
	 * @return this OverlayKey object
	 */
	OverlayKey  operator++( int );

	/**
	 * bitwise shift right
	 *
	 * @param num number of bits to shift
	 * @return this OverlayKey object
	 */
	OverlayKey  operator>>( uint32_t num ) const;

	/**
	 * bitwise shift left
	 *
	 * @param num number of bits to shift
	 * @return this OverlayKey object
	 */
	OverlayKey  operator<<( uint32_t num ) const;

	/**
	 * bitwise AND of rhs->key and this->key
	 *
	 * @param rhs the OverlayKey AND is calculated with
	 * @return this OverlayKey object
	 */
	OverlayKey  operator& ( const OverlayKey& rhs ) const;

	/**
	 * bitwise OR of rhs->key and this->key
	 *
	 * @param rhs the OverlayKey OR is calculated with
	 * @return this OverlayKey object
	 */
	OverlayKey  operator| ( const OverlayKey& rhs ) const;

	/**
	 * bitwise XOR of rhs->key and this->key
	 *
	 * @param rhs the OverlayKey XOR is calculated with
	 * @return this OverlayKey object
	 */
	OverlayKey  operator^ ( const OverlayKey& rhs ) const;

	/**
	 * bitwise NOT of this->key
	 *
	 * @return this OverlayKey object
	 */
	OverlayKey  operator~ () const;

	/**
	 * returns the n-th bit of this->key
	 *
	 * @param n the position of the returned bit
	 * @return the bit on position n in this->key
	 */
	OverlayKeyBit operator[]( uint32_t n );

	/**
	 * sets a bit of this->key
	 *
	 * @param pos the position of the bit to set
	 * @param value new value for bit at position pos
	 * @return *this
	 */
	OverlayKey& setBit(uint32_t pos, bool value);

	//-------------------------------------------------------------------------
	// additional math
	//-------------------------------------------------------------------------

	/**
	 * Returns a sub integer at position p with n-bits. p is counted starting
	 * from the least significant bit of the key as bit 0. Bit p of the key
	 * becomes bit 0 of the returned integer. n is counting "upwards", e.g.,
	 * p=0, n=8 returns the lowest byte
	 * p=8, n=8 returns the second lowest byte (higher byte of 16-bit word)
	 *
	 * @param p the position of the sub-integer
	 * @param n the number of bits to be returned (max.32)
	 * @return The sub-integer.
	 */
	uint32_t getBitRange(uint32_t p, uint32_t n) const;

	double toDouble() const;

	inline bool getBit(uint32_t p) const
	{
	    return getBitRange(p, 1);
	};

	// return as 64bit unsigned integer (folds high and low word together with XOR)
	std::uint64_t toUint64_t() const { return keyw[0] ^ keyw[1]; }
	/**
	 * Returns a hash value for the key
	 *
	 * @return size_t The hash value
	 */
	std::size_t hash() const noexcept;

	/**
	 * Returns the position of the msb in this key, which represents
	 * just the logarithm to base 2.
	 *
	 * @return The logarithm to base 2 of this key.
	 */
	int log_2() const;

	/**
	 * Fills the suffix starting at pos with random bits to lsb.
	 *
	 * @param pos
	 * @param rng the rng to use for randomness
	 * @return OverlayKey
	 */
	OverlayKey randomSuffix(uint32_t pos, cRNG* rng) const;

	/**
	 * Fills the prefix starting at pos with random bits to msb.
	 *
	 * @param pos
	 * @param rng the rng to use for randomness
	 * @return OverlayKey
	 */
	OverlayKey randomPrefix(uint32_t pos, cRNG* rng) const;

	/**
	 * Calculates the number of equal bits (digits) from the left with another
	 * Key (shared prefix length)
	 *
	 * @param compKey the Key to compare with
	 * @param bitsPerDigit optional number of bits per digit, default is 1
	 * @return length of shared prefix
	 */
	uint32_t sharedPrefixLength(const OverlayKey& compKey,
	                            uint32_t bitsPerDigit = 1) const;

	/**
	 * Returns true, if this key is element of the interval (keyA, keyB)
	 * on the ring.
	 *
	 * @param keyA The left border of the interval
	 * @param keyB The right border of the interval
	 * @return True, if the key is element of the interval (keyA, keyB)
	 */
	bool isBetween(const OverlayKey& keyA, const OverlayKey& keyB) const;

	/**
	 * Returns true, if this key is element of the interval (keyA, keyB]
	 * on the ring.
	 *
	 * @param keyA The left border of the interval
	 * @param keyB The right border of the interval
	 * @return True, if the key is element of the interval (keyA, keyB]
	 */
	bool isBetweenR(const OverlayKey& keyA, const OverlayKey& keyB) const;

	/**
	 * Returns true, if this key is element of the interval [keyA, keyB)
	 * on the ring.
	 *
	 * @param keyA The left border of the interval
	 * @param keyB The right border of the interval
	 * @return True, if the key is element of the interval [keyA, keyB)
	 */
	bool isBetweenL(const OverlayKey& keyA, const OverlayKey& keyB) const;

	/**
	 * Returns true, if this key is element of the interval [keyA, keyB]
	 * on the ring.
	 *
	 * @param keyA The left border of the interval
	 * @param keyB The right border of the interval
	 * @return True, if the key is element of the interval [keyA, keyB]
	 */
	bool isBetweenLR(const OverlayKey& keyA, const OverlayKey& keyB) const;

	//-------------------------------------------------------------------------
	// Raw read access, required for crypto hash functions
	//-------------------------------------------------------------------------
	
	const uint8_t* getRawKey() const { return  reinterpret_cast<const uint8_t*>(&key); }
	
	//-------------------------------------------------------------------------
	// static methods
	//-------------------------------------------------------------------------

	/**
	 * Set the length of an OverlayKey
	 *
	 * @param length keylength in bits
	 */
	static void setKeyLength(uint32_t length);

	/**
	 * Returns the length in number of bits.
	 *
	 * @return The length in number of bits.
	 */
	static uint32_t getLength();

	/**
	 * Returns a random key.
	 *
	 * @return A random key.
	 */
	static OverlayKey random(cRNG* rng);

	/**
	 * Returns the maximum key, i.e. a key filled with bit 1
	 *
	 * @return The maximum key, i.e. a key filled with bit 1
	 */
	static OverlayKey getMax();

	/**
	 * Returns a key 2^exponent.
	 *
	 * @param exponent The exponent.
	 * @return Key=2^exponent.
	 */
	static OverlayKey pow2(uint32_t exponent);

	/**
	 * Returns the bit length of the commen prefix of the two keys.
	 */
	static int getCommonPrefixLength(const OverlayKey& a, const OverlayKey& b);

private:
	// private constants

	static constexpr uint32_t MAX_KEYLENGTH = 128; /**< maximum length of the key */
	static uint32_t keyLength; /**< actual length of the key */
	static overlaykey_t MSB_MASK; /**< bits to fill up if key is not full length, e.g., 112bit */
	static overlaykey_t MSB; /** just the MSB **/

	union {
		overlaykey_t key; /**< the overlay key this object represents */
		std::uint64_t keyw[2]; // the same key as two 64-bit words
	};

	// private "helper" methods
	/**
	 * trims key after key operations
	 */
	void trim();

	/**
	 * set this->key to 0 and isUnspec to false
	 */
	void clear();
};



/**
 * An auxiliary class for single bits in OverlayKey
 *
 * Allows statements like "key[n] = true"
 */
class OverlayKeyBit
{
public:

	OverlayKeyBit(bool value, uint32_t pos, OverlayKey* key)
	    : bit(value), pos(pos), key(key)
	{};

	/** Converts to a boolean value */
	inline operator bool()
	{
	    return bit;
	};

	inline OverlayKeyBit& operator=(const OverlayKeyBit& value)
	{
	    key->setBit(pos, value.bit);
	    return *this;
	};

	/** Sets the corresponding bit to a boolean value
	* @param value value to set to
	*/
	inline OverlayKeyBit& operator=(bool value)
	{
	    key->setBit(pos, value);
	    return *this;
	};

	inline OverlayKeyBit& operator^=(bool value)
	{
	    key->setBit(pos, (*key)[pos] ^ value);
	    return *this;
	};

private:

	bool bit;
	uint32_t pos;
	OverlayKey* key;
};


namespace std {
	template<> class hash<OverlayKey>
	{
	public:
		size_t operator()(const OverlayKey& k) const;
	};


	inline
	std::size_t std::hash<OverlayKey>::operator()(const OverlayKey& k) const {
			using std::size_t;
			
			return k.hash();	
	};

}


// default construction: create an unspecified node key
inline
OverlayKey::OverlayKey() : key(INVALIDKEYVALUE)
{
}


// create a key out of a normal integer
inline
OverlayKey::OverlayKey(overlaykey_t num) : key(num)
{
	trim();
}

// create a key out of two 64bit integer words
inline
OverlayKey::OverlayKey( std::uint64_t highword, std::uint64_t lowword) : keyw{lowword, highword}
{
	trim();
};


// copy constructor
inline
OverlayKey::OverlayKey(const OverlayKey& rhs) : cObject(rhs), key(rhs.key)
{
}


// move constructor
inline
OverlayKey::OverlayKey(OverlayKey&& rhs) : cObject(rhs), key(rhs.key)
{
	rhs.key= INVALIDKEYVALUE;
}


// returns the length in bits
inline
uint32_t OverlayKey::getLength()
{
	return OverlayKey::keyLength;
}


inline
bool OverlayKey::isUnspecified() const
{
	return key == INVALIDKEYVALUE;
}

// returns a key with the maximum valid value
inline
OverlayKey OverlayKey::getMax()
{
	return OverlayKey(~0-1);
}

//--------------------------------------------------------------------
// operators
//--------------------------------------------------------------------

// assignment operator
inline
OverlayKey& OverlayKey::operator=(const OverlayKey& rhs)
{
	if (this == &rhs) // exclude assignment to itself, mempcy doesn't work then
		return *this;

	cObject::operator=(rhs);
	key= rhs.key;
	return *this;
}


// move assignment operator
inline
OverlayKey& OverlayKey::operator=(OverlayKey&& rhs)
{
	if (this == &rhs) // exclude assignment to itself, mempcy doesn't work then
		return *this;

	cObject::operator=(std::move(rhs));
	key= rhs.key;
	rhs.key= INVALIDKEYVALUE;
	return *this;
}


// sub one prefix operator
inline
OverlayKey& OverlayKey::operator--()
{
	key--;
	if (key == INVALIDKEYVALUE) // wrap from 0 to 0xFFFF....FFFE
		key--;
	trim();
	return *this;
}

// sub one postfix operator
inline
OverlayKey OverlayKey::operator--(int)
{
	OverlayKey clone(*this);
	key--;
	if (key == INVALIDKEYVALUE) // wrap from 0 to 0xFFFF....FFFE
		key--;
	trim();
	return clone;
}


// add one prefix operator
inline
OverlayKey& OverlayKey::operator++()
{
	key++;
        // wrap around at 0xFFFF....FFFE to 0
	if (key == INVALIDKEYVALUE)
		key++;
	trim();
	return *this;
}

// sub one postfix operator
inline
OverlayKey OverlayKey::operator++(int)
{
	OverlayKey clone(*this);
	key++;
	if (key == INVALIDKEYVALUE)
		key++;
	trim();
	return clone;
}


// add assign operator
inline
OverlayKey& OverlayKey::operator+=( const OverlayKey& rhs )
{
	key+= rhs.key;
	trim();
	return *this;
}


// sub assign operator
inline
OverlayKey& OverlayKey::operator-=( const OverlayKey& rhs )
{
	key-= rhs.key;
	trim();
	return *this;
}


// add operator
inline
OverlayKey OverlayKey::operator+(const OverlayKey& rhs) const
{
	OverlayKey result(*this);
	result += rhs;
	return result;
}


// sub operator
inline
OverlayKey OverlayKey::operator-(const OverlayKey& rhs) const
{
	OverlayKey result(*this);
	result -= rhs;
	return result;
}

// compare operators
inline
bool OverlayKey::operator<(const OverlayKey& compKey) const
{
	return key < compKey.key;
}

inline
bool OverlayKey::operator>(const OverlayKey& compKey) const
{
	return key > compKey.key;
}

inline
bool OverlayKey::operator<=(const OverlayKey& compKey) const
{
	return key <= compKey.key;
}

inline
bool OverlayKey::operator>=(const OverlayKey& compKey) const
{
	return key >= compKey.key;
}

inline
bool OverlayKey::operator==(const OverlayKey& compKey) const
{
	return key == compKey.key;
}

inline
bool OverlayKey::operator!=(const OverlayKey& compKey) const
{
	return key != compKey.key;
}

// bitwise xor
inline
OverlayKey OverlayKey::operator^ (const OverlayKey& rhs) const
{
	return key ^ rhs.key;
}

// bitwise or
inline
OverlayKey OverlayKey::operator| (const OverlayKey& rhs) const
{
	return key | rhs.key;
}

// bitwise and
inline
OverlayKey OverlayKey::operator& (const OverlayKey& rhs) const
{
	return key & rhs.key;
}

// complement
inline
OverlayKey OverlayKey::operator~() const
{
	OverlayKey result(~key);
	result.trim();

	return result;
}


// get bit
inline
OverlayKeyBit OverlayKey::operator[](uint32_t n)
{
	return OverlayKeyBit(getBit(n), n, this);
}


inline
int OverlayKey::getCommonPrefixLength(const OverlayKey& a, const OverlayKey& b)
{
	return getLength() - (a ^ b).log_2();
}


// bitwise shift right
inline
OverlayKey OverlayKey::operator>>(uint32_t num) const
{
	OverlayKey result(*this);

	result.key= key >> num;
	result.trim();
	return result;
}

// bitwise shift left
inline
OverlayKey OverlayKey::operator<<(uint32_t num) const
{
	OverlayKey result(*this);

	result.key= key << num;
	result.trim();
	return result;
}


inline
OverlayKey& OverlayKey::setBit(uint32_t pos, bool value)
{
	if (pos >= keyLength) {
		throw cRuntimeError("OverlayKey::setBitAt(): "
		                    "pos >= keyLength!");
	}

	overlaykey_t bitvalue(1 << pos);
	if (value) {
		key |= bitvalue;
	} else {
		key &= ~bitvalue;
	}

	return *this;
};

//--------------------------------------------------------------------
// private methods (mostly inlines)
//--------------------------------------------------------------------

// trims uppermost (unused) bits of a key after each operation
// except if key == INVALIDKEYVALUE 
inline
void OverlayKey::trim()
{
	if (key != INVALIDKEYVALUE)
		key &= MSB_MASK;
}


// sets key to zero and clears unspecified bit
inline
void OverlayKey::clear()
{
	key= INVALIDKEYVALUE;
}


// fill in overlay key with random bits (not trimmed)
inline
void omnet_random(OverlayKey::overlaykey_t& key, cRNG* rng)
{
	key= 0;
	// with small probability the random key will hit
	// the invalid value, in this case we just repeat
	do {
		// fill in 32 bit chunks
		uint32_t tmpword;
		for (uint32_t i= 0; i < 4; i++) {
			tmpword= intuniform(rng, 0, 0xFFFFFFFF);
			key<<= 32;
			key|= tmpword;
		}
	}
	while (key == OverlayKey::INVALIDKEYVALUE);
}


#endif
