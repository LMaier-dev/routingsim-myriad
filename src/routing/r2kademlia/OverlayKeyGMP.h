//
// Copyright (C) 2006 Institut fuer Telematik, Universitaet Karlsruhe (TH)
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
 * @file OverlayKeyGMP.h
 * @author Sebastian Mies, Ingmar Baumgart
 */

#ifndef __OVERLAYKEY_GMP_H_
#define __OVERLAYKEY_GMP_H_

// uses GNU multiple precision functions
#include <gmp.h>
#include <stdint.h>

#include <omnetpp.h>
#include <crng.h>
#include <limits>

using namespace omnetpp;

namespace routingsim {

using OverlayKey = class OverlayKeyGMP;

class BinaryValue;
class OverlayKeyGMPBit;

/**
 * replacement function for mpn_random() using omnet's rng
 */
inline void omnet_random_gmp(mp_limb_t *r1p, mp_size_t r1n, cRNG* rng);

/**
 * A common overlay key class.
 *
 * Wraps common functions from Gnu MP library.
 *
 * @author Sebastian Mies.
 */
class OverlayKeyGMP : public cObject
{
public:
	//-------------------------------------------------------------------------
	// constants
	//-------------------------------------------------------------------------

	static const OverlayKeyGMP UNSPECIFIED_KEY; /**< OverlayKeyGMP without defined key */
	static const OverlayKeyGMP ZERO; /**< OverlayKeyGMP with key initialized as 0 */
	static const OverlayKeyGMP ONE; /**< OverlayKeyGMP with key initialized as 1 */

	//-------------------------------------------------------------------------
	// construction and destruction
	//-------------------------------------------------------------------------

	/**
	 * Default constructor
	 *
	 * Contructs an unspecified overlay key
	 */
	OverlayKeyGMP();

	/**
	 * Constructs an overlay key initialized with a common integer
	 *
	 * @param num The integer to initialize this key with
	 */
	OverlayKeyGMP( uint32_t num );

	/**
	 * Constructs a key out of a buffer
	 * @param buffer Source buffer
	 * @param size Buffer size (in bytes)
	 */
	OverlayKeyGMP( const unsigned char* buffer, uint32_t size);

	/**
	 * Constructs a key out of a string number.
	 */
	OverlayKeyGMP( const std::string& str, uint32_t base = 16 );

	/**
	 * Copy constructor.
	 *
	 * @param rhs The key to copy.
	 */
	OverlayKeyGMP( const OverlayKeyGMP& rhs );


	/**
	 * override dup() function from cObject base class
	 */
	virtual OverlayKeyGMP* dup() const override { return new OverlayKeyGMP(*this); };

	/**
	 * Default destructor.
	 *
	 * Does nothing ATM.
	 */
	//~OverlayKeyGMP();

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

	static bool isValid(std::string str);

	//-------------------------------------------------------------------------
	// operators
	//-------------------------------------------------------------------------

	/**
	 * compares this to a given OverlayKeyGMP
	 *
	 * @param compKey the the OverlayKeyGMP to compare this to
	 * @return true if compKey->key is smaller than this->key, else false
	 */
	bool operator< ( const OverlayKeyGMP& compKey ) const;

	/**
	 * compares this to a given OverlayKeyGMP
	 *
	 * @param compKey the the OverlayKeyGMP to compare this to
	 * @return true if compKey->key is greater than this->key, else false
	 */
	bool operator> ( const OverlayKeyGMP& compKey ) const;

	/**
	 * compares this to a given OverlayKeyGMP
	 *
	 * @param compKey the the OverlayKeyGMP to compare this to
	 * @return true if compKey->key is smaller than or equal to this->key, else false
	 */
	bool operator<=( const OverlayKeyGMP& compKey ) const;

	/**
	 * compares this to a given OverlayKeyGMP
	 *
	 * @param compKey the the OverlayKeyGMP to compare this to
	 * @return true if compKey->key is greater than or equal to this->key, else false
	 */
	bool operator>=( const OverlayKeyGMP& compKey ) const;

	/**
	 * compares this to a given OverlayKeyGMP
	 *
	 * @param compKey the the OverlayKeyGMP to compare this to
	 * @return true if compKey->key is equal to this->key, else false
	 */
	bool operator==( const OverlayKeyGMP& compKey ) const;

	/**
	 * compares this to a given OverlayKeyGMP
	 *
	 * @param compKey the the OverlayKeyGMP to compare this to
	 * @return true if compKey->key is not equal to this->key, else false
	 */
	bool operator!=( const OverlayKeyGMP& compKey ) const;

	/**
	 * Unifies all compare operations in one method
	 *
	 * @param compKey key to compare with
	 * @return int -1 if smaller, 0 if equal, 1 if greater
	 */
	int compareTo( const OverlayKeyGMP& compKey ) const;

	/**
	 * assigns OverlayKeyGMP of rhs to this->key
	 *
	 * @param rhs the OverlayKeyGMP with the defined key
	 * @return this OverlayKeyGMP object
	 */
	OverlayKeyGMP& operator= ( const OverlayKeyGMP& rhs );

	/**
	 * substracts 1 from this->key
	 *
	 * @return this OverlayKeyGMP object
	 */
	OverlayKeyGMP& operator--();

	/**
	 * adds 1 to this->key
	 *
	 * @return this OverlayKeyGMP object
	 */
	OverlayKeyGMP& operator++();

	/**
	 * adds rhs->key to this->key
	 *
	 * @param rhs the OverlayKeyGMP with the defined key
	 * @return this OverlayKeyGMP object
	 */
	OverlayKeyGMP& operator+=( const OverlayKeyGMP& rhs );

	/**
	 * substracts rhs->key from this->key
	 *
	 * @param rhs the OverlayKeyGMP with the defined key
	 * @return this OverlayKeyGMP object
	 */
	OverlayKeyGMP& operator-=( const OverlayKeyGMP& rhs );

	/**
	 * adds rhs->key to this->key
	 *
	 * @param rhs the OverlayKeyGMP with the defined key
	 * @return this OverlayKeyGMP object
	 */
	OverlayKeyGMP  operator+ ( const OverlayKeyGMP& rhs ) const;

	/**
	 * substracts rhs->key from this->key
	 *
	 * @param rhs the OverlayKeyGMP with the defined key
	 * @return this OverlayKeyGMP object
	 */
	OverlayKeyGMP  operator- ( const OverlayKeyGMP& rhs ) const;

	/**
	 * substracts 1 from this->key
	 *
	 * @return this OverlayKeyGMP object
	 */
	OverlayKeyGMP  operator--( int );

	/**
	 * adds 1 to this->key
	 *
	 * @return this OverlayKeyGMP object
	 */
	OverlayKeyGMP  operator++( int );

	/**
	 * bitwise shift right
	 *
	 * @param num number of bits to shift
	 * @return this OverlayKeyGMP object
	 */
	OverlayKeyGMP  operator>>( uint32_t num ) const;

	/**
	 * bitwise shift left
	 *
	 * @param num number of bits to shift
	 * @return this OverlayKeyGMP object
	 */
	OverlayKeyGMP  operator<<( uint32_t num ) const;

	/**
	 * bitwise AND of rhs->key and this->key
	 *
	 * @param rhs the OverlayKeyGMP AND is calculated with
	 * @return this OverlayKeyGMP object
	 */
	OverlayKeyGMP  operator& ( const OverlayKeyGMP& rhs ) const;

	/**
	 * bitwise OR of rhs->key and this->key
	 *
	 * @param rhs the OverlayKeyGMP OR is calculated with
	 * @return this OverlayKeyGMP object
	 */
	OverlayKeyGMP  operator| ( const OverlayKeyGMP& rhs ) const;

	/**
	 * bitwise XOR of rhs->key and this->key
	 *
	 * @param rhs the OverlayKeyGMP XOR is calculated with
	 * @return this OverlayKeyGMP object
	 */
	OverlayKeyGMP  operator^ ( const OverlayKeyGMP& rhs ) const;

	/**
	 * bitwise NOT of this->key
	 *
	 * @return this OverlayKeyGMP object
	 */
	OverlayKeyGMP  operator~ () const;

	/**
	 * returns the n-th bit of this->key
	 *
	 * @param n the position of the returned bit
	 * @return the bit on position n in this->key
	 */
	OverlayKeyGMPBit operator[]( uint32_t n );

	/**
	 * sets a bit of this->key
	 *
	 * @param pos the position of the bit to set
	 * @param value new value for bit at position pos
	 * @return *this
	 */
	OverlayKeyGMP& setBit(uint32_t pos, bool value);

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
	 * @return OverlayKeyGMP
	 */
	OverlayKeyGMP randomSuffix(uint32_t pos, cRNG* rng) const;

	/**
	 * Fills the prefix starting at pos with random bits to msb.
	 *
	 * @param pos
	 * @param rng the rng to use for randomness
	 * @return OverlayKeyGMP
	 */
	OverlayKeyGMP randomPrefix(uint32_t pos, cRNG* rng) const;

	/**
	 * Calculates the number of equal bits (digits) from the left with another
	 * Key (shared prefix length)
	 *
	 * @param compKey the Key to compare with
	 * @param bitsPerDigit optional number of bits per digit, default is 1
	 * @return length of shared prefix
	 */
	uint32_t sharedPrefixLength(const OverlayKeyGMP& compKey,
	                            uint32_t bitsPerDigit = 1) const;

	/**
	 * Returns true, if this key is element of the interval (keyA, keyB)
	 * on the ring.
	 *
	 * @param keyA The left border of the interval
	 * @param keyB The right border of the interval
	 * @return True, if the key is element of the interval (keyA, keyB)
	 */
	bool isBetween(const OverlayKeyGMP& keyA, const OverlayKeyGMP& keyB) const;

	/**
	 * Returns true, if this key is element of the interval (keyA, keyB]
	 * on the ring.
	 *
	 * @param keyA The left border of the interval
	 * @param keyB The right border of the interval
	 * @return True, if the key is element of the interval (keyA, keyB]
	 */
	bool isBetweenR(const OverlayKeyGMP& keyA, const OverlayKeyGMP& keyB) const;

	/**
	 * Returns true, if this key is element of the interval [keyA, keyB)
	 * on the ring.
	 *
	 * @param keyA The left border of the interval
	 * @param keyB The right border of the interval
	 * @return True, if the key is element of the interval [keyA, keyB)
	 */
	bool isBetweenL(const OverlayKeyGMP& keyA, const OverlayKeyGMP& keyB) const;

	/**
	 * Returns true, if this key is element of the interval [keyA, keyB]
	 * on the ring.
	 *
	 * @param keyA The left border of the interval
	 * @param keyB The right border of the interval
	 * @return True, if the key is element of the interval [keyA, keyB]
	 */
	bool isBetweenLR(const OverlayKeyGMP& keyA, const OverlayKeyGMP& keyB) const;

	//-------------------------------------------------------------------------
	// Raw read access, required for crypto hash functions
	//-------------------------------------------------------------------------
	
	const uint8_t* getRawKey() const { return  reinterpret_cast<const uint8_t*>(key); }
	
	//-------------------------------------------------------------------------
	// static methods
	//-------------------------------------------------------------------------

	/**
	 * Set the length of an OverlayKeyGMP
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
	static OverlayKeyGMP random(cRNG* rng);

	/**
	 * Returns the maximum key, i.e. a key filled with bit 1
	 *
	 * @return The maximum key, i.e. a key filled with bit 1
	 */
	static OverlayKeyGMP getMax();

	/**
	 * Returns a key 2^exponent.
	 *
	 * @param exponent The exponent.
	 * @return Key=2^exponent.
	 */
	static OverlayKeyGMP pow2(uint32_t exponent);

	/**
	 * Returns the bit length of the commen prefix of the two keys.
	 */
	static int getCommonPrefixLength(const OverlayKeyGMP& a, const OverlayKeyGMP& b);

private:
	// private constants

	static constexpr uint32_t MAX_KEYLENGTH = 128; /**< maximum length of the key */
	static uint32_t keyLength; /**< actual length of the key */
	static uint32_t aSize; /**< number of needed machine words to hold the key*/
	static mp_limb_t GMP_MSB_MASK; /**< bits to fill up if key does not
	exactly fit in one or more machine words */

	// private fields
	bool isUnspec; /**< is this->key unspecified? */

	mp_limb_t key[MAX_KEYLENGTH / (8*sizeof(mp_limb_t)) +
	          (MAX_KEYLENGTH % (8*sizeof(mp_limb_t))!=0 ? 1 : 0)];
	/**< the overlay key this object represents */


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
 * An auxiliary class for single bits in OverlayKeyGMP
 *
 * Allows statements like "key[n] = true"
 */
class OverlayKeyGMPBit
{
public:

	OverlayKeyGMPBit(bool value, uint32_t pos, OverlayKeyGMP* key)
	    : bit(value), pos(pos), key(key)
	{};

	/** Converts to a boolean value */
	inline operator bool()
	{
	    return bit;
	};

	inline OverlayKeyGMPBit& operator=(const OverlayKeyGMPBit& value)
	{
	    key->setBit(pos, value.bit);
	    return *this;
	};

	/** Sets the corresponding bit to a boolean value
	* @param value value to set to
	*/
	inline OverlayKeyGMPBit& operator=(bool value)
	{
	    key->setBit(pos, value);
	    return *this;
	};

	inline OverlayKeyGMPBit& operator^=(bool value)
	{
	    key->setBit(pos, (*key)[pos] ^ value);
	    return *this;
	};

private:

	bool bit;
	uint32_t pos;
	OverlayKeyGMP* key;
};


// default construction: create an unspecified node key
inline
OverlayKeyGMP::OverlayKeyGMP()
{
	isUnspec = true;
}


// create a key out of a normal integer
inline
OverlayKeyGMP::OverlayKeyGMP(uint32_t num)
{
	clear();
	key[0] = num;
	trim();
}



// copy constructor
inline
OverlayKeyGMP::OverlayKeyGMP(const OverlayKeyGMP& rhs) : cObject(rhs), isUnspec(rhs.isUnspec)
{
	if (this != &rhs) // do not copy into itself
		memcpy( &key, &rhs.key, sizeof(key) );
}



// returns the length in bits
inline
uint32_t OverlayKeyGMP::getLength()
{
	return OverlayKeyGMP::keyLength;
}


inline
bool OverlayKeyGMP::isUnspecified() const
{
	return isUnspec;
}


//--------------------------------------------------------------------
// operators
//--------------------------------------------------------------------

// assignment operator
inline
OverlayKeyGMP& OverlayKeyGMP::operator=(const OverlayKeyGMP& rhs)
{
	if (this == &rhs) // exclude assignment to itself, mempcy doesn't work then
		return *this;

	cObject::operator=(rhs);
	isUnspec = rhs.isUnspec;
	memcpy( &key, &rhs.key, sizeof(key) );
	return *this;
}


// sub one prefix operator
inline
OverlayKeyGMP& OverlayKeyGMP::operator--()
{
	return (*this -= ONE);
}

// sub one postfix operator
inline
OverlayKeyGMP OverlayKeyGMP::operator--(int)
{
	OverlayKeyGMP clone = *this;
	*this -= ONE;
	return clone;
}


// add one prefix operator
inline
OverlayKeyGMP& OverlayKeyGMP::operator++()
{
	return (*this += ONE);
}

// sub one postfix operator
inline
OverlayKeyGMP OverlayKeyGMP::operator++(int)
{
	OverlayKeyGMP clone(*this);
	*this += ONE;
	return clone;
}


// add assign operator
inline
OverlayKeyGMP& OverlayKeyGMP::operator+=( const OverlayKeyGMP& rhs )
{
	mpn_add_n((mp_limb_t*)key, (mp_limb_t*)key, (mp_limb_t*)rhs.key, aSize);
	trim();
	isUnspec = false;
	return *this;
}


// sub assign operator
inline
OverlayKeyGMP& OverlayKeyGMP::operator-=( const OverlayKeyGMP& rhs )
{
	mpn_sub_n((mp_limb_t*)key, (mp_limb_t*)key, (mp_limb_t*)rhs.key, aSize);
	trim();
	isUnspec = false;
	return *this;
}


// add operator
inline
OverlayKeyGMP OverlayKeyGMP::operator+(const OverlayKeyGMP& rhs) const
{
	OverlayKeyGMP result(*this);
	result += rhs;
	return result;
}


// sub operator
inline
OverlayKeyGMP OverlayKeyGMP::operator-(const OverlayKeyGMP& rhs) const
{
	OverlayKeyGMP result(*this);
	result -= rhs;
	return result;
}

// compare operators
inline
bool OverlayKeyGMP::operator<(const OverlayKeyGMP& compKey) const
{
	return compareTo(compKey) < 0;
}

inline
bool OverlayKeyGMP::operator>(const OverlayKeyGMP& compKey) const
{
	return compareTo(compKey) > 0;
}

inline
bool OverlayKeyGMP::operator<=(const OverlayKeyGMP& compKey) const
{
	return compareTo(compKey) <=0;
}

inline
bool OverlayKeyGMP::operator>=(const OverlayKeyGMP& compKey) const
{
	return compareTo(compKey) >=0;
}

inline
bool OverlayKeyGMP::operator==(const OverlayKeyGMP& compKey) const
{
	return compareTo(compKey) ==0;
}

inline
bool OverlayKeyGMP::operator!=(const OverlayKeyGMP& compKey) const
{
	return compareTo(compKey) !=0;
}

// bitwise xor
inline
OverlayKeyGMP OverlayKeyGMP::operator^ (const OverlayKeyGMP& rhs) const
{
	OverlayKeyGMP result(*this);
	for (uint32_t i=0; i<aSize; i++) {
		result.key[i] ^= rhs.key[i];
	}
	return result;
}

// bitwise or
inline
OverlayKeyGMP OverlayKeyGMP::operator| (const OverlayKeyGMP& rhs) const
{
	OverlayKeyGMP result(*this);
	for (uint32_t i=0; i<aSize; i++) {
		result.key[i] |= rhs.key[i];
	}
	return result;
}

// bitwise and
inline
OverlayKeyGMP OverlayKeyGMP::operator& (const OverlayKeyGMP& rhs) const
{
	OverlayKeyGMP result(*this);
	for (uint32_t i=0; i<aSize; i++) {
		result.key[i] &= rhs.key[i];
	}
	return result;
}

// complement
inline
OverlayKeyGMP OverlayKeyGMP::operator~ () const
{
	OverlayKeyGMP result(*this);
	for (uint32_t i=0; i<aSize; i++) {
		result.key[i] = ~key[i];
	}
	result.trim();

	return result;
}


// get bit
inline
OverlayKeyGMPBit OverlayKeyGMP::operator[](uint32_t n)
{
	return OverlayKeyGMPBit(getBit(n), n, this);
}


inline
int OverlayKeyGMP::getCommonPrefixLength(const OverlayKeyGMP& a, const OverlayKeyGMP& b)
{
	return getLength() - (a ^ b).log_2();
}

//--------------------------------------------------------------------
// private methods (mostly inlines)
//--------------------------------------------------------------------

// trims uppermost (unused) bits of a key after each operation
inline void OverlayKeyGMP::trim()
{
	key[aSize-1] &= GMP_MSB_MASK;
}


// compares this key to any other
inline
int OverlayKeyGMP::compareTo( const OverlayKeyGMP& compKey ) const
{
	ASSERT (!compKey.isUnspec and !isUnspec);
	return mpn_cmp(key,compKey.key,aSize);
}


// sets key to zero and clears unspecified bit
inline void OverlayKeyGMP::clear()
{
	memset( key, 0, sizeof(key) );
	isUnspec = false;
}


// replacement function for mpn_random() using omnet's rng
inline void omnet_random_gmp(mp_limb_t *r1p, mp_size_t r1n, cRNG* rng)
{
	// fill in 32 bit chunks
	uint32_t* chunkPtr = (uint32_t*)r1p;

	for (uint32_t i=0; i < ((r1n*sizeof(mp_limb_t) + 3) / 4); i++) {
		chunkPtr[i] = rng->intRand(std::numeric_limits<uint32_t>::max());
	}
}

} // end namespace

namespace std {
	template<> class hash<routingsim::OverlayKeyGMP>
	{
	public:
		size_t operator()(const routingsim::OverlayKeyGMP& k) const;
	};


	inline
	std::size_t std::hash<routingsim::OverlayKeyGMP>::operator()(const routingsim::OverlayKeyGMP& k) const {
			using std::size_t;

			return k.hash();
	};

	/**
	 * Common stdc++ console output method
	 */
	inline
	std::ostream& operator<<(std::ostream& os, const routingsim::OverlayKeyGMP& c) {
		return (os << c.toString(16));
	}
}

#endif
