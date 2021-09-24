#ifndef __ROUTINGSIM_Stateseqno_H_
#define __ROUTINGSIM_Stateseqno_H_

#include <cstdint>
#include <iostream>

/**
 *
 * this class realizes a 16-bit sequence number
 * with wrap around and special comparison
 * (comparison only works within 15-bit window)
 *
 * 0xffff is an invalid seq number. it will be skipped over when incrementing.
 **/


class stateseqno_t {
private:	
	uint16_t seqnumber= 0u;
public:
	stateseqno_t() : seqnumber(0u) {}
	stateseqno_t(unsigned short no) : seqnumber(no) {}
	stateseqno_t& operator=(const stateseqno_t& other) { seqnumber= other.seqnumber; return *this; }
	// pre increment ++a
	stateseqno_t& operator++() { ++seqnumber; if (seqnumber == invalidSeqNo) { ++seqnumber; }; return *this; }
	// post increment a++
	stateseqno_t operator++(int) { seqnumber++; if (seqnumber == invalidSeqNo) { seqnumber++; }; return *this; }
	stateseqno_t& operator+=(const stateseqno_t& other) { seqnumber+= (uint16_t) other; return *this; }
	stateseqno_t operator+(const stateseqno_t& other) { return seqnumber + (uint16_t) other; }
	bool operator== (const stateseqno_t& other) const { return other.seqnumber == seqnumber; }
	// s < s' (modulo 2^16) when 0 < ((s' - s) MOD 2^16) < 32768
	bool operator< (const stateseqno_t& other) const { return  (seqnumber != other.seqnumber) and (uint16_t)(other.seqnumber - seqnumber) < 0x8000u;}
	bool operator> (const stateseqno_t& other) const { return  (seqnumber != other.seqnumber) and not((uint16_t)(other.seqnumber - seqnumber) < 0x8000u); }
	bool operator<= (const stateseqno_t& other) const { return operator<(other) or seqnumber == other.seqnumber; }
	bool operator>= (const stateseqno_t& other) const { return operator>(other) or seqnumber == other.seqnumber; }
	explicit operator uint16_t() const { return seqnumber; }

	std::ostream& operator<<(std::ostream& os) const { return os << seqnumber; }

	bool isInvalid() const { return seqnumber == invalidSeqNo; }
	static constexpr uint16_t invalidSeqNo= 0xffff;
};



inline
std::ostream& operator<<(std::ostream& os, stateseqno_t sseq) { return sseq.operator<<(os); }
#endif // __ROUTINGSIM_Stateseqno_H_
