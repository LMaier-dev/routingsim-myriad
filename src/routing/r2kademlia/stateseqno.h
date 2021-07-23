#ifndef __ROUTINGSIM_Stateseqno_H_
#define __ROUTINGSIM_Stateseqno_H_

#include <cstdint>

class stateseqno_t {
private:	
	uint16_t seqnumber= 0u;
public:
	stateseqno_t() : seqnumber(0u) {}
	stateseqno_t(unsigned short no) : seqnumber(no) {}
	stateseqno_t& operator=(const stateseqno_t& other) { seqnumber= other.seqnumber; return *this; }
	// pre increment ++a
	stateseqno_t& operator++() { ++seqnumber; return *this; }
	// post increment a++
	stateseqno_t operator++(int) { seqnumber++; return *this; }
	bool operator== (const stateseqno_t& other) const { return other.seqnumber == seqnumber; }
	// s < s' (modulo 2^16) when 0 < ((s' - s) MOD 2^16) < 32768
	bool operator< (const stateseqno_t& other) const { return  (uint16_t)(other.seqnumber - seqnumber) < 0x8000u;}
	bool operator> (const stateseqno_t& other) const { return  (seqnumber != other.seqnumber) and not((uint16_t)(other.seqnumber - seqnumber) < 0x8000u); }
	bool operator<= (const stateseqno_t& other) const { return seqnumber < other.seqnumber or seqnumber == other.seqnumber; }
	bool operator>= (const stateseqno_t& other) const { return seqnumber > other.seqnumber or seqnumber == other.seqnumber; }
        operator uint16_t() const { return seqnumber; }
};

#endif // __ROUTINGSIM_Stateseqno_H_
