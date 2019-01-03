#ifndef STATISTICSAMPLE_H_
#define STATISTICSAMPLE_H_

#include "Statistics.h"

namespace routingsim {

/// nodes that drop statistics should implement this interface
class StatisticSample {
	friend class Statistics;
protected:
	virtual void sampleStatistics( Statistics& stats ) = 0;
};

}  // namespace routingsim

#endif /* STATISTICSAMPLE_H_ */
