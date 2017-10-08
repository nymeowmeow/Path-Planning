#ifndef _JMT_H_
#define _JMT_H_

#include <vector>

std::vector<double> getMinJerk(const std::vector<double>& start,
			const std::vector<double>& end, double T);
double evaluate(const std::vector<double>& coeffs, double x); 
#endif //jmt.h
