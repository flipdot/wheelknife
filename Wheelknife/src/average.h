#ifndef _MOVING_AVERAGE_H
#define _MOVING_AVERAGE_H

#include <vector>
#include <iostream>
#include <cmath>
using namespace std;

class MovingAverage {
private:
	double val;
    vector<double> buffer;
    int m_length;

public:
	MovingAverage(int length) {
        m_length = length;
        buffer.clear();
	}

    double update(double x) {
        double sum = 0;
        if( buffer.size() >= m_length )
            buffer.erase(buffer.begin());
        buffer.push_back(x);
        for(double &b : buffer)
            sum += b;
        return sum/buffer.size();
    }
};

#endif //_MOVING_AVERAGE_H