#ifndef _MOVING_AVERAGE_H
#define _MOVING_AVERAGE_H

#include <vector>
#include <iostream>
#include <cmath>

class MovingAverage {
// Simple Moving Average, which returns the average of the last _length_ buffer elements.
// _buffer_ is a circular buffer.
private:
    std::vector<double> buffer;
    int m_length;

public:
	MovingAverage(int length) : m_length(length) {
        buffer.clear();
	}

    double update(double x) {
        double sum = 0;
        if( buffer.size() >= m_length )
            buffer.erase(buffer.begin());
        buffer.push_back(x);
        for(double &b : buffer) {
            sum += b;
        }
        return sum/buffer.size();
    }
};

class NoiseFilter {
// Simple Noise Filter, which returns the same value, if it does not differ 
// more than diff_val from the last value
private:
    int m_diff_val;
    double current_value;

public:
	NoiseFilter(double diff_val) : m_diff_val(diff_val) {
        current_value = 0;
	}

    double update(double x) {
        if( abs(x-current_value) > m_diff_val )
            current_value = x;
        return current_value;
    }
};

class MaFilter {
private:
    MovingAverage ma;
    NoiseFilter nf;
public:
    MaFilter(int ma_length, double filter_diff) : ma(MovingAverage(ma_length)), nf(NoiseFilter(filter_diff)) {}
    double update(double x) {
        return nf.update(ma.update(x));
    }
};

#endif //_MOVING_AVERAGE_H