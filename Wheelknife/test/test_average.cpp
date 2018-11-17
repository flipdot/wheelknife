#include "../src/average.h"
#include <iostream>


using namespace std;

void test_ma(){
    MovingAverage ma = MovingAverage(3);
    cout << "Testing Moving Average: ";
    for(int i = 0; i < 10; i++)
        cout << ma.update(i) << " ";
    cout << endl;
}

void test_NoiseFilter() {
    NoiseFilter fi = NoiseFilter(5);
    cout << "Testing Noise Filter: ";
    for(int i = 0; i < 10; i++)
        cout << fi.update(i) << " ";
    cout << endl;
}

void test_MaFilter() {
    MaFilter mf = MaFilter(2, 5);
    cout << "Testing MaFilter: ";
    for(int i = 0; i < 10; i++)
        cout << mf.update(i) << " ";
    cout << endl;
}

int main(void) {
    test_ma();
    test_NoiseFilter();
    test_MaFilter();
}