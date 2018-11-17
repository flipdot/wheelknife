#include "../src/average.h"
#include <iostream>


using namespace std;

int main(void) {
    MovingAverage ma = MovingAverage(3);
    for(int i = 0; i < 10; i++)
        cout << ma.update(i) << endl;
}