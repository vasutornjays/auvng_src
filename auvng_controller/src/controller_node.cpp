#include <iostream>
#include "pid.h"

using namespace std;

int main () {
    Pid_object pid_1 (3,4);
    Pid_object pid_2;
    cout << "area: " << pid_1.area() << endl;
    cout << "area: " << pid_2.area() << endl;
    return 0;
}