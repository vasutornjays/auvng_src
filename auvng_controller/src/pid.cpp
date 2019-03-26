#include <iostream>
#include "pid.h"

using namespace std;

Pid_object::Pid_object () {
    width = 5;
    height = 5;
}

Pid_object::Pid_object (int x, int y) {
    width = x;
    height = y;
}