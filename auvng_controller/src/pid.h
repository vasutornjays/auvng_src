#ifndef PID_H
#define PID_H


class Pid_object {

        int width, height;

    public:
        Pid_object ();
        Pid_object (int x, int y);
        int area() {return width*height;}      
};

#endif