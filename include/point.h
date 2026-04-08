#ifndef POINT_H_
#define POINT_H_

class Vector;

class Point{
public:

    double _x,_y;

    Point():_x(0),_y(0){};
    Point(double x,double y):_x(x),_y(y){};
    Point(const Point& p):_x(p._x),_y(p._y){};

    Point &operator=(Point const &p) {
        if (this == &p)
            return *this;
        else {
            this->_x = p._x;
            this->_y = p._y;
            return *this;
        }
    }
    Point operator+(const Vector &v) const ;
    Point operator-(const Vector &v) const ;
    Vector operator-(const Point &p) const ;
    friend Point operator+(const Vector &v, const Point &p);

    void setP(double x, double y);

    void resetP();

};

#endif
