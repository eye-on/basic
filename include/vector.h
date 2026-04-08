#ifndef VECTOR_H_
#define VECTOR_H_

#include "point.h"
#include <cmath>

class Vector{
public:

    double _x,_y;

    Vector():_x(0),_y(0){};
    Vector(double x,double y):_x(x),_y(y){};
    Vector(const Vector& p):_x(p._x),_y(p._y){};
    Vector(const Point& s,const Point& e){
        _x=e._x-s._x;
        _y=e._y-s._y;
    }

    Vector &operator=(Vector const &v) {
        if (this == &v)
            return *this;
        else {
            this->_x = v._x;
            this->_y = v._y;
            return *this;
        }
    }
 
    Vector operator+(Vector const &v) const { return Vector(_x + v._x, _y + v._y); }
    Vector operator-(Vector const &v) const { return Vector(_x - v._x, _y - v._y); }
    double operator*(Vector const &v) const { return _x * v._x + _y * v._y; }
    Vector operator*(double a) const { return Vector(_x * a, _y * a); }
    Vector operator/(double a) const { return Vector(_x / a, _y / a); }
    friend Vector operator*(double a, Vector v) { return Vector(v._x * a, v._y * a); }
    double mod() const { return std::sqrt(_x * _x + _y * _y); }

    void setV(double x, double y);

    void resetV();

};

#endif
