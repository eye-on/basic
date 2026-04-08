#include "point.h"
#include "vector.h"

Point Point::operator+(const Vector &v) const { return Point(_x + v._x, _y + v._y); }
Point Point::operator-(const Vector &v) const { return Point(_x - v._x, _y - v._y); }
Vector Point::operator-(const Point &p) const { return Vector(_x - p._x, _y - p._y); }
inline Point operator+(const Vector &v, const Point &p) { return Point(p._x + v._x, p._y + v._y); }

void Point::setP(double x, double y) {
        _x = x;
        _y = y;
}

void Point::resetP() {
        _x = 0;
        _y = 0;
}
