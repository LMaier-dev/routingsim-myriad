#ifndef __ROUTINGSIM_BASETYPES_H
#define __ROUTINGSIM_BASETYPES_H

#include <cmath>
#include <iostream>
#include <numbers>
#include <omnetpp.h>
#include <vector>

namespace routingsim {

#ifdef __CDT_PARSER__
#define foreach(a, b) a;
#else
#define foreach(a, b) BOOST_FOREACH (a, b)
#endif

/// identifier type
typedef unsigned long id_t;

/// node identifier definitions
using nodeid_t = id_t;
const nodeid_t undefined_node = ~0;
const nodeid_t neighbor_cast = ~0;

/// distance definition
typedef float distance_t;
const distance_t default_distance = 1;
const distance_t infinite_distance = ~0;

/// a simple vector_set class
template <class T> class vector_set : public std::vector<T> {
public:
  typedef std::vector<T> super;

public:
  vector_set() : super() {}

  vector_set(const super &set) : super(set) {}

  /// insert a value to the set
  typename super::iterator insert(const T &value) {
    for (typename super::iterator i = super::begin(); i != super::end(); i++)
      if (*i == value)
        return i;
    super::push_back(value);
    return super::end() - 1;
  }

  /// insert a another set
  void insert(const vector_set<T> &set) {
    for (typename super::const_iterator i = set.begin(); i != set.end(); i++)
      insert(*i);
  }

  /// erase a value from the set
  void erase(const T &value) {
    for (typename super::iterator i = super::begin(); i != super::end(); i++)
      if (*i == value) {
        i = super::erase(i);
        break;
      }
  }

  /// erase a another set
  void erase(const vector_set<T> &set) {
    for (typename super::const_iterator i = set.begin(); i != set.end(); i++)
      erase(*i);
  }

  /// returns the number of elements
  size_t count(const T &value) const {
    size_t c = 0;
    for (typename super::const_iterator i = super::begin(); i != super::end();
         i++)
      if (*i == value)
        c++;
    return c;
  }
};

template <class T>
std::ostream &operator<<(std::ostream &os, const vector_set<T> &vs) {
  for (typename vector_set<T>::const_iterator i = vs.begin(); i != vs.end();
       i++) {
    if (i != vs.begin())
      os << ",";
    os << *i;
  }
  return os;
}

/// hop definition
class hop_t {
public:
  hop_t()
      : from(undefined_node), to(undefined_node), distance(default_distance) {}

  hop_t(const hop_t &hop)
      : from(hop.from), to(hop.to), distance(hop.distance) {}

  hop_t(nodeid_t from, nodeid_t to, distance_t distance = default_distance)
      : from(from), to(to), distance(distance) {}

  /// reverse hop
  hop_t reverse() const {
    hop_t rev(*this);
    nodeid_t temp = rev.to;
    rev.to = rev.from;
    rev.from = temp;
    return rev;
  }

  /// compare hop, distance is not compared!!
  bool operator==(const hop_t &hop) const {
    return hop.from == from && hop.to == to;
  }

  /// compare hop
  bool operator!=(const hop_t &hop) const { return !(operator==)(hop); }

  nodeid_t from, to;
  distance_t distance;
};

std::ostream &operator<<(std::ostream &os, const hop_t &h);

/// a neighbor record of a node
class neighbor_t {
public:
  nodeid_t id;
  simtime_t activity;
  simtime_t lastsent;

  cGate *out;
};

class Coord2D {
public:
  static const Coord2D ZERO;
  static const Coord2D ONE;
  double x;
  double y;

  Coord2D() : x(0.0), y(0.0){};

  Coord2D(double x, double y) {
    this->x = x;
    this->y = y;
  };

  Coord2D operator+(Coord2D &a) const { return {x + a.x, y + a.y}; }

  Coord2D operator+(double f) const { return {x + f, y + f}; }

  Coord2D operator+=(Coord2D a) { return {x += a.x, y += a.y}; }

  Coord2D operator-(Coord2D &rhs) const { return {x - rhs.x, y - rhs.y}; }

  Coord2D operator-=(Coord2D a) { return {x -= a.x, y -= a.y}; }

  Coord2D operator*(Coord2D &rhs) const { return {x * rhs.x, y * rhs.y}; }

  Coord2D operator*=(Coord2D a) { return {x *= a.x, y *= a.y}; }

  Coord2D operator*(double d) const { return {x * d, y * d}; }

  Coord2D operator/(double d) const {
    if (d == 0) {
      throw cRuntimeError("Dividing by zero");
    }
    return {x / d, y / d};
  }

  bool operator==(Coord2D const &rhs) const {
    return this->x == rhs.x && this->y == rhs.y;
  }

  bool operator!=(Coord2D const &rhs) const {
    return this->x != rhs.x || this->y != rhs.y;
  }

  double distance(Coord2D rhs) const {
    return sqrt((x - rhs.x) * (x - rhs.x) + (y - rhs.y) * (y - rhs.y));
  }

  double distance() const { return sqrt(x*x + y*y); }

  Coord2D get_normalized() const {
    if (this->distance() == 0)
      return *this;
    return *this / this->distance();
  }

  [[nodiscard]] std::string toString() const {
    std::ostringstream oss;
    oss << "(" << x << ", " << y << ")";
    return oss.str();
  }
};

class Rad2D {
private:
  double radius;

public:
  Rad2D() { setRadius(0); };
  Rad2D(double d) { setRadius(d); };

  static const Rad2D ZERO;
  void setRadius(double r) { radius = fmod(r, 360); }

  [[nodiscard]] double getRadius() const { return radius; }

  void add_deg(double d) { setRadius(radius + d); }

  void setFromVector(Coord2D vector) {
    Coord2D vector_normalized = vector.get_normalized();
    radius = atan2(vector_normalized.y, vector_normalized.x);
  }

  Coord2D toVector() const {
    return Coord2D(sin(radius * 180/std::numbers::pi), cos(radius * 180/std::numbers::pi));
  }

  Rad2D operator=(double d) {
    setRadius(d);
    return *this;
  }
};

template <typename T>
class Unit {
protected:
  double value;

public:
  explicit Unit(const double d) : value(d) {}

  T operator+(const Unit &rhs) const { return T{value + rhs.value}; }
  T operator-(const Unit &rhs) const { return T{value - rhs.value}; }
  T operator*(const Unit &rhs) const { return T{value * rhs.value}; }
  T operator/(const Unit &rhs) const { return T{value / rhs.value}; }

  [[nodiscard]] double getValue() const { return value; }
};

class DB {
protected:
  double value;

public:
  explicit DB(const double d = 0) : value(d) {}
  DB operator+(const DB &rhs) const { return DB(getValue() + rhs.getValue()); }
  DB operator-(const DB &rhs) const { return DB(getValue() - rhs.getValue()); }
  DB operator*(const DB &rhs) const { return DB{value + rhs.value}; }
  DB operator/(const DB &rhs) const { return DB{value - rhs.value}; }

  [[nodiscard]] double getValue() const {
    return value;
  }

  [[nodiscard]] double toDouble() const {
    return pow(10, value/10);
  }

  void fromDouble(const double d) {
    value = 10 * log10(d);
  }
};

class MHz: public Unit<MHz> {
public:
  explicit MHz(const double d) : Unit<MHz>(d) {};
  [[nodiscard]] double toHz() const { return value * 1000 * 1000; }
};

} // namespace routingsim

#endif /* __ROUTINGSIM_BASETYPES_H */
