#include <algorithm>
#include <list>
#include <random>
#include "vec2.h"

Vec2::Vec2(double iX, double iY) : x(iX), y(iY) {}

Vec2::Vec2() : x(0), y(0) {}

Vec2 abs(const Vec2 &iVec)
{
  return Vec2(std::abs(iVec.x), std::abs(iVec.y));
}

Vec2 Vec2::operator+(const Vec2 &iVec) const
{
  return Vec2(x + iVec.x, y + iVec.y);
}

Vec2 Vec2::operator-(const Vec2 &iVec) const
{
  return Vec2(x - iVec.x, y - iVec.y);
}

Vec2 &Vec2::operator+=(const Vec2 &iVec)
{
  x += iVec.x;
  y += iVec.y;
  return *this;
}

Vec2 &Vec2::operator-=(const Vec2 &iVec)
{
  x -= iVec.x;
  y -= iVec.y;
  return *this;
}

double Vec2::operator^(const Vec2 &iVec) const
{
  return x * iVec.y - y * iVec.x;
}

double Vec2::operator*(const Vec2 &iVec) const
{
  return x * iVec.x + y * iVec.y;
}

Vec2 Vec2::operator*(double iNumber) const
{
  return Vec2(x * iNumber, y * iNumber);
}

Vec2 Vec2::operator/(double iNumber) const
{
  return Vec2(x / iNumber, y / iNumber);
}

Vec2 Vec2::operator-() const
{
  return Vec2(-x, -y);
}

bool Vec2::operator<(const Vec2 &iVec) const
{
  if (x < iVec.x || (x == iVec.x && y < iVec.y))
    return true;
  return false;
}

bool operator==(const Vec2 &iVec1, const Vec2 &iVec2)
{
  return iVec1.x == iVec2.x && iVec1.y == iVec2.y;
}

Vec2 &Vec2::operator*=(double iNumber)
{
  x *= iNumber;
  y *= iNumber;
  return *this;
}

bool Vec2::EqualTo(const Vec2 &iOther, double iTol) const
{
  const double diffX = std::abs(x - iOther.x);
  const double diffY = std::abs(y - iOther.y);
  return std::max(diffX, diffY) < iTol;
}

double Vec2::NormSq() const
{
  return x * x + y * y;
}

double Vec2::Norm() const
{
  return std::sqrt(NormSq());
}

Vec2 Vec2::GetNormalized() const
{
  return *this / Norm();
}

Vec2 operator*(double iNum, const Vec2 &iVec)
{
  return iVec * iNum;
}

const Vec2 Vec2::leftBottomMost = {-MAX_DOUBLE, -MAX_DOUBLE};
const Vec2 Vec2::rightTopMost   = {MAX_DOUBLE, MAX_DOUBLE};
