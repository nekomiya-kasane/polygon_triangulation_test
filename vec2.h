#pragma once

#ifndef DUMMY_MATH_H
#  define DUMMY_MATH_H

#  define _CRT_FUNCTIONS_REQUIRED 1

#  define SEIDEL_INVALID_INDEX (size_t)(-1)

#  include <cmath>
#  include <list>
#  include <stack>
#  include <vector>

constexpr double MAX_DOUBLE = std::numeric_limits<double>::max();

struct Vec2;
typedef std::vector<Vec2> Vec2Set;

struct Vec2
{
  Vec2(double iX, double iY);
  Vec2();

  friend Vec2 abs(const Vec2 &iVec);
  friend Vec2 operator*(double iNum, const Vec2 &iVec);

  Vec2 operator+(const Vec2 &iVec) const;
  Vec2 operator-(const Vec2 &iVec) const;
  Vec2 &operator+=(const Vec2 &iVec);
  Vec2 &operator-=(const Vec2 &iVec);
  double operator^(const Vec2 &iVec) const;
  double operator*(const Vec2 &iVec) const;
  Vec2 operator*(double iNumber) const;
  Vec2 &operator*=(double iNumber);
  Vec2 operator/(double iNumber) const;
  Vec2 operator-() const;
  bool operator<(const Vec2 &iVec) const;
  friend bool operator==(const Vec2 &iVec1, const Vec2 &iVec2);
  bool EqualTo(const Vec2 &iOther, double iTol) const;
  double NormSq() const;
  double Norm() const;
  Vec2 GetNormalized() const;

  static const Vec2 rightTopMost, leftBottomMost;

  double x;
  double y;
};
#endif