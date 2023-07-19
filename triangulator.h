#pragma once

#include "trapezoidMapP.h"

using Triangle  = Vec2[3];
using Triangles = std::vector<Triangle>;

class TrapezoidMapPT : public TrapezoidMapP
{
public:
  void Diagnolize();
  void ExtractCycles();

  Triangles EarClipping();
};