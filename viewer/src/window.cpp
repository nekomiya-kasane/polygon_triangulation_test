﻿#include "easyx.h"
#include "graphics.h"

#include "viewable.h"

#include <chrono>

int main()
{
  initgraph(1024, 768);
  BeginBatchDraw();

  setlinecolor(WHITE);
  setfillcolor(RED);
  settextcolor(YELLOW);
  settextstyle(15, 0, "Victor Mono");

  char rate[] = " -1 loops/s";

  int i      = 1;
  auto start = std::chrono::steady_clock::now(), end = std::chrono::steady_clock::now();

  Vec2Set points = {{-1, 1}, {0, 2}, {1, 0}, {0, -2}};

  ViewableTriangulator tri;
  tri.config.useGivenSeed = true;
  tri.config.seed         = 1;

  tri.AddPolygon(points, true);
  tri.Build();
  tri.Triangulate();

  while (true)
  {
    ++i;
    cleardevice();

    if (i % 10 == 0)
    {
      i       = 1;
      end     = std::chrono::steady_clock::now();
      auto ms = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
      start   = end;

      wsprintf(rate, "%3d loops/s", int(1. / (ms * 1.0e-7)));
    }

    {
      tri.Draw({512, 384}, {100, -100});
      // tri._needRedraw = false;
    }

    outtextxy(0, 0, rate);
    FlushBatchDraw();

    Sleep(10);
  }

  EndBatchDraw();
  closegraph();
}