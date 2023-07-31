#include "viewable.h"

#include <chrono>

#ifdef USE_EASYX
#  include "easyx.h"
#  include "graphics.h"

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

  // Vec2Set points = {{-1, 1}, {0, 2}, {1, 0}, {0, -2}};
  // Vec2Set points = {{53, 131},  {122, 238}, {204, 167}, {239, 269}, {93, 353},
  //                  {247, 328}, {326, 369}, {296, 215}, {222, 46},  {157, 119}};

  Vec2Set points = {{0, 2}, {0, 1}, {-1, 0}, {4, 0}, {4, 1}};

  ViewableTriangulator tri;
  tri.config.useGivenSeed = true;
  tri.config.seed         = 1;

  tri.AddPolygon(points, true);
  tri.Build();
  tri.Triangulate();

  tri.SetOrigin({512, 384});

  Vec2 lt, rb;
  tri.GetBoundingBox(lt, rb);
  Vec2 ori = (lt + rb) / 2, factor{0.8 * 1024. / (rb - lt).x, 0.8 * 768. / (rb - lt).y};

  while (true)
  {
    ++i;
    cleardevice();

    // todo: interactive

    if (i % 10 == 0)
    {
      i       = 1;
      end     = std::chrono::steady_clock::now();
      auto ms = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
      start   = end;

      wsprintf(rate, "%3d loops/s", int(1. / (ms * 1.0e-7)));
    }

    {
      tri.Draw(ori, factor);
      // tri._needRedraw = false;
    }

    outtextxy(0, 0, rate);
    FlushBatchDraw();

    Sleep(10);
  }

  EndBatchDraw();
  closegraph();
}
#else
#  include "raylib.h"

#  include "raymath.h"
#  include "rlgl.h"

#  define RAYGUI_IMPLEMENTATION
#  include "raygui.h"  // Required for GUI controls

struct
{
  bool drawGrid = false;
} states;

int main()
{
  Vec2Set points = {{0, 2}, {0, 1}, {-1, 0}, {4, 0}, {4, 1}};

  ViewableTriangulator tri;
  tri.config.useGivenSeed = true;
  tri.config.seed         = 1;

  tri.AddPolygon(points, true);
  tri.Build();
  tri.Triangulate();

  Vec2 lt, rb;
  tri.GetBoundingBox(lt, rb);

  // visualization
  const int screenWidth  = 1024;
  const int screenHeight = 768;

  InitWindow(screenWidth, screenHeight, "Seidel Algorithm Visualizer");
  SetWindowMinSize(screenWidth, screenHeight);

  Camera2D camera = {0};
  camera.zoom     = 1.0f;
  SetTargetFPS(144);

  Font font = LoadFontEx("assets/victor_mono.ttf", 96, 0, 0);
  if (!IsFontReady(font))
  {
    TRACE("Font not ready.");
  }
  tri.drawingConfig.font = font;

  while (!WindowShouldClose())  // Escape or exit button clicked
  {
    // move canvas
    if (IsMouseButtonDown(MOUSE_BUTTON_MIDDLE))
    {
      Vector2 delta = GetMouseDelta();
      delta         = Vector2Scale(delta, -1.0f / camera.zoom);

      camera.target = Vector2Add(camera.target, delta);
    }

    // zoom
    float wheel = GetMouseWheelMove();
    if (wheel != 0)
    {
      Vector2 mouseWorldPos     = GetScreenToWorld2D(GetMousePosition(), camera);
      camera.offset             = GetMousePosition();
      camera.target             = mouseWorldPos;
      const float zoomIncrement = 0.125f;
      camera.zoom += (wheel * zoomIncrement);
      if (camera.zoom < zoomIncrement)
        camera.zoom = zoomIncrement;
    }

    // draw
    BeginDrawing();
    {
      ClearBackground(BLACK);

      BeginMode2D(camera);
      {
        // draw grid
        if (states.drawGrid)
        {
          rlPushMatrix();
          {
            rlTranslatef(0, 25 * 50, 0);
            rlRotatef(90, 1, 0, 0);
            DrawGrid(100, 50);
          }
          rlPopMatrix();
        }

        states.drawGrid = GuiCheckBox(Rectangle{10, 40, 20, 20}, " Draw grids", states.drawGrid);

        // draw the shape
        tri.SetOrigin({512, 384});

        Vec2 ori = (lt + rb) / 2, factor{0.8 * 1024. / (rb - lt).x, 0.8 * 768. / (rb - lt).y};

        tri.Draw(ori, factor);

        // draw rendering info
        DrawFPS(10, 10);
      }
      EndMode2D();
    }
    EndDrawing();
  }

  UnloadFont(font);

  CloseWindow();
}
#endif