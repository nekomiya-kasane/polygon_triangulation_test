#include "polygon_generator.h"
#include "viewable.h"

#include <chrono>
#include <cstring>

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
#  pragma warning(push, 0)
#  include "raylib.h"

#  include "raymath.h"
#  include "rlgl.h"

#  define RAYGUI_IMPLEMENTATION
#  include "raygui.h"  // Required for GUI controls
#  pragma warning(pop)

struct States
{
  bool drawGrid          = false;
  bool generateRandom    = 0;
  int generateRandomSize = 8;
} states;

bool ResolveConfig(Triangulator &tri)
{
  States oldStates               = states;
  Triangulator::Config oldConfig = tri.config;
  tri.config.printData   = GuiCheckBox(Rectangle{10, 40, 20, 20}, " Print data", tri.config.printData);
  tri.config.printCase   = GuiCheckBox(Rectangle{10, 70, 20, 20}, " Print case", tri.config.printCase);
  states.drawGrid        = GuiCheckBox(Rectangle{10, 100, 20, 20}, " Draw grids", states.drawGrid);
  tri.config.incremental = GuiCheckBox(Rectangle{10, 130, 20, 20}, " Incremental", tri.config.incremental);
  if (tri.config.incremental)
  {
    tri.config.assignDepth = tri.config.generateMountains = tri.config.triangulation = false;
    tri.config.maxSegment = GuiSliderBar(Rectangle{10, 160, 100, 20}, NULL, " Segments",
                                         tri.config.maxSegment, 0, tri._segments.Size());
    DrawText(std::to_string(tri.config.maxSegment).c_str(), 175, 133, 14, GRAY);
  }
  else
  {
    tri.config.assignDepth = GuiCheckBox(Rectangle{10, 160, 20, 20}, " Depth", tri.config.assignDepth);
    if (tri.config.assignDepth)
    {
      tri.config.generateMountains =
          GuiCheckBox(Rectangle{10, 190, 20, 20}, " Mountains", tri.config.generateMountains);
      if (tri.config.generateMountains)
      {
        tri.config.triangulation =
            GuiCheckBox(Rectangle{10, 220, 20, 20}, " Triangles", tri.config.triangulation);
      }
      else
      {
        tri.config.triangulation = false;
      }
    }
    else
    {
      tri.config.generateMountains = tri.config.triangulation = false;
    }
  }

  if (std::memcmp(&states, &oldStates, sizeof(States)) ||
      std::memcmp(&oldConfig, &tri.config, sizeof(Triangulator::Config)))
    return true;
  return false;
}

int main()
{
  // Vec2Set points = {{36, 5}, {62, 0}, {94, 66}, {95, 72}, {100, 92}, {73, 76}, {26, 84}, {21, 100}, {0,
  // 40}};
  Vec2Set points = {{2, 661},   {87, 488},  {113, 997}, {709, 935}, {770, 831},
                    {392, 181}, {273, 256}, {164, 46},  {13, 299}};

  // Vec2Set points = {{183, 149}, {562, 966}, {819, 892}, {547, 138},
  //                   {524, 752}, {480, 54},  {327, 91},  {276, 168}};

  ViewableTriangulator tri;
  tri.config.useGivenSeed = true;
  tri.config.seed         = 1;
  tri.config.incremental  = true;

  // visualization
  const int screenWidth  = 1920;
  const int screenHeight = 1080;

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

  Vec2 lt, rb;
  bool first = true;
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

    Vector2 mousePos = GetMousePosition();

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

        states.generateRandom = GuiCheckBox(
            Rectangle{10, screenHeight - 30, 20, 20},
            (" Generate random " + std::to_string(states.generateRandomSize)).c_str(), states.generateRandom);
        states.generateRandomSize = GuiSliderBar(Rectangle{10, screenHeight - 60, 100, 20}, NULL,
                                                 " Point count", states.generateRandomSize, 0, 20);
        DrawTextEx(font, (std::to_string(int(mousePos.x)) + ", " + std::to_string(int(mousePos.y))).c_str(),
                   Vector2{10, screenHeight - 90}, 20, 0, GRAY);

        if (states.generateRandom)
        {
          points = PolygonGenerator::GenerateRandomPolygon(states.generateRandomSize);
        }
        if (first || states.generateRandom)
        {
          tri.Reset();
          tri.AddPolygon(points, true);
          tri.GetBoundingBox(lt, rb);
        }

        if (ResolveConfig(tri) || first || states.generateRandom)
        {
          tri.ClearCache();
          tri.Build();
          tri.Triangulate();

          first                 = false;
          states.generateRandom = false;
        }

        // draw the shape
        tri.SetOrigin({screenWidth / 2, screenHeight / 2});
        tri.SetBox({screenWidth, screenHeight});
        tri.SetFocus(Vec2{mousePos.x, mousePos.y});

        Vec2 ori = (lt + rb) / 2, factor{0.8 * screenWidth / (rb - lt).x, 0.8 * screenHeight / (rb - lt).y};

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