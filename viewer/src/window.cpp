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

const int screenWidth    = 1920;
const int screenHeight   = 1080;
const int infoPanelHight = 200;

struct States
{
  bool drawGrid                = false;
  bool generateRandom          = false;
  bool randomSeed              = false;
  bool useSculpingGenerator    = true;
  bool allowHole               = true;
  bool mountTriDropdownPressed = false;
  int generateRandomSize       = 10;
} states;

/* GLOBALS */ Camera2D camera = {{0, 0}, {0, 0}, 0, 1.0f};
/* GLOBALS */ Font font;
/* GLOBALS */
/* GLOBALS */ ViewableTriangulator tri;
/* GLOBALS */
/* GLOBALS */ std::vector<Vec2Set> points;
/* GLOBALS */ Vec2 lt, rb;
/* GLOBALS */ bool first = true;
/* GLOBALS */
/* GLOBALS */ std::string mountainTriangulationMethod =
    "EarClipping (Normal);EarClipping (Random);EarClipping (Sorted);Chimney (Normal);Chimney (Greedy)";

Vector2 RealPos(const Vector2 &pos)
{
  return GetScreenToWorld2D(pos, camera);
}

Rectangle RealRect(float x, float y, float w, float h)
{
  auto pos = GetScreenToWorld2D({x, y}, camera);
  return Rectangle{pos.x, pos.y, w / camera.zoom, h / camera.zoom};
}

void ResolveCameraInteraction(Camera2D &camera)
{
  // move canvas
  if (IsMouseButtonDown(MOUSE_BUTTON_MIDDLE))
  {
    Vector2 delta = GetMouseDelta();
    delta         = Vector2Scale(delta, -1.0f / camera.zoom);

    camera.target = Vector2Add(camera.target, delta);
  }

  // reset camera
  if (IsKeyReleased(KEY_SPACE))
  {
    camera.target = {0, 0};
    camera.zoom   = 1.0f;
    camera.offset = {0, 0};
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
}

void DrawMyGrid()
{
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
}

bool ResolveConfig(Triangulator &tri)
{
  States oldStates                     = states;
  Triangulator::Config oldConfig       = tri.config;
  Triangulator::ConfigTri oldConfigTri = tri.configTri;
  states.drawGrid        = GuiCheckBox(RealRect(10, 100, 20, 20), " Draw grids", states.drawGrid);

#  ifdef _DEBUG
  tri.config.printData   = GuiCheckBox(RealRect(10, 40, 20, 20), " Print data", tri.config.printData);
  tri.config.printCase   = GuiCheckBox(RealRect(10, 70, 20, 20), " Print case", tri.config.printCase);
  tri.config.incremental = GuiCheckBox(RealRect(10, 130, 20, 20), " Incremental", tri.config.incremental);
  if (tri.config.incremental)
  {
    tri.config.assignDepth = tri.config.generateMountains = tri.config.triangulation = false;
    tri.config.maxSegment = GuiSliderBar(RealRect(10, 160, 100, 20), NULL, " Segments", tri.config.maxSegment,
                                         0, tri._segments.Size());
    DrawText(std::to_string(tri.config.maxSegment).c_str(), 175, 133, 14, GRAY);
  }
  else
  {
    tri.config.assignDepth = GuiCheckBox(RealRect(10, 160, 20, 20), " Depth", tri.config.assignDepth);
    if (tri.config.assignDepth)
    {
      tri.config.generateMountains =
          GuiCheckBox(RealRect(10, 190, 20, 20), " Mountains", tri.config.generateMountains);
      if (tri.config.generateMountains)
      {
        tri.config.triangulation =
            GuiCheckBox(RealRect(10, 220, 20, 20), " Triangles", tri.config.triangulation);
        if (tri.config.triangulation)
        {
#  endif
          bool pressed =
              GuiDropdownBox(RealRect(10, 250, 200, 20), mountainTriangulationMethod.c_str(),
                             (int *)&tri.configTri.mountainResolutionMethod, states.mountTriDropdownPressed);
          if (pressed)
            states.mountTriDropdownPressed = !states.mountTriDropdownPressed;
#  ifdef _DEBUG
        }
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
#  endif

  if (std::memcmp(&states, &oldStates, sizeof(States)) ||
      std::memcmp(&oldConfig, &tri.config, sizeof(Triangulator::Config)) ||
      std::memcmp(&oldConfigTri, &tri.configTri, sizeof(Triangulator::ConfigTri)))
    return true;
  return false;
}

void ResolveGeneration()
{
  auto randomInfoText   = "[ ] Random Polygon " + std::to_string(states.generateRandomSize);
  states.randomSeed     = GuiLabelButton(RealRect(250, screenHeight - 30, 20, 20), "[ ] Random Seed ");
  states.generateRandom = GuiLabelButton(RealRect(10, screenHeight - 30, 20, 20), randomInfoText.c_str());
  states.useSculpingGenerator = GuiCheckBox(RealRect(10, screenHeight - 60, 20, 20),
                                            " Use sculpting generator ", states.useSculpingGenerator);
  states.allowHole = GuiCheckBox(RealRect(10, screenHeight - 90, 20, 20), " Allow hole ", states.allowHole);
  states.generateRandomSize = GuiSliderBar(RealRect(10, screenHeight - 120, 100, 20), NULL, " Point count",
                                           states.generateRandomSize, 0, 100);
}

//======================================================================================================================

void InitiateViewer()
{
  InitWindow(screenWidth, screenHeight + infoPanelHight, "Seidel Algorithm Visualizer");
  SetWindowMinSize(screenWidth, screenHeight + infoPanelHight);
  SetTargetFPS(144);

  font = LoadFontEx("assets/victor_mono.ttf", 96, 0, 0);
  if (!IsFontReady(font))
  {
    TRACE("Font not ready.");
  }
  tri.drawingConfig.font = font;

  // init
  {
    // GuiSetFont(font);
    GuiSetStyle(LABEL, TEXT_COLOR_NORMAL, 0xFFFFFFFF);
    GuiSetStyle(LABEL, BASE_COLOR_NORMAL, 0xFFFFFFFF);
    GuiSetStyle(LABEL, TEXT_COLOR_FOCUSED, 0xABDBE3FF);
    GuiSetStyle(LABEL, BASE_COLOR_FOCUSED, 0xABDBE3FF);
    GuiSetStyle(LABEL, TEXT_COLOR_PRESSED, 0x76B5C5FF);
    GuiSetStyle(LABEL, BASE_COLOR_PRESSED, 0x76B5C5FF);
  }
}

bool GeneratePoints()
{
  if (states.generateRandom)
  {
    points = states.useSculpingGenerator ? PolygonGenerator::GenerateRandomPolygonBySculpting(
                                               states.generateRandomSize, 0, 1000, 0, 1000, states.allowHole)
                                         : PolygonGenerator::GenerateRandomPolygon(states.generateRandomSize);
  }
  if (first || states.generateRandom)
  {
    tri.Reset();
    size_t i = 0;
    for (const auto &contour : points)
    {
      if (contour.size() < 3)
        continue;
#  ifdef _DEBUG
      if (tri.config.printData)
      {
        std::cout << i++ << " {";
        for (const auto &point : contour)
          std::cout << "{" << (int)point.x << ", " << (int)point.y << "}, ";
        std::cout << "}";
      }
#  endif
      tri.AddPolygon(contour, true);
    }
    if (!tri._vertices.Size())
      return false;
    tri.GetBoundingBox(lt, rb);
  }
  if (states.randomSeed)
  {
    tri.config.seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
  }
  return true;
}

int main()
{
  tri.config.checkIntersection = false;
  //  points = {
  //      {{192, 470},  {280, 316},  {450, 383},  {399, 309},  {289, 152},  {334, 138},  {788, 146},
  //       {812, 187},  {714, 305},  {829, 406},  {928, 338},  {966, 307},  {948, 248},  {832, 108},
  //       {1395, 175}, {1728, 110}, {1487, 330}, {1533, 403}, {1657, 425}, {1724, 685}, {1673, 792},
  //       {1625, 793}, {1176, 972}, {219, 953},  {391, 560}},
  //      {{415, 806}, {564, 677}, {906, 582}, {1218, 421}, {1287, 467}, {1193, 574}, {1169, 718}, {1281,
  //      788}}};

  // points = {{{-1, -0.5}, {-1, 1.5}, {1, -1}, {1, 1}}};
  //  tri.configTri.multithreading = true;
  points = {{{29, 19},
             {335, 60},
             {398, 108},
             {516, 220},
             {507, 468},
             {531, 469},
             {549, 367},
             {571, 26},
             {733, 72},
             {859, 416},
             {848, 599},
             {738, 639},
             {832, 687},
             {849, 741},
             {792, 769},
             {248, 872},
             {227, 886},
             {136, 895},
             {133, 877}}};
  //   std::vector<Vec2Set> points = {
  //       {{36, 5}, {62, 0}, {94, 66}, {95, 72}, {100, 92}, {73, 76}, {26, 84}, {21, 100}, {0, 40}}};
  //    std::vector<Vec2Set> points = {{{-1, -1}, {-1, 1}, {1, 1}, {1, -1}}, {{-2, -2}, {-2, 2}, {2, 2}, {2,
  //    -2}},
  //                                   {{-3, -3}, {-3, 3}, {3, 3}, {3, -3}}, {{-4, -4}, {-4, 4}, {4, 4}, {4,
  //                                   -4}},
  //                                   {{-5, -5}, {-5, 5}, {5, 5}, {5, -5}}, {{-6, -6}, {-6, 6}, {6, 6}, {6,
  //                                   -6}},
  //                                   {{-7, -7}, {-7, 7}, {7, 7}, {7, -7}}, {{-8, -8}, {-8, 8}, {8, 8}, {8,
  //                                   -8}}};

  // std::vector<Vec2Set> points = {{{2, 2}, {2, -2}, {0, -2}, {1, 0}, {-1, 0}, {0, -2}, {-2, -2}, {-2, 2}}};

  // Vec2Set points = {{183, 149}, {562, 966}, {819, 892}, {547, 138},
  //                   {524, 752}, {480, 54},  {327, 91},  {276, 168}};

  // points = PolygonGenerator::GenerateRandomPolygon(5000);

  tri.config.useGivenSeed = true;
  tri.config.seed         = 1;
#  ifdef _DEBUG
  tri.config.incremental  = true;
  tri.config.maxSegment   = 1;
#  endif

  // visualization
  InitiateViewer();

  while (!WindowShouldClose())  // Escape or exit button clicked
  {
    ResolveCameraInteraction(camera);
    Vector2 mousePos = GetMousePosition(), worldMousePos = GetScreenToWorld2D(mousePos, camera);

    // draw
    BeginDrawing();
    ClearBackground(BLACK);
    BeginMode2D(camera);

    //======================================================================================================
    // update properties
    {
      GuiSetStyle(DEFAULT, TEXT_SIZE, 18 / camera.zoom);
      DrawMyGrid();
    }

    // generate random
    ResolveGeneration();
    if (!GeneratePoints())
      continue;

    if (ResolveConfig(tri) || first || states.generateRandom || states.randomSeed)
    {
      tri.ClearCache();
      tri.Build();
      tri.Triangulate();

      first                 = false;
      states.generateRandom = false;
      states.randomSeed     = false;
    }

    tri._infoBuf.clear();
    tri._infoBuf += "Screen Pos: (" + std::to_string(int(mousePos.x)) + ", " +
                    std::to_string(int(mousePos.y)) + ") World Pos: (" + std::to_string(worldMousePos.x) +
                    ", " + std::to_string(screenHeight - worldMousePos.y) +
                    ") Zoom: " + std::to_string(camera.zoom) + "\n";

    // draw the shape
    auto realPos = RealPos({mousePos.x, mousePos.y});
    tri.SetOrigin({screenWidth / 2, screenHeight / 2});
    tri.SetBox({screenWidth, screenHeight});
    tri.SetFocus({realPos.x, realPos.y});
    tri.SetZoomFactor(camera.zoom);

    Vec2 ori = (lt + rb) / 2, factor{0.8 * screenWidth / (rb - lt).x, 0.8 * screenHeight / (rb - lt).y};

    tri.Draw(ori, factor);

    // draw info
    auto fpsText = std::to_string(GetFPS()) + " fps";
    DrawTextEx(font, fpsText.c_str(), RealPos({10, 10}), 20 / camera.zoom, 0, WHITE);
    DrawTextEx(font, tri._infoBuf.c_str(), RealPos({5, screenHeight + 5}), 20 / camera.zoom, 0, WHITE);

    //======================================================================================================
    EndMode2D();
    EndDrawing();
  }

  UnloadFont(font);

  CloseWindow();
}
#endif