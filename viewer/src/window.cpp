﻿#include "polygon_generator.h"
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
  //      {{415, 806}, {564, 677}, {906, 582}, {1218, 421}, {1287, 467}, {1193, 574}, {1169, vv718}, {1281,
  //      788}}};

  // points = {{{-1, -0.5}, {-1, 1.5}, {1, -1}, {1, 1}}};
  //  tri.configTri.multithreading = true;
  // points = {{{0, 0},     {98, 21},   {91, 41},  {0, 0},     {67, 74},   {50, 87},  {0, 0},     {10, 99},
  //           {-10, 99},  {0, 0},     {-50, 87}, {-67, 74},  {0, 0},     {-91, 41}, {-98, 21},  {0, 0},
  //           {-98, -21}, {-91, -41}, {0, 0},    {-67, -74}, {-50, -87}, {0, 0},    {-10, -99}, {10, -99},
  //           {0, 0},     {50, -87},  {67, -74}, {0, 0},     {91, -41},  {98, -21}}};
  // std::vector<Vec2Set> points = {
  //     {{36, 5}, {62, 0}, {94, 66}, {95, 72}, {100, 92}, {73, 76}, {26, 84}, {21, 100}, {0, 40}}};
  //     std::vector<Vec2Set> points = {{{-1, -1}, {-1, 1}, {1, 1}, {1, -1}}, {{-2, -2}, {-2, 2}, {2, 2}, {2,
  //     -2}},
  //                                    {{-3, -3}, {-3, 3}, {3, 3}, {3, -3}}, {{-4, -4}, {-4, 4}, {4, 4}, {4,
  //                                    -4}},
  //                                    {{-5, -5}, {-5, 5}, {5, 5}, {5, -5}}, {{-6, -6}, {-6, 6}, {6, 6}, {6,
  //                                    -6}},
  //                                    {{-7, -7}, {-7, 7}, {7, 7}, {7, -7}}, {{-8, -8}, {-8, 8}, {8, 8}, {8,
  //                                    -8}}};

  // points = {{{2, 2}, {2, -3}, {0, -2}, {1, -1}, {-1, 0}, {0, -2}, {-2, -1}, {-2, 2}}};

  // Vec2Set points = {{183, 149}, {562, 966}, {819, 892}, {547, 138},
  //                   {524, 752}, {480, 54},  {327, 91},  {276, 168}};

  // points = PolygonGenerator::GenerateRandomPolygon(5000);

  points = {{{0., 0.}, {99.998, 0.628943},    {99.9921, 1.25786},
             {0., 0.}, {99.9684, 2.51552},    {99.9506, 3.14422},
             {0., 0.}, {99.9031, 4.40121},    {99.8734, 5.02946},
             {0., 0.}, {99.8023, 6.28533},    {99.7608, 6.9129},
             {0., 0.}, {99.6659, 8.16721},    {99.6126, 8.79389},
             {0., 0.}, {99.4941, 10.0462},    {99.4289, 10.6717},
             {0., 0.}, {99.2868, 11.9216},    {99.2099, 12.5458},
             {0., 0.}, {99.0442, 13.7927},    {98.9555, 14.4154},
             {0., 0.}, {98.7664, 15.659},     {98.6659, 16.2799},
             {0., 0.}, {98.4533, 17.5196},    {98.3412, 18.1385},
             {0., 0.}, {98.1053, 19.3741},    {97.9815, 19.9907},
             {0., 0.}, {97.7223, 21.2216},    {97.5869, 21.8358},
             {0., 0.}, {97.3045, 23.0616},    {97.1575, 23.6731},
             {0., 0.}, {96.8521, 24.8934},    {96.6936, 25.502},
             {0., 0.}, {96.3651, 26.7163},    {96.1952, 27.3218},
             {0., 0.}, {95.8439, 28.5297},    {95.6626, 29.1319},
             {0., 0.}, {95.2886, 30.3329},    {95.0959, 30.9316},
             {0., 0.}, {94.6993, 32.1253},    {94.4954, 32.7203},
             {0., 0.}, {94.0764, 33.9063},    {93.8612, 34.4974},
             {0., 0.}, {93.4199, 35.6753},    {93.1937, 36.2621},
             {0., 0.}, {92.7302, 37.4315},    {92.4929, 38.014},
             {0., 0.}, {92.0074, 39.1744},    {91.7592, 39.7523},
             {0., 0.}, {91.2519, 40.9034},    {90.9929, 41.4765},
             {0., 0.}, {90.464, 42.6178},     {90.1941, 43.1859},
             {0., 0.}, {89.6438, 44.317},     {89.3633, 44.8799},
             {0., 0.}, {88.7917, 46.0004},    {88.5006, 46.558},
             {0., 0.}, {87.908, 47.6675},     {87.6064, 48.2195},
             {0., 0.}, {86.993, 49.3176},     {86.6811, 49.8638},
             {0., 0.}, {86.047, 50.9502},     {85.7248, 51.4903},
             {0., 0.}, {85.0704, 52.5646},    {84.7381, 53.0986},
             {0., 0.}, {84.0635, 54.1602},    {83.7212, 54.6879},
             {0., 0.}, {83.0267, 55.7367},    {82.6745, 56.2577},
             {0., 0.}, {81.9603, 57.2932},    {81.5983, 57.8076},
             {0., 0.}, {80.8647, 58.8294},    {80.4931, 59.3368},
             {0., 0.}, {79.7404, 60.3446},    {79.3593, 60.845},
             {0., 0.}, {78.5876, 61.8384},    {78.1972, 62.3314},
             {0., 0.}, {77.4069, 63.3101},    {77.0072, 63.7957},
             {0., 0.}, {76.1987, 64.7593},    {75.7898, 65.2373},
             {0., 0.}, {74.9633, 66.1854},    {74.5455, 66.6556},
             {0., 0.}, {73.7012, 67.588},     {73.2746, 68.0502},
             {0., 0.}, {72.4128, 68.9665},    {71.9777, 69.4206},
             {0., 0.}, {71.0987, 70.3205},    {70.6551, 70.7662},
             {0., 0.}, {69.7593, 71.6494},    {69.3073, 72.0867},
             {0., 0.}, {68.3951, 72.9528},    {67.9349, 73.3815},
             {0., 0.}, {67.0065, 74.2303},    {66.5383, 74.6502},
             {0., 0.}, {65.594, 75.4813},     {65.118, 75.8923},
             {0., 0.}, {64.1582, 76.7054},    {63.6745, 77.1074},
             {0., 0.}, {62.6996, 77.9023},    {62.2084, 78.2951},
             {0., 0.}, {61.2186, 79.0714},    {60.7201, 79.4548},
             {0., 0.}, {59.7159, 80.2123},    {59.2102, 80.5863},
             {0., 0.}, {58.1918, 81.3247},    {57.6792, 81.6891},
             {0., 0.}, {56.6471, 82.4082},    {56.1277, 82.7628},
             {0., 0.}, {55.0822, 83.4623},    {54.5562, 83.8071},
             {0., 0.}, {53.4977, 84.4867},    {52.9653, 84.8215},
             {0., 0.}, {51.8941, 85.481},     {51.3555, 85.8057},
             {0., 0.}, {50.2721, 86.4449},    {49.7274, 86.7593},
             {0., 0.}, {48.6322, 87.378},     {48.0816, 87.6821},
             {0., 0.}, {46.9749, 88.28},      {46.4188, 88.5737},
             {0., 0.}, {45.301, 89.1506},     {44.7394, 89.4337},
             {0., 0.}, {43.6109, 89.9894},    {43.044, 90.2619},
             {0., 0.}, {41.9052, 90.7962},    {41.3334, 91.058},
             {0., 0.}, {40.1847, 91.5707},    {39.608, 91.8216},
             {0., 0.}, {38.4499, 92.3126},    {37.8685, 92.5526},
             {0., 0.}, {36.7013, 93.0216},    {36.1155, 93.2506},
             {0., 0.}, {34.9397, 93.6975},    {34.3497, 93.9154},
             {0., 0.}, {33.1657, 94.34},      {32.5717, 94.5467},
             {0., 0.}, {31.3798, 94.949},     {30.782, 95.1444},
             {0., 0.}, {29.5828, 95.5241},    {28.9814, 95.7083},
             {0., 0.}, {27.7753, 96.0653},    {27.1705, 96.2381},
             {0., 0.}, {25.9578, 96.5722},    {25.3499, 96.7336},
             {0., 0.}, {24.1312, 97.0448},    {23.5203, 97.1946},
             {0., 0.}, {22.2959, 97.4828},    {21.6823, 97.6211},
             {0., 0.}, {20.4527, 97.8861},    {19.8366, 98.0128},
             {0., 0.}, {18.6022, 98.2546},    {17.9839, 98.3696},
             {0., 0.}, {16.7451, 98.588},     {16.1247, 98.6914},
             {0., 0.}, {14.882, 98.8864},     {14.2598, 98.9781},
             {0., 0.}, {13.0136, 99.1496},    {12.3898, 99.2295},
             {0., 0.}, {11.1406, 99.3775},    {10.5154, 99.4456},
             {0., 0.}, {9.26367, 99.57},      {8.63725, 99.6263},
             {0., 0.}, {7.38341, 99.7271},    {6.75604, 99.7715},
             {0., 0.}, {5.50051, 99.8486},    {4.87241, 99.8812},
             {0., 0.}, {3.61566, 99.9346},    {2.98706, 99.9554},
             {0., 0.}, {1.72952, 99.985},     {1.10064, 99.9939},
             {0., 0.}, {-0.157237, 99.9999},  {-0.786176, 99.9969},
             {0., 0.}, {-2.04394, 99.9791},   {-2.67271, 99.9643},
             {0., 0.}, {-3.92991, 99.9227},   {-4.55829, 99.8961},
             {0., 0.}, {-5.81448, 99.8308},   {-6.44225, 99.7923},
             {0., 0.}, {-7.69699, 99.7033},   {-8.32391, 99.653},
             {0., 0.}, {-9.57675, 99.5404},   {-10.2026, 99.4782},
             {0., 0.}, {-11.4531, 99.342},    {-12.0777, 99.268},
             {0., 0.}, {-13.3254, 99.1082},   {-13.9485, 99.0224},
             {0., 0.}, {-15.1929, 98.8391},   {-15.8143, 98.7416},
             {0., 0.}, {-17.055, 98.5349},    {-17.6744, 98.4257},
             {0., 0.}, {-18.9111, 98.1956},   {-19.5283, 98.0747},
             {0., 0.}, {-20.7604, 97.8213},   {-21.3752, 97.6888},
             {0., 0.}, {-22.6023, 97.4122},   {-23.2146, 97.2681},
             {0., 0.}, {-24.4362, 96.9684},   {-25.0456, 96.8128},
             {0., 0.}, {-26.2614, 96.4901},   {-26.8678, 96.323},
             {0., 0.}, {-28.0772, 95.9774},   {-28.6803, 95.799},
             {0., 0.}, {-29.8831, 95.4306},   {-30.4827, 95.2408},
             {0., 0.}, {-31.6783, 94.8498},   {-32.2742, 94.6487},
             {0., 0.}, {-33.4622, 94.2352},   {-34.0542, 94.0229},
             {0., 0.}, {-35.2342, 93.5871},   {-35.8221, 93.3637},
             {0., 0.}, {-36.9937, 92.9057},   {-37.5773, 92.6712},
             {0., 0.}, {-38.74, 92.1912},     {-39.319, 91.9457},
             {0., 0.}, {-40.4725, 91.4439},   {-41.0468, 91.1875},
             {0., 0.}, {-42.1906, 90.664},    {-42.76, 90.3968},
             {0., 0.}, {-43.8936, 89.8518},   {-44.4579, 89.574},
             {0., 0.}, {-45.5811, 89.0077},   {-46.14, 88.7192},
             {0., 0.}, {-47.2523, 88.1318},   {-47.8057, 87.8329},
             {0., 0.}, {-48.9067, 87.2246},   {-49.4543, 86.9153},
             {0., 0.}, {-50.5437, 86.2864},   {-51.0854, 85.9668},
             {0., 0.}, {-52.1627, 85.3174},   {-52.6983, 84.9876},
             {0., 0.}, {-53.7631, 84.318},    {-54.2924, 83.9782},
             {0., 0.}, {-55.3444, 83.2886},   {-55.8671, 82.9389},
             {0., 0.}, {-56.906, 82.2296},    {-57.422, 81.8701},
             {0., 0.}, {-58.4473, 81.1413},   {-58.9565, 80.7721},
             {0., 0.}, {-59.9678, 80.0241},   {-60.4699, 79.6454},
             {0., 0.}, {-61.467, 78.8785},    {-61.9619, 78.4903},
             {0., 0.}, {-62.9443, 77.7047},   {-63.4317, 77.3073},
             {0., 0.}, {-64.3991, 76.5033},   {-64.879, 76.0967},
             {0., 0.}, {-65.8311, 75.2746},   {-66.3032, 74.8591},
             {0., 0.}, {-67.2396, 74.0192},   {-67.7038, 73.5948},
             {0., 0.}, {-68.6242, 72.7374},   {-69.0803, 72.3043},
             {0., 0.}, {-69.9843, 71.4297},   {-70.4322, 70.9881},
             {0., 0.}, {-71.3195, 70.0965},   {-71.759, 69.6466},
             {0., 0.}, {-72.6294, 68.7384},   {-73.0603, 68.2803},
             {0., 0.}, {-73.9134, 67.3559},   {-74.3355, 66.8897},
             {0., 0.}, {-75.171, 65.9494},    {-75.5843, 65.4753},
             {0., 0.}, {-76.4019, 64.5193},   {-76.8062, 64.0375},
             {0., 0.}, {-77.6056, 63.0664},   {-78.0007, 62.577},
             {0., 0.}, {-78.7817, 61.5909},   {-79.1675, 61.0942},
             {0., 0.}, {-79.9297, 60.0936},   {-80.3061, 59.5897},
             {0., 0.}, {-81.0493, 58.5748},   {-81.4161, 58.0639},
             {0., 0.}, {-82.14, 57.0352},     {-82.4971, 56.5175},
             {0., 0.}, {-83.2015, 55.4753},   {-83.5488, 54.9509},
             {0., 0.}, {-84.2334, 53.8956},   {-84.5707, 53.3648},
             {0., 0.}, {-85.2352, 52.2968},   {-85.5625, 51.7597},
             {0., 0.}, {-86.2068, 50.6793},   {-86.5238, 50.1361},
             {0., 0.}, {-87.1476, 49.0438},   {-87.4543, 48.4947},
             {0., 0.}, {-88.0574, 47.3908},   {-88.3537, 46.8361},
             {0., 0.}, {-88.9359, 45.721},    {-89.2217, 45.1607},
             {0., 0.}, {-89.7827, 44.0349},   {-90.0579, 43.4693},
             {0., 0.}, {-90.5975, 42.3331},   {-90.862, 41.7624},
             {0., 0.}, {-91.3801, 40.6162},   {-91.6338, 40.0407},
             {0., 0.}, {-92.1302, 38.8849},   {-92.3729, 38.3047},
             {0., 0.}, {-92.8474, 37.1397},   {-93.0792, 36.555},
             {0., 0.}, {-93.5316, 35.3813},   {-93.7523, 34.7924},
             {0., 0.}, {-94.1825, 33.6103},   {-94.392, 33.0173},
             {0., 0.}, {-94.7999, 31.8274},   {-94.9982, 31.2305},
             {0., 0.}, {-95.3835, 30.0331},   {-95.5705, 29.4326},
             {0., 0.}, {-95.9332, 28.2281},   {-96.1088, 27.6242},
             {0., 0.}, {-96.4487, 26.4131},   {-96.6129, 25.806},
             {0., 0.}, {-96.9299, 24.5887},   {-97.0826, 23.9785},
             {0., 0.}, {-97.3765, 22.7555},   {-97.5177, 22.1426},
             {0., 0.}, {-97.7885, 20.9142},   {-97.9181, 20.2987},
             {0., 0.}, {-98.1657, 19.0655},   {-98.2837, 18.4477},
             {0., 0.}, {-98.508, 17.2099},    {-98.6142, 16.59},
             {0., 0.}, {-98.8151, 15.3483},   {-98.9097, 14.7265},
             {0., 0.}, {-99.0871, 13.4812},   {-99.1699, 12.8577},
             {0., 0.}, {-99.3238, 11.6093},   {-99.3949, 10.9844},
             {0., 0.}, {-99.5252, 9.73325},   {-99.5844, 9.1071},
             {0., 0.}, {-99.6911, 7.85375},   {-99.7385, 7.22659},
             {0., 0.}, {-99.8215, 5.97145},   {-99.8571, 5.34351},
             {0., 0.}, {-99.9164, 4.08702},   {-99.9402, 3.45852},
             {0., 0.}, {-99.9758, 2.20114},   {-99.9876, 1.5723},
             {0., 0.}, {-99.9995, 0.314473},  {-99.9995, -0.314473},
             {0., 0.}, {-99.9876, -1.5723},   {-99.9758, -2.20114},
             {0., 0.}, {-99.9402, -3.45852},  {-99.9164, -4.08702},
             {0., 0.}, {-99.8571, -5.34351},  {-99.8215, -5.97145},
             {0., 0.}, {-99.7385, -7.22659},  {-99.6911, -7.85375},
             {0., 0.}, {-99.5844, -9.1071},   {-99.5252, -9.73325},
             {0., 0.}, {-99.3949, -10.9844},  {-99.3238, -11.6093},
             {0., 0.}, {-99.1699, -12.8577},  {-99.0871, -13.4812},
             {0., 0.}, {-98.9097, -14.7265},  {-98.8151, -15.3483},
             {0., 0.}, {-98.6142, -16.59},    {-98.508, -17.2099},
             {0., 0.}, {-98.2837, -18.4477},  {-98.1657, -19.0655},
             {0., 0.}, {-97.9181, -20.2987},  {-97.7885, -20.9142},
             {0., 0.}, {-97.5177, -22.1426},  {-97.3765, -22.7555},
             {0., 0.}, {-97.0826, -23.9785},  {-96.9299, -24.5887},
             {0., 0.}, {-96.6129, -25.806},   {-96.4487, -26.4131},
             {0., 0.}, {-96.1088, -27.6242},  {-95.9332, -28.2281},
             {0., 0.}, {-95.5705, -29.4326},  {-95.3835, -30.0331},
             {0., 0.}, {-94.9982, -31.2305},  {-94.7999, -31.8274},
             {0., 0.}, {-94.392, -33.0173},   {-94.1825, -33.6103},
             {0., 0.}, {-93.7523, -34.7924},  {-93.5316, -35.3813},
             {0., 0.}, {-93.0792, -36.555},   {-92.8474, -37.1397},
             {0., 0.}, {-92.3729, -38.3047},  {-92.1302, -38.8849},
             {0., 0.}, {-91.6338, -40.0407},  {-91.3801, -40.6162},
             {0., 0.}, {-90.862, -41.7624},   {-90.5975, -42.3331},
             {0., 0.}, {-90.0579, -43.4693},  {-89.7827, -44.0349},
             {0., 0.}, {-89.2217, -45.1607},  {-88.9359, -45.721},
             {0., 0.}, {-88.3537, -46.8361},  {-88.0574, -47.3908},
             {0., 0.}, {-87.4543, -48.4947},  {-87.1476, -49.0438},
             {0., 0.}, {-86.5238, -50.1361},  {-86.2068, -50.6793},
             {0., 0.}, {-85.5625, -51.7597},  {-85.2352, -52.2968},
             {0., 0.}, {-84.5707, -53.3648},  {-84.2334, -53.8956},
             {0., 0.}, {-83.5488, -54.9509},  {-83.2015, -55.4753},
             {0., 0.}, {-82.4971, -56.5175},  {-82.14, -57.0352},
             {0., 0.}, {-81.4161, -58.0639},  {-81.0493, -58.5748},
             {0., 0.}, {-80.3061, -59.5897},  {-79.9297, -60.0936},
             {0., 0.}, {-79.1675, -61.0942},  {-78.7817, -61.5909},
             {0., 0.}, {-78.0007, -62.577},   {-77.6056, -63.0664},
             {0., 0.}, {-76.8062, -64.0375},  {-76.4019, -64.5193},
             {0., 0.}, {-75.5843, -65.4753},  {-75.171, -65.9494},
             {0., 0.}, {-74.3355, -66.8897},  {-73.9134, -67.3559},
             {0., 0.}, {-73.0603, -68.2803},  {-72.6294, -68.7384},
             {0., 0.}, {-71.759, -69.6466},   {-71.3195, -70.0965},
             {0., 0.}, {-70.4322, -70.9881},  {-69.9843, -71.4297},
             {0., 0.}, {-69.0803, -72.3043},  {-68.6242, -72.7374},
             {0., 0.}, {-67.7038, -73.5948},  {-67.2396, -74.0192},
             {0., 0.}, {-66.3032, -74.8591},  {-65.8311, -75.2746},
             {0., 0.}, {-64.879, -76.0967},   {-64.3991, -76.5033},
             {0., 0.}, {-63.4317, -77.3073},  {-62.9443, -77.7047},
             {0., 0.}, {-61.9619, -78.4903},  {-61.467, -78.8785},
             {0., 0.}, {-60.4699, -79.6454},  {-59.9678, -80.0241},
             {0., 0.}, {-58.9565, -80.7721},  {-58.4473, -81.1413},
             {0., 0.}, {-57.422, -81.8701},   {-56.906, -82.2296},
             {0., 0.}, {-55.8671, -82.9389},  {-55.3444, -83.2886},
             {0., 0.}, {-54.2924, -83.9782},  {-53.7631, -84.318},
             {0., 0.}, {-52.6983, -84.9876},  {-52.1627, -85.3174},
             {0., 0.}, {-51.0854, -85.9668},  {-50.5437, -86.2864},
             {0., 0.}, {-49.4543, -86.9153},  {-48.9067, -87.2246},
             {0., 0.}, {-47.8057, -87.8329},  {-47.2523, -88.1318},
             {0., 0.}, {-46.14, -88.7192},    {-45.5811, -89.0077},
             {0., 0.}, {-44.4579, -89.574},   {-43.8936, -89.8518},
             {0., 0.}, {-42.76, -90.3968},    {-42.1906, -90.664},
             {0., 0.}, {-41.0468, -91.1875},  {-40.4725, -91.4439},
             {0., 0.}, {-39.319, -91.9457},   {-38.74, -92.1912},
             {0., 0.}, {-37.5773, -92.6712},  {-36.9937, -92.9057},
             {0., 0.}, {-35.8221, -93.3637},  {-35.2342, -93.5871},
             {0., 0.}, {-34.0542, -94.0229},  {-33.4622, -94.2352},
             {0., 0.}, {-32.2742, -94.6487},  {-31.6783, -94.8498},
             {0., 0.}, {-30.4827, -95.2408},  {-29.8831, -95.4306},
             {0., 0.}, {-28.6803, -95.799},   {-28.0772, -95.9774},
             {0., 0.}, {-26.8678, -96.323},   {-26.2614, -96.4901},
             {0., 0.}, {-25.0456, -96.8128},  {-24.4362, -96.9684},
             {0., 0.}, {-23.2146, -97.2681},  {-22.6023, -97.4122},
             {0., 0.}, {-21.3752, -97.6888},  {-20.7604, -97.8213},
             {0., 0.}, {-19.5283, -98.0747},  {-18.9111, -98.1956},
             {0., 0.}, {-17.6744, -98.4257},  {-17.055, -98.5349},
             {0., 0.}, {-15.8143, -98.7416},  {-15.1929, -98.8391},
             {0., 0.}, {-13.9485, -99.0224},  {-13.3254, -99.1082},
             {0., 0.}, {-12.0777, -99.268},   {-11.4531, -99.342},
             {0., 0.}, {-10.2026, -99.4782},  {-9.57675, -99.5404},
             {0., 0.}, {-8.32391, -99.653},   {-7.69699, -99.7033},
             {0., 0.}, {-6.44225, -99.7923},  {-5.81448, -99.8308},
             {0., 0.}, {-4.55829, -99.8961},  {-3.92991, -99.9227},
             {0., 0.}, {-2.67271, -99.9643},  {-2.04394, -99.9791},
             {0., 0.}, {-0.786176, -99.9969}, {-0.157237, -99.9999},
             {0., 0.}, {1.10064, -99.9939},   {1.72952, -99.985},
             {0., 0.}, {2.98706, -99.9554},   {3.61566, -99.9346},
             {0., 0.}, {4.87241, -99.8812},   {5.50051, -99.8486},
             {0., 0.}, {6.75604, -99.7715},   {7.38341, -99.7271},
             {0., 0.}, {8.63725, -99.6263},   {9.26367, -99.57},
             {0., 0.}, {10.5154, -99.4456},   {11.1406, -99.3775},
             {0., 0.}, {12.3898, -99.2295},   {13.0136, -99.1496},
             {0., 0.}, {14.2598, -98.9781},   {14.882, -98.8864},
             {0., 0.}, {16.1247, -98.6914},   {16.7451, -98.588},
             {0., 0.}, {17.9839, -98.3696},   {18.6022, -98.2546},
             {0., 0.}, {19.8366, -98.0128},   {20.4527, -97.8861},
             {0., 0.}, {21.6823, -97.6211},   {22.2959, -97.4828},
             {0., 0.}, {23.5203, -97.1946},   {24.1312, -97.0448},
             {0., 0.}, {25.3499, -96.7336},   {25.9578, -96.5722},
             {0., 0.}, {27.1705, -96.2381},   {27.7753, -96.0653},
             {0., 0.}, {28.9814, -95.7083},   {29.5828, -95.5241},
             {0., 0.}, {30.782, -95.1444},    {31.3798, -94.949},
             {0., 0.}, {32.5717, -94.5467},   {33.1657, -94.34},
             {0., 0.}, {34.3497, -93.9154},   {34.9397, -93.6975},
             {0., 0.}, {36.1155, -93.2506},   {36.7013, -93.0216},
             {0., 0.}, {37.8685, -92.5526},   {38.4499, -92.3126},
             {0., 0.}, {39.608, -91.8216},    {40.1847, -91.5707},
             {0., 0.}, {41.3334, -91.058},    {41.9052, -90.7962},
             {0., 0.}, {43.044, -90.2619},    {43.6109, -89.9894},
             {0., 0.}, {44.7394, -89.4337},   {45.301, -89.1506},
             {0., 0.}, {46.4188, -88.5737},   {46.9749, -88.28},
             {0., 0.}, {48.0816, -87.6821},   {48.6322, -87.378},
             {0., 0.}, {49.7274, -86.7593},   {50.2721, -86.4449},
             {0., 0.}, {51.3555, -85.8057},   {51.8941, -85.481},
             {0., 0.}, {52.9653, -84.8215},   {53.4977, -84.4867},
             {0., 0.}, {54.5562, -83.8071},   {55.0822, -83.4623},
             {0., 0.}, {56.1277, -82.7628},   {56.6471, -82.4082},
             {0., 0.}, {57.6792, -81.6891},   {58.1918, -81.3247},
             {0., 0.}, {59.2102, -80.5863},   {59.7159, -80.2123},
             {0., 0.}, {60.7201, -79.4548},   {61.2186, -79.0714},
             {0., 0.}, {62.2084, -78.2951},   {62.6996, -77.9023},
             {0., 0.}, {63.6745, -77.1074},   {64.1582, -76.7054},
             {0., 0.}, {65.118, -75.8923},    {65.594, -75.4813},
             {0., 0.}, {66.5383, -74.6502},   {67.0065, -74.2303},
             {0., 0.}, {67.9349, -73.3815},   {68.3951, -72.9528},
             {0., 0.}, {69.3073, -72.0867},   {69.7593, -71.6494},
             {0., 0.}, {70.6551, -70.7662},   {71.0987, -70.3205},
             {0., 0.}, {71.9777, -69.4206},   {72.4128, -68.9665},
             {0., 0.}, {73.2746, -68.0502},   {73.7012, -67.588},
             {0., 0.}, {74.5455, -66.6556},   {74.9633, -66.1854},
             {0., 0.}, {75.7898, -65.2373},   {76.1987, -64.7593},
             {0., 0.}, {77.0072, -63.7957},   {77.4069, -63.3101},
             {0., 0.}, {78.1972, -62.3314},   {78.5876, -61.8384},
             {0., 0.}, {79.3593, -60.845},    {79.7404, -60.3446},
             {0., 0.}, {80.4931, -59.3368},   {80.8647, -58.8294},
             {0., 0.}, {81.5983, -57.8076},   {81.9603, -57.2932},
             {0., 0.}, {82.6745, -56.2577},   {83.0267, -55.7367},
             {0., 0.}, {83.7212, -54.6879},   {84.0635, -54.1602},
             {0., 0.}, {84.7381, -53.0986},   {85.0704, -52.5646},
             {0., 0.}, {85.7248, -51.4903},   {86.047, -50.9502},
             {0., 0.}, {86.6811, -49.8638},   {86.993, -49.3176},
             {0., 0.}, {87.6064, -48.2195},   {87.908, -47.6675},
             {0., 0.}, {88.5006, -46.558},    {88.7917, -46.0004},
             {0., 0.}, {89.3633, -44.8799},   {89.6438, -44.317},
             {0., 0.}, {90.1941, -43.1859},   {90.464, -42.6178},
             {0., 0.}, {90.9929, -41.4765},   {91.2519, -40.9034},
             {0., 0.}, {91.7592, -39.7523},   {92.0074, -39.1744},
             {0., 0.}, {92.4929, -38.014},    {92.7302, -37.4315},
             {0., 0.}, {93.1937, -36.2621},   {93.4199, -35.6753},
             {0., 0.}, {93.8612, -34.4974},   {94.0764, -33.9063},
             {0., 0.}, {94.4954, -32.7203},   {94.6993, -32.1253},
             {0., 0.}, {95.0959, -30.9316},   {95.2886, -30.3329},
             {0., 0.}, {95.6626, -29.1319},   {95.8439, -28.5297},
             {0., 0.}, {96.1952, -27.3218},   {96.3651, -26.7163},
             {0., 0.}, {96.6936, -25.502},    {96.8521, -24.8934},
             {0., 0.}, {97.1575, -23.6731},   {97.3045, -23.0616},
             {0., 0.}, {97.5869, -21.8358},   {97.7223, -21.2216},
             {0., 0.}, {97.9815, -19.9907},   {98.1053, -19.3741},
             {0., 0.}, {98.3412, -18.1385},   {98.4533, -17.5196},
             {0., 0.}, {98.6659, -16.2799},   {98.7664, -15.659},
             {0., 0.}, {98.9555, -14.4154},   {99.0442, -13.7927},
             {0., 0.}, {99.2099, -12.5458},   {99.2868, -11.9216},
             {0., 0.}, {99.4289, -10.6717},   {99.4941, -10.0462},
             {0., 0.}, {99.6126, -8.79389},   {99.6659, -8.16721},
             {0., 0.}, {99.7608, -6.9129},    {99.8023, -6.28533},
             {0., 0.}, {99.8734, -5.02946},   {99.9031, -4.40121},
             {0., 0.}, {99.9506, -3.14422},   {99.9684, -2.51552},
             {0., 0.}, {99.9921, -1.25786},   {99.998, -0.628943}}};

  tri.config.useGivenSeed = true;
  tri.config.seed         = -270861712;
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