#pragma warning(push, 0)
#include "raylib.h"

#include "raymath.h"
#include "rlgl.h"

#define RAYGUI_IMPLEMENTATION
#include "raygui.h"  // Required for GUI controls
#pragma warning(pop)

#include "fist/triangulator.h"
#include "polygon_generator.h"

#include <iostream>
#include <string>

std::vector<double> data = {120, 20, 20, 20, 520, 20, 620, 620, 20, 720};
FistTriangulator tri(data.size() / 2);
std::vector<std::pair<size_t, size_t>> components;

enum State
{
  NONE,
  ADDING_POINTS
} state;

struct States
{
  bool generateRandom    = false;
  bool printData         = true;
  int generateRandomSize = 20;
  Vector2 mousePos;
  Font font;
} states;

void DrawPoint(double x, double y, unsigned int index)
{
  DrawCircle(x, y, 2, RED);
  DrawCircleLines(x, y, 2, WHITE);
  DrawTextEx(states.font, ("v" + std::to_string(index)).c_str(), Vector2{(float)x, (float)y}, 20.f, 0.f, RED);
}

void DrawTriangles(const Vector2 &A, const Vector2 &B, const Vector2 &C, unsigned int index)
{
  float midX = (A.x + B.x + C.x) / 3, midY = (A.y + B.y + C.y) / 3;

  DrawTriangle(A, B, C, Fade(GREEN, 0.3f));
  DrawTriangle(A, C, B, Fade(GREEN, 0.3f));
  DrawTriangleLines(A, B, C, GREEN);
  DrawTextEx(states.font, ("T" + std::to_string(index)).c_str(), Vector2{midX, midY}, 20, 0, GREEN);
}

int main()
{
  // visualization
  const int screenWidth    = 1920;
  const int screenHeight   = 1080;
  const int infoPanelHight = 200;

  tri.SetBoundary(data.data(), data.size() / 2);
  tri.Triangulate();

  InitWindow(screenWidth, screenHeight + infoPanelHight, "Seidel Algorithm Visualizer");
  SetWindowMinSize(screenWidth, screenHeight + infoPanelHight);

  Camera2D camera = {0};
  camera.zoom     = 1.0f;
  SetTargetFPS(144);

  states.font = LoadFontEx("assets/victor_mono.ttf", 96, 0, 0);

  bool first = true;
  while (!WindowShouldClose())  // Escape or exit button clicked
  {
    // move canvas
    if (IsKeyReleased(KEY_Q))
    {
      data.clear();
      tri.triangles.clear();
    }
    else if (IsKeyReleased(KEY_ENTER))
    {
      tri = FistTriangulator(data.size() / 2);
      for (size_t i = 0; i < components.size(); ++i)
      {
        if (i == 0)
        {
          tri.SetBoundary(data.data(), components[i].second);
        }
        else
        {
          tri.AppendHole(data.data() + 2 * components[i].first, components[i].second);
        }
      }

      tri.Triangulate();
    }

    if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT))
    {
      if (state == NONE)
      {
        state = ADDING_POINTS;
        components.push_back({data.size() / 2, 0});
      }
      Vector2 pos = GetMousePosition();
      data.push_back(pos.x);
      data.push_back(pos.y);
      components.back().second++;
    }
    if (IsMouseButtonReleased(MOUSE_BUTTON_RIGHT))
    {
      if (state == ADDING_POINTS)
      {
        state = NONE;
      }
    }

    states.mousePos = GetMousePosition();

    // draw
    BeginDrawing();
    {
      ClearBackground(BLACK);

      BeginMode2D(camera);
      {
        // generate random
        states.generateRandom = GuiCheckBox(
            Rectangle{10, screenHeight - 30, 20, 20},
            (" Generate random " + std::to_string(states.generateRandomSize)).c_str(), states.generateRandom);
        states.generateRandomSize = GuiSliderBar(Rectangle{10, screenHeight - 120, 100, 20}, NULL,
                                                 " Point count", states.generateRandomSize, 0, 100);
        DrawTextEx(
            states.font,
            (std::to_string(int(states.mousePos.x)) + ", " + std::to_string(int(states.mousePos.y))).c_str(),
            Vector2{10, screenHeight - 150}, 20, 0, GRAY);

        if (states.generateRandom)
        {
          auto points = PolygonGenerator::GenerateRandomPolygon(states.generateRandomSize);

          tri = FistTriangulator(data.size() / 2);
          data.clear();
          for (const auto &point : points[0])
          {
            data.push_back(point.x);
            data.push_back(point.y);
          }
          tri.SetBoundary(data.data(), data.size() / 2);
          tri.Triangulate();

          states.generateRandom = false;
          state == NONE;
        }

        // draw points
        for (size_t i = 0; i < data.size() / 2; ++i)
        {
          DrawPoint(data[2 * i], data[2 * i + 1], i);
        }
        const auto &result = tri.triangles;
        for (size_t i = 0; i < result.size() / 3; ++i)
        {
          DrawTriangles({(float)data[2 * result[3 * i]], (float)data[2 * result[3 * i] + 1]},
                        {(float)data[2 * result[3 * i + 1]], (float)data[2 * result[3 * i + 1] + 1]},
                        {(float)data[2 * result[3 * i + 2]], (float)data[2 * result[3 * i + 2] + 1]}, i);
        }

        // draw info
        DrawFPS(10, 10);
      }
      EndMode2D();
    }
    EndDrawing();
  }

  UnloadFont(states.font);

  CloseWindow();
}