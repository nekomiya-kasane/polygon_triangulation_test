#pragma once

#include "allocator.h"
#include "vec2.h"

struct Node
{
  Node(uint32_t id, double x, double y);

  uint32_t id;
  double x, y;
  Node *prev, *next;
  Node *prevZ, *nextZ;

  bool steiner;
};

class FistTriangulator
{
public:
  enum Chiraty
  {
    UNKNOWN,
    CLOCKWISE,
    COUNTERCLOCKWISE,
  };

  uint32_t INVALID_UINT = -1;
  struct
  {
    short dim                       = 2;
    double tolerance                = 1e-12;
    uint32_t useZCurveAccellaration = 80;
  } config;

  FistTriangulator(uint32_t pointCount, short dimension = 2);

  // internal charility: clockwise for the boundary and CCW for the holes.
  // if the charility is unknown, it will be calculated internally.
  void SetBoundary(double *points, uint32_t size, Chiraty boundaryChirality = UNKNOWN);
  void AppendHole(double *points, uint32_t size, Chiraty holeCharility = UNKNOWN);
  void Triangulate();

  inline bool IsReady() const { return !!_boundary; }

protected:
  Node *CreateLinkedList(double *points,
                         uint32_t size,
                         Chiraty expectedChiraty = CLOCKWISE,
                         Chiraty targetChiraty   = UNKNOWN);
  Node *FindBridge(Node *boundary, Node *hole) const;
  Node *SplitPolygon(Node *A, Node *B);
  void RemoveHoles();
  void RemoveDuplicateVertices(Node *start, Node *end);

  double EvalSignedArea(double *points, uint32_t size) const; /* positive if CCW */
  double EvalSignedArea(const Node *const A,
                        const Node *const B,
                        const Node *const C) const; /* positive if CCW */
  bool LocallyInside(const Node *const base, const Node *const point) const;
  bool MiddleInside(const Node *const A, const Node *const B) const;
  uint32_t EvalDepth(const Node *const base, double x, double y) const;
  bool PointInTriangle(double Ax, double Ay, double Bx, double By, double Cx, double Cy, double Px, double Py)
      const;

protected:
  // insert node just after [last]
  Node *InsertNode(uint32_t id, double x, double y, Node *last = nullptr);
  void RemoveNode(Node *node);

  Node *_boundary = nullptr;

  std::vector<Node *> _holes;
  RecyclableAllocator<Node, false> _alloc;
};