#pragma once

#include "allocator.h"
#include "vec2.h"

struct Node
{
  Node(uint32_t id, double x, double y);

  uint32_t id;
  int32_t zid;
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

  static const uint32_t INVALID_UINT = -1;
  struct
  {
    short dim                       = 2;
    double tolerance                = 1e-12;
    uint32_t useZCurveAccellaration = 80;
  } config;

  FistTriangulator(uint32_t pointCount, short dimension = 2);

  // internal charility: clockwise for the boundary and CCW for the holes.
  // if the charility is unknown, it will be calculated internally.
  // add boundary then holes
  void SetBoundary(double *points, uint32_t size, Chiraty boundaryChirality = UNKNOWN);
  void AppendHole(double *points, uint32_t size, Chiraty holeCharility = UNKNOWN);
  void Triangulate();

  inline bool IsReady() const { return !!_boundary; }

  std::vector<uint32_t> triangles;

protected:
  // border manipulations
  Node *CreateLinkedList(double *points,
                         uint32_t size,
                         Chiraty expectedChiraty = CLOCKWISE,
                         Chiraty targetChiraty   = UNKNOWN);
  Node *FindBridge(Node *boundary, Node *hole) const;
  Node *SplitPolygon(Node *A, Node *B);
  Node *RemoveHoles();
  Node *RemoveRebundantVertices(Node *start, Node *end = nullptr);
  Node *CureLocalIntersections(Node *start);
  void ClumsyEarcut(Node *start);
  void Earcut(Node *start, int mode = 0); /* mode will be increased autoly to handle failure */

  // geometric tester
  double EvalSignedArea(double *points, uint32_t size) const; /* positive if CCW */
  static double EvalSignedArea(const Node *const A,
                               const Node *const B,
                               const Node *const C); /* positive if CCW */
  static bool LocallyInside(const Node *const base, const Node *const point);
  static bool MiddleInside(const Node *const A, const Node *const B);
  bool ValidDiagonal(const Node *A, const Node *B) const;
  static uint32_t EvalDepth(const Node *const base, double x, double y);
  static bool
  PointInTriangle(double Ax, double Ay, double Bx, double By, double Cx, double Cy, double Px, double Py);
  // static bool PointOnSegment(Node *L1, Node *L2, Node *point);
  static bool Intersected(const Node *L11, const Node *L12, const Node *L21, const Node *L22);
  static bool IntersectedWithPolygon(const Node *A, const Node *B, const Node *polygon);
  static bool ValidEar(const Node *node);
  static bool ValidHashedEar(const Node *node);

  // z curve acceleration
  int32_t EvalZOrder(double x, double y) const;
  void AssignZOrder(Node *borderNode);

protected:
  // insert node just after [last]
  Node *InsertNode(uint32_t id, double x, double y, Node *last = nullptr);
  void RemoveNode(Node *node);

  Node *_boundary = nullptr;
  double _minX, _maxX, _minY, _maxY, _invSize;  // bounding box, for z-curve acceleration

  std::vector<Node *> _holes;
  RecyclableAllocator<Node, false> _alloc;
};