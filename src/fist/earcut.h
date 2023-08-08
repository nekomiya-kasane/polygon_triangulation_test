#pragma once

#include <vector>

using ZOrder = unsigned int;
using NodeID = unsigned int;

struct Node
{
  unsigned int i;
  double x, y;

  ZOrder z;
  Node *prev = nullptr, *next = nullptr;
  Node *prevZ = nullptr, *nextZ = nullptr;

  bool steiner = false;
};

using Triangles = std::vector<unsigned int>;

class EarCutTriangulator
{
  Triangles Triangulate(const std::vector<double> &data,
                        const std::vector<unsigned int> &holeIndices,
                        short dim = 2);
  Node *FilterPoints(Node *start, Node *end = nullptr);
  Node *CureLocalIntersections(Node *start, Triangles &triangles, short dim);
  Node *FindHoleBridge(Node *hole, Node *outerNode);
  Node *ConnectHole(Node *hole, Node *outerNode);
  Node *ConnectHoles(const std::vector<double> &data,
                     const std::vector<unsigned int> &holeIndices,
                     Node *outerNode,
                     short dim);
  void IndexCurve(Node *start, double minX, double minY, double invSize);
  Node *SortLinked(Node *list);

  Node *CreateLinkedList(const std::vector<double> &data,
                         unsigned int start,
                         unsigned int end,
                         short dim,
                         bool clockwise);
  void EarcutLinked(Node *ear,
                    Triangles &triangles,
                    short dim,
                    double minX,
                    double minY,
                    double invSize,
                    short pass);

  bool IsEar(Node *ear);
  bool IsEarHashed(Node *ear, double minX, double minY, double invSize);
  bool SectorContainsSector(Node *m, Node *p);

  Node *InsertNode(NodeID i, double x, double y, Node *last);
  void RemoveNode(Node *p);

  ZOrder GetZOrder(double ix, double iy, double minX, double minY, double invSize);
  Node *GetLeftmost(Node *start);
  Node *SplitPolygon(Node *a, Node *b);
  void SplitEarcut(Node *start, Triangles &triangles, short dim, double minX, double minY, double invSize);

  bool
  PointInTriangle(double ax, double ay, double bx, double by, double cx, double cy, double px, double py);
  bool IsValidDiagonal(Node *a, Node *b);
  bool Equals(Node *p1, Node *p2);
  bool LocallyInside(Node *a, Node *b);
  bool MiddleInside(Node *a, Node *b);
  bool Intersects(Node *p1, Node *q1, Node *p2, Node *q2);
  bool IntersectsPolygon(Node *a, Node *b);
  bool OnSegment(Node *p, Node *q, Node *r);

  short GetSign(double num);
  double EvalArea(Node *p, Node *q, Node *r);
  double EvalSignedArea(const std::vector<double> &data, unsigned int start, unsigned int end, short dim);
};