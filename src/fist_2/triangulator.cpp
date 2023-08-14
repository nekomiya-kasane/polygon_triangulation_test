#include "triangulator.h"

#include <limits>

//=======================================================
// Node
//=======================================================

constexpr const char *const nodeFiller()
{
  char *const result = new char[sizeof(Node)];

  for (size_t i = 0; i < sizeof(Node); ++i)
    result[i] = '\0';

#define _FILL_DATA_(TYPE, MEMBER, VAL) *(TYPE *)&result[offsetof(Node, MEMBER)] = VAL;
  //_FILL_DATA_(Node *, prev, nullptr);
  //_FILL_DATA_(Node *, next, nullptr);
  _FILL_DATA_(Node *, prevZ, nullptr);
  _FILL_DATA_(Node *, nextZ, nullptr);

  _FILL_DATA_(bool, steiner, false);
#undef _FILL_DATA_

  return result;
}

Node::Node(uint32_t id, double x, double y)
{
  static const char *const data = nodeFiller();
  memcpy(this, data, sizeof(Node));

  this->id = id;
  this->x  = x;
  this->y  = y;
}

//=======================================================
// Triangulator
//=======================================================

FistTriangulator::FistTriangulator(uint32_t pointCount, short dimension)
{
  _alloc     = RecyclableAllocator<Node, false>(pointCount);
  config.dim = dimension;
}

void FistTriangulator::SetBoundary(double *points, uint32_t size, Chiraty boundaryCharility)
{
  if (size < 2)
    return;

  _boundary = CreateLinkedList(points, size, CLOCKWISE, boundaryCharility);
}

void FistTriangulator::AppendHole(double *points, uint32_t size, Chiraty holeCharility /* = UNKNOWN */)
{
  if (!size)
    return;

  Node *node = CreateLinkedList(points, size, COUNTERCLOCKWISE, holeCharility);
  if (!node)
    return;
  if (node->next == node)
    node->steiner = true;

  // find the left-most point (node)
  Node *curNode = node, *leftmostNode = node;
  do
  {
    if ((curNode->x < leftmostNode->x) || (curNode->x == leftmostNode->x && curNode->y < leftmostNode->y))
      leftmostNode = curNode;
    curNode = curNode->next;
  } while (curNode != node);

  _holes.push_back(leftmostNode);
}

void FistTriangulator::Triangulate()
{
  RemoveHoles();
}

Node *FistTriangulator::CreateLinkedList(double *points,
                                         uint32_t size,
                                         Chiraty expectedChiraty,
                                         Chiraty targetChiraty)
{
  if (!size)
    return nullptr;

  Node *lastNode = nullptr;

  if (expectedChiraty != UNKNOWN && targetChiraty == UNKNOWN)
    targetChiraty = EvalSignedArea(points, size) > 0 ? CLOCKWISE : COUNTERCLOCKWISE;

  if (expectedChiraty == UNKNOWN || targetChiraty == expectedChiraty)
  {
    for (uint32_t i = 0; i < size; ++i)
      lastNode = InsertNode(i, points[config.dim * i], points[config.dim * i + 1], lastNode);
  }
  else
  {
    for (uint32_t i = size - 1; i >= 0; --i)
      lastNode = InsertNode(i, points[config.dim * i], points[config.dim * i + 1], lastNode);
  }
}

// David Eberly's Algorithm
// Ref: https://www.geometrictools.com/Documentation/TriangulationByEarClipping.pdf
// todo: maybe searching through the whole polygon is not necessary. Use a geometric hashing technique
Node *FistTriangulator::FindBridge(Node *boundary, Node *hole) const
{
  Node *curBdrNode = _boundary, *nearestBdrEdgeNode = nullptr;
  const Node *leftMost = hole;

  /*        + cur
           /
          /       /
         o-------*
        /         \__
       /
      + next
  To find the edge with which the intersection of the horizontal line from the leftmost vertex of the hole has
  the largest x value smaller than the x coordinate of the vertex. Since the boundary polygon is CW, only
  edges going down can be candidates. */

  double maxX = -std::numeric_limits<double>::infinity();

  do
  {
    /* not going down or the horizontal line and the edge are not intersected */
    if (leftMost->y > curBdrNode->y || leftMost->y < leftMost->next->y ||
        curBdrNode->y == curBdrNode->next->y)
    {
      curBdrNode = curBdrNode->next;
      continue;
    }

    /* going down and intersected, x = x0 + (y - y0) / k */
    double ox = curBdrNode->x + (leftMost->y - curBdrNode->y) * (curBdrNode->next->x - curBdrNode->x) /
                                    (curBdrNode->next->y - curBdrNode->y);
    if (ox <= leftMost->x && ox > maxX)
    {
      maxX               = ox;
      nearestBdrEdgeNode = curBdrNode->x < curBdrNode->next->x /* left-top to right-bottom if true */
                               ? curBdrNode
                               : curBdrNode->next; /* select vertex with the lower x */
      if (std::abs(ox - leftMost->x) < config.tolerance / 4)
        /* leftmost vertex of the hole lies on the edge */
        return nearestBdrEdgeNode;
    }
    curBdrNode = curBdrNode->next;

  } while (curBdrNode != _boundary);

  if (!nearestBdrEdgeNode)
    return nullptr;

  // check if the diagonal is valid with no edge crossed. This is necessary since self-intersection may exist.
  /*      + cur
         /
        /       /
       o-------o                 o is the vertices of the triangle we focus on
      /.......  \__
     /....↖ any point in this triangle?
    /..
   o next
  If no such point, then the diagonal is valid, otherwise choose the vertex of the minimum angle with the ray
  as connection vertex */
  // clang-format off
  Node *originalNearestBdrEdgeNode = curBdrNode = nearestBdrEdgeNode;
  double minTan = std::numeric_limits<double>::infinity();
  // clang-format on

  do
  {
    // leftMost is the rightmost vertex of the triangle, nearestBdrEdgeNode is the leftmost
    if (curBdrNode->x >= leftMost->x
        /* current node is outside of the triangle since its x coordinate is larger than the rightmost vertex
           of the triangle */
        || curBdrNode->x < nearestBdrEdgeNode->x
        /* current node is outside of the triangle since its x coordinate is smaller than the leftmost vertex
           of the triangle */
        || !PointInTriangle(leftMost->y < nearestBdrEdgeNode->y ? leftMost->x : maxX, leftMost->y, /* A */
                            maxX, leftMost->y,                                                     /* B */
                            leftMost->y < nearestBdrEdgeNode->y ? maxX : leftMost->x, leftMost->y, /* C */
                            curBdrNode->x, curBdrNode->y                                           /* P */
                            )
        /* current node is outside of the triangle verified by a test method */
        /* triangle (A, B, C), test point (P)

           case 1:                                case 2:
             leftMost.y < nearestBdrEdgeNode.y      leftMost.y > nearestBdrEdgeNode.y

           o <- nearestBdrEdgeNode (C)                    +
            \...             ___                         /            ____
             \......        /                           /            /
              \.........   /                      maxX /            /
        (B) -> o----------o <- leftMost (A)    (B) -> o------------o <- leftMost (C)
           maxX \          \  Hole                   /...........   \   Hole
                 \          \___                    /........        \_____
                  \                                /....
                   \                              /.
                    +                            o <- nearestBdrEdgeNode (A)
        */
    )
    {
      curBdrNode = curBdrNode->next;
      continue;
    }

    // point P is in the triangle

    curBdrNode = curBdrNode->next;
  } while (curBdrNode != originalNearestBdrEdgeNode)
}

void FistTriangulator::RemoveHoles()
{
  std::sort(_holes.begin(), _holes.end(), [](Node *left, Node *right) -> bool { return left->x < right->x; });

  for (const auto hole : _holes)
  {
    // try to find a bridge diagonal between the hole and the boundary
    Node *boundaryNode = FindBridge(_boundary, hole);
  }
}

double FistTriangulator::EvalSignedArea(double *points, uint32_t size)
{
  double area = 0.;
  for (uint32_t i = 0, j = size - 1 /* last */; i < size; j = i, ++i)
  {
    double x0 = points[config.dim * j], y0 = points[config.dim * j + 1], x1 = points[config.dim * i],
           y1 = points[config.dim * i + 1];
    area += (x0 - x1) * (y1 + y0);
  }
  return area;
}

double FistTriangulator::EvalSignedArea(const Node *const A, const Node *const B, const Node *const C)
{
  return (B->y - A->y) * (C->x - B->x) - (B->x - A->x) * (C->y - B->y);
}

bool FistTriangulator::LocallyInside(const Node *const base, const Node *const point)
{
  bool cw = EvalSignedArea(base->prev, base, base->next) < 0;

  /* true occasions:
    [area < 0]                  [area > 0]
     base        base->next                    base->prev
        +____>____+            .................+
       /...........          point -> *......../
      ↗............            ...............↙
     /........* <- point       ............../
    +..............            +-------<----*
    base->prev             base->next      base
  */
  if (cw)
    return EvalSignedArea(base, point, base->next) >= 0 && EvalSignedArea(base, base->prev, point) >= 0;
  return EvalSignedArea(base, point, base->prev) < 0 || EvalSignedArea(base, base->next, point);
}

bool FistTriangulator::MiddleInside(const Node *const A, const Node *const B) const
{
  // test if a point (which is the midpoint of A and B) is in a polygon by counting the segments on its right
  double midX = (A->x + B->x) / 2, midY = (A->y + B->y) / 2;
  uint32_t depth = EvalDepth(A, midX, midY);

  return depth & 0b1; /* odd depth indicates internally */
}

uint32_t FistTriangulator::EvalDepth(const Node *const base, double x, double y) const
{
  const Node *curNode = base;
  uint32_t depth      = 0;
  do
  {
    if ((curNode->y > y) != (curNode->next->y > y)
        /* the horizontal line from (x, y) intersects the segment from curNode to curNode->next */
        && (x <
            curNode->x + (curNode->next->x - curNode->x) * (y - curNode->y) / (curNode->next->y - curNode->y))
        /* the segment lies on the right of (x, y). Certainly, we can also count the segments on the left by
           turning < to > */
    )
    {
      ++depth;
    }
  } while (curNode != base);

  return depth;
}

bool FistTriangulator::PointInTriangle(double Ax,
                                       double Ay,
                                       double Bx,
                                       double By,
                                       double Cx,
                                       double Cy,
                                       double Px,
                                       double Py) const
{
  // todo: the triangle is CW?
  bool res = (Cx - Px) * (Ay - Py) >= (Ax - Px) * (Cy - Py) &&
             (Ax - Px) * (By - Py) >= (Bx - Px) * (Ay - Py) && (Bx - Px) * (Cy - Py) >= (Cx - Px) * (By - Py);
  return res;
}

Node *FistTriangulator::InsertNode(uint32_t id, double x, double y, Node *last)
{
  // original: last --> last.next
  // functioned: last --> node --> last.next

  Node *node = _alloc.alloc(id, x, y);

  // no last node provided
  if (!last)
  {
    node->prev = node->next = node;
    return node;
  }

  // insert after last
  node->next       = last->next;
  node->prev       = last;
  node->next->prev = node;
  last->next       = node;
}

void FistTriangulator::RemoveNode(Node *node)
{
  // detach
  node->next->prev = node->prev;
  node->prev->next = node->next;

  if (node->prevZ)
    node->prevZ->nextZ = node->nextZ;
  if (node->nextZ)
    node->nextZ->prevZ = node->prevZ;
}