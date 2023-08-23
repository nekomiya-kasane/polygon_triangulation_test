#pragma once

#include "IndexPool.hpp"
#include "vec2.h"

namespace Plasma
{

class TrapezoidMap
{
public:
  typedef int VertexId;
  typedef VertexId EdgeId;
  typedef int NodeId;
  typedef int RegionId;

  struct Node
  {
    NodeId Left;
    NodeId Right;

    // Value stored depends on node type
    // Vertex index is stored for vertex nodes, the index of the first
    // vertex of the edge is stored for edge nodes, the region index is stored
    // for region nodes
    int Value;

    enum NodeType
    {
      Vertex,
      Edge,
      Region
    };

    char Type;
  };

  // Trapezoidal region graph structure
  struct Region
  {
    // Region can have up to 2 region neighbors above and below
    RegionId TopNeighbor[2];
    RegionId BotNeighbor[2];

    // Left/Right bounding edges
    EdgeId LeftEdge;
    EdgeId RightEdge;

    // Top/bottom bounding vertices
    VertexId TopVertex;
    VertexId BotVertex;

    NodeId NodeIndex;

    // Tag each region to indicate whether it is inside or outside the shape
    int Depth;

    int WindingOrder;

    void SetTop(int left, int right)
    {
      TopNeighbor[0] = left;
      TopNeighbor[1] = right;
    }

    void SetTop(int top)
    {
      TopNeighbor[0] = top;
      TopNeighbor[1] = -1;
    }

    void SetBottom(int left, int right)
    {
      BotNeighbor[0] = left;
      BotNeighbor[1] = right;
    }

    void SetBottom(int bottom)
    {
      BotNeighbor[0] = bottom;
      BotNeighbor[1] = -1;
    }
  };

  struct BelowCache
  {
    RegionId Index[3];
  };

public:
  TrapezoidMap(const std::vector<Vec2> &vertices,
               const std::vector<unsigned int> &contours,
               int maxEdges,
               int seed);

  // Gets the region that the query point lies inside
  const Region *Query(const Vec2 &point) const;

  // Determines if an error occurred during map construction
  bool IsValid() const;

  // Binary tree to partition the shape into regions
  // Non-leaf nodes can be edges which partition space to the left
  // and right, or vertices which partition space up and down
  // Leaf nodes are "regions"
  NodeId mRoot;
  IndexPool<Node, NodeId> mNodes;

  // Forms a graph connecting all regions
  // Regions can have 0, 1, or 2 region neighbors above,
  // 0, 1, or 2 region neighbors below, and a left and right edge
  IndexPool<Region> mRegions;

  // Maps the corresponding edge index to the point with the higher
  // y value on that edge
  std::vector<VertexId> mTopIndex;
  std::vector<VertexId> mBotIndex;

  // Maps the given vertex to the contour index it lies on
  std::vector<VertexId> mContourId;

  // Maps the given edge index to the second point on that edge
  std::vector<VertexId> mEndIndex;
  std::vector<VertexId> mPrevIndex;

  // Copy of the input vertices
  std::vector<Vec2> mVertices;

  std::vector<VertexId> mFarthestEnd;
  std::vector<VertexId> mFarthestPrev;

  // Cache all regions below each vertex (there can be up to 3)
  std::vector<BelowCache> mRegionsBelow;

  // Map is built by inserting edges/vertices in a random order
  // Math::Random mRandom;

  std::vector<char> mWindingOrder;

  // Determines if an error occurred during map construction
  bool mIsValid;

  // Internal

  bool IsAboveInternal(VertexId a, VertexId b) const;

  const Region *QueryFrom(NodeId rootId, const Vec2 &point) const;
  const Region *QueryInternal(NodeId rootId, VertexId vertexId) const;

  NodeId MergeAbove(RegionId regionId, RegionId *parent);

  // Inserts an edge into the tree/graph dividing space on either side of that
  // edge
  bool InsertEdge(EdgeId edgeId);

  // Inserts a vertex into the tree/graph dividing space vertically
  void InsertVertex(NodeId rootId, VertexId vertexId);

  bool UpdateAbove(Region *originalCopy,
                   RegionId indexL,
                   RegionId indexR,
                   VertexId topEdgeIndex,
                   RegionId leftParent,
                   RegionId rightParent);

  bool UpdateBelow(Region *O,
                   RegionId indexL,
                   RegionId indexR,
                   EdgeId botEdgeIndex,
                   RegionId *,
                   RegionId *,
                   bool isLast);

  bool IsLeft(EdgeId edgeStart, EdgeId edgeEnd, const Vec2 &point) const;
  bool IsLeftInternal(EdgeId edgeStart, EdgeId edgeEnd, VertexId pointId) const;

  VertexId FindCommonTopPoint(RegionId indexL, RegionId indexR);
  RegionId FindRegionBelow(RegionId indexO, EdgeId edgeStart);
};

}  // namespace Plasma
