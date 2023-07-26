#include <gtest/gtest.h>

#include "triangulator.h"

#include "common.h"

constexpr AnyID Nil = INVALID_INDEX, Inf = INFINITY_INDEX;

// Demonstrate some basic assertions.
TEST(BasicTest, DiamondClockwise)
{
  Vec2Set points = {{-1, 1}, {0, 2}, {1, 0}, {0, -2}};

  Triangulator triangulator;
  triangulator.config.useGivenSeed = true;
  triangulator.config.seed         = 1;

  triangulator.AddPolygon(points, true);
  triangulator.Build();

  // check size
  EXPECT_EQ(triangulator._nodes.Size(), 18);
  EXPECT_EQ(triangulator._vertices.Size(), 4);
  EXPECT_EQ(triangulator._segments.Size(), 4);
  EXPECT_EQ(triangulator._regions.Size(), 9);
  EXPECT_EQ(triangulator._prevVertices.Size(), 4);
  EXPECT_EQ(triangulator._endVertices.Size(), 4);

  // check the trapezoid map
  const auto &node0 = triangulator._nodes[0];
  EXPECT_NODE(node0, 0, Node::VERTEX, 0, 2, 1);
  const auto &node1 = triangulator._nodes[1];
  EXPECT_NODE(node1, 1, Node::VERTEX, 3, 4, 3);
  const auto &node2 = triangulator._nodes[2];
  EXPECT_NODE(node2, 2, Node::VERTEX, 1, 8, 7);
  const auto &node3 = triangulator._nodes[3];
  EXPECT_NODE(node3, 3, Node::REGION, 2, Nil, Nil);
  const auto &node4 = triangulator._nodes[4];
  EXPECT_NODE(node4, 4, Node::SEGMENT, 3, 6, 5);
  const auto &node5 = triangulator._nodes[5];
  EXPECT_NODE(node5, 5, Node::VERTEX, 2, 10, 9);
  const auto &node6 = triangulator._nodes[6];
  EXPECT_NODE(node6, 6, Node::REGION, 1, Nil, Nil);
  const auto &node7 = triangulator._nodes[7];
  EXPECT_NODE(node7, 7, Node::SEGMENT, 1, 12, 11);
  const auto &node8 = triangulator._nodes[8];
  EXPECT_NODE(node8, 8, Node::REGION, 0, Nil, Nil);
  const auto &node9 = triangulator._nodes[9];
  EXPECT_NODE(node9, 9, Node::SEGMENT, 2, 15, 14);
  const auto &node10 = triangulator._nodes[10];
  EXPECT_NODE(node10, 10, Node::SEGMENT, 1, 13, 11);
  const auto &node11 = triangulator._nodes[11];
  EXPECT_NODE(node11, 11, Node::REGION, 6, Nil, Nil);
  const auto &node12 = triangulator._nodes[12];
  EXPECT_NODE(node12, 12, Node::SEGMENT, 0, 17, 16);
  const auto &node13 = triangulator._nodes[13];
  EXPECT_NODE(node13, 13, Node::REGION, 3, Nil, Nil);
  const auto &node14 = triangulator._nodes[14];
  EXPECT_NODE(node14, 14, Node::REGION, 7, Nil, Nil);
  const auto &node15 = triangulator._nodes[15];
  EXPECT_NODE(node15, 15, Node::REGION, 5, Nil, Nil);
  const auto &node16 = triangulator._nodes[16];
  EXPECT_NODE(node16, 16, Node::REGION, 8, Nil, Nil);
  const auto &node17 = triangulator._nodes[17];
  EXPECT_NODE(node17, 17, Node::REGION, 4, Nil, Nil);

  // check low neighbors of vertices
  const auto &n0 = triangulator._lowNeighbors[0];
  EXPECT_LOW_NEIGHBOR(n0, 1, Nil, 3);
  const auto &n1 = triangulator._lowNeighbors[1];
  EXPECT_LOW_NEIGHBOR(n1, 4, 8, 6);
  const auto &n2 = triangulator._lowNeighbors[2];
  EXPECT_LOW_NEIGHBOR(n2, 5, Nil, 7);
  const auto &n3 = triangulator._lowNeighbors[3];
  EXPECT_LOW_NEIGHBOR(n3, 2, Nil, Nil);

  // check regions
  const auto &s0 = triangulator._regions[0];
  EXPECT_REGION(s0, 8, Inf, 1, Inf, Inf, 4, 6, Inf, Inf);
  const auto &s1 = triangulator._regions[1];
  EXPECT_REGION(s1, 6, 0, 3, Inf, 3, 2, 2, 4, 4);
  const auto &s2 = triangulator._regions[2];
  EXPECT_REGION(s2, 3, 3, Inf, Inf, Inf, Inf, Inf, 1, 7);
  const auto &s3 = triangulator._regions[3];
  EXPECT_REGION(s3, 13, 0, 2, 3, 1, 5, 5, 8, 8);
  const auto &s4 = triangulator._regions[4];
  EXPECT_REGION(s4, 17, 1, 0, Inf, 0, 1, 1, 0, 0);
  const auto &s5 = triangulator._regions[5];
  EXPECT_REGION(s5, 15, 2, 3, 3, 2, Nil, Nil, 3, 3);
  const auto &s6 = triangulator._regions[6];
  EXPECT_REGION(s6, 11, 1, 2, 1, Inf, 7, 7, 0, 0);
  const auto &s7 = triangulator._regions[7];
  EXPECT_REGION(s7, 14, 2, 3, 2, Inf, 2, 2, 6, 6);

  auto results = triangulator.Triangulate();
  EXPECT_EQ(results.size(), 2);
}