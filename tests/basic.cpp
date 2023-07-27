#include <gtest/gtest.h>

#include "triangulator.h"

#include "common.h"

constexpr AnyID Nil = INVALID_INDEX, Inf = INFINITY_INDEX;

// Demonstrate some basic assertions.
TEST(BasicTest, DiamondClockwiseMergeRight)
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

TEST(BasicTest, BoatCounterclockwiseMergeLeft)
{
  Vec2Set points = {{53, 131}, {122, 238}, {204, 167}, {239, 269}, {93, 353}};

  Triangulator triangulator;
  triangulator.config.useGivenSeed = true;
  triangulator.config.seed         = 1;

  triangulator.AddPolygon(points, true);
  triangulator.Build();

  // check size
  EXPECT_EQ(triangulator._nodes.Size(), 25);
  EXPECT_EQ(triangulator._vertices.Size(), 5);
  EXPECT_EQ(triangulator._segments.Size(), 5);
  EXPECT_EQ(triangulator._regions.Size(), 12);
  EXPECT_EQ(triangulator._prevVertices.Size(), 5);
  EXPECT_EQ(triangulator._endVertices.Size(), 5);

  // check the trapezoid map
  const auto &node0 = triangulator._nodes[0];
  EXPECT_NODE(node0, 0, Node::VERTEX, 4, 2, 1);
  const auto &node1 = triangulator._nodes[1];
  EXPECT_NODE(node1, 1, Node::VERTEX, 3, 4, 3);
  const auto &node2 = triangulator._nodes[2];
  EXPECT_NODE(node2, 2, Node::REGION, 0, Nil, Nil);
  const auto &node3 = triangulator._nodes[3];
  EXPECT_NODE(node3, 3, Node::VERTEX, 1, 8, 7);
  const auto &node4 = triangulator._nodes[4];
  EXPECT_NODE(node4, 4, Node::SEGMENT, 3, 6, 5);
  const auto &node5 = triangulator._nodes[5];
  EXPECT_NODE(node5, 5, Node::REGION, 3, Nil, Nil);
  const auto &node6 = triangulator._nodes[6];
  EXPECT_NODE(node6, 6, Node::SEGMENT, 4, 18, 17);
  const auto &node7 = triangulator._nodes[7];
  EXPECT_NODE(node7, 7, Node::VERTEX, 2, 10, 9);
  const auto &node8 = triangulator._nodes[8];
  EXPECT_NODE(node8, 8, Node::SEGMENT, 2, 14, 13);
  const auto &node9 = triangulator._nodes[9];
  EXPECT_NODE(node9, 9, Node::VERTEX, 0, 16, 15);
  const auto &node10 = triangulator._nodes[10];
  EXPECT_NODE(node10, 10, Node::SEGMENT, 1, 12, 11);
  const auto &node11 = triangulator._nodes[11];
  EXPECT_NODE(node11, 11, Node::SEGMENT, 2, 15, 13);
  const auto &node12 = triangulator._nodes[12];
  EXPECT_NODE(node12, 12, Node::SEGMENT, 4, 18, 20);
  const auto &node13 = triangulator._nodes[13];
  EXPECT_NODE(node13, 13, Node::REGION, 7, Nil, Nil);
  const auto &node14 = triangulator._nodes[14];
  EXPECT_NODE(node14, 14, Node::SEGMENT, 4, 18, 19);
  const auto &node15 = triangulator._nodes[15];
  EXPECT_NODE(node15, 15, Node::REGION, 8, Nil, Nil);
  const auto &node16 = triangulator._nodes[16];
  EXPECT_NODE(node16, 16, Node::SEGMENT, 4, 18, 21);
  const auto &node17 = triangulator._nodes[17];
  EXPECT_NODE(node17, 17, Node::REGION, 9, Nil, Nil);
  const auto &node18 = triangulator._nodes[18];
  EXPECT_NODE(node18, 18, Node::REGION, 1, Nil, Nil);
  const auto &node19 = triangulator._nodes[19];
  EXPECT_NODE(node19, 19, Node::REGION, 2, Nil, Nil);
  const auto &node20 = triangulator._nodes[20];
  EXPECT_NODE(node20, 20, Node::SEGMENT, 0, 23, 22);
  const auto &node21 = triangulator._nodes[21];
  EXPECT_NODE(node21, 21, Node::SEGMENT, 0, 23, 24);
  const auto &node22 = triangulator._nodes[22];
  EXPECT_NODE(node22, 22, Node::REGION, 10, Nil, Nil);
  const auto &node23 = triangulator._nodes[23];
  EXPECT_NODE(node23, 23, Node::REGION, 4, Nil, Nil);
  const auto &node24 = triangulator._nodes[24];
  EXPECT_NODE(node24, 24, Node::REGION, 5, Nil, Nil);

  // check low neighbors of vertices
  const auto &n0 = triangulator._lowNeighbors[0];
  EXPECT_LOW_NEIGHBOR(n0, 8, Nil, Nil);
  const auto &n1 = triangulator._lowNeighbors[1];
  EXPECT_LOW_NEIGHBOR(n1, 4, 10, 6);
  const auto &n2 = triangulator._lowNeighbors[2];
  EXPECT_LOW_NEIGHBOR(n2, 5, Nil, Nil);
  const auto &n3 = triangulator._lowNeighbors[3];
  EXPECT_LOW_NEIGHBOR(n3, 2, Nil, 7);
  const auto &n4 = triangulator._lowNeighbors[4];
  EXPECT_LOW_NEIGHBOR(n4, 1, 9, 3);

  // check regions
  const auto &s0 = triangulator._regions[0];
  EXPECT_REGION(s0, 2, Inf, 4, Inf, Inf, 1, 3, Inf, Inf);
  const auto &s1 = triangulator._regions[1];
  EXPECT_REGION(s1, 18, 4, 0, Inf, 4, 8, 8, 0, 0);
  const auto &s2 = triangulator._regions[2];
  EXPECT_REGION(s2, 19, 3, 1, 4, 2, 4, 6, 9, 9);
  const auto &s3 = triangulator._regions[3];
  EXPECT_REGION(s3, 5, 4, 3, 3, Inf, 7, 7, 0, 0);
  const auto &s4 = triangulator._regions[4];
  EXPECT_REGION(s4, 23, 1, 0, 1, 0, Nil, Nil, 2, 2);
  const auto &s5 = triangulator._regions[5];
  EXPECT_REGION(s5, 24, 2, 0, 0, Inf, 8, 8, 10, 7);
  const auto &s6 = triangulator._regions[6];
  EXPECT_REGION(s6, 15, 1, 2, 1, 2, Nil, Nil, 2, 2);
  const auto &s7 = triangulator._regions[7];
  EXPECT_REGION(s7, 13, 3, 2, 2, Inf, 5, 5, 3, 3);
  const auto &s8 = triangulator._regions[8];
  EXPECT_REGION(s8, 15, 0, Inf, Inf, Inf, Inf, Inf, 1, 5);
  const auto &s9 = triangulator._regions[9];
  EXPECT_REGION(s9, 17, 4, 3, 4, 3, 2, 2, Nil, Nil);
  const auto &s10 = triangulator._regions[10];
  EXPECT_REGION(s10, 22, 1, 2, 0, 1, 5, 5, Nil, Nil);

  auto results = triangulator.Triangulate();
  EXPECT_EQ(results.size(), 3);
}