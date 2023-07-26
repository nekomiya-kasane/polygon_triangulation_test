#include <gtest/gtest.h>

#include "triangulator.h"

#include "common.h"

// Demonstrate some basic assertions.
TEST(BasicTest, DiamondClockwise)
{
  Vec2Set points = {{-1, 1}, {0, 2}, {1, 0}, {0, -2}};

  Triangulator triangulator;
  triangulator.config.useGivenSeed = true;
  triangulator.config.seed         = 1;

  triangulator.AddPolygon(points, true);
  triangulator.Build();

  EXPECT_EQ(triangulator._nodes.Size(), 18);
  EXPECT_EQ(triangulator._vertices.Size(), 4);
  EXPECT_EQ(triangulator._segments.Size(), 4);
  EXPECT_EQ(triangulator._regions.Size(), 9);
  EXPECT_EQ(triangulator._prevVertices.Size(), 4);
  EXPECT_EQ(triangulator._endVertices.Size(), 4);

  const auto &node0 = triangulator._nodes[0];
  EXPECT_NODE(node0, 0, Node::VERTEX, 0, 2, 1);
  const auto &node1 = triangulator._nodes[1];
  EXPECT_NODE(node1, 1, Node::VERTEX, 3, 4, 3);
  const auto &node2 = triangulator._nodes[2];
  EXPECT_NODE(node2, 2, Node::VERTEX, 1, 8, 7);
  const auto &node3 = triangulator._nodes[3];
  EXPECT_NODE(node3, 3, Node::REGION, 2, INVALID_INDEX, INVALID_INDEX);
  const auto &node4 = triangulator._nodes[4];
  EXPECT_NODE(node4, 4, Node::SEGMENT, 3, 6, 5);
  const auto &node5 = triangulator._nodes[5];
  EXPECT_NODE(node5, 5, Node::VERTEX, 2, 10, 9);
  const auto &node6 = triangulator._nodes[6];
  EXPECT_NODE(node6, 6, Node::REGION, 1, INVALID_INDEX, INVALID_INDEX);
  const auto &node7 = triangulator._nodes[7];
  EXPECT_NODE(node7, 7, Node::SEGMENT, 1, 12, 11);
  const auto &node8 = triangulator._nodes[8];
  EXPECT_NODE(node8, 8, Node::REGION, 0, INVALID_INDEX, INVALID_INDEX);
  const auto &node9 = triangulator._nodes[9];
  EXPECT_NODE(node9, 9, Node::SEGMENT, 2, 15, 14);
  const auto &node10 = triangulator._nodes[10];
  EXPECT_NODE(node10, 10, Node::SEGMENT, 1, 13, 11);
  const auto &node11 = triangulator._nodes[11];
  EXPECT_NODE(node11, 11, Node::REGION, 6, INVALID_INDEX, INVALID_INDEX);
  const auto &node12 = triangulator._nodes[12];
  EXPECT_NODE(node12, 12, Node::SEGMENT, 0, 17, 16);
  const auto &node13 = triangulator._nodes[13];
  EXPECT_NODE(node13, 13, Node::REGION, 3, INVALID_INDEX, INVALID_INDEX);
  const auto &node14 = triangulator._nodes[14];
  EXPECT_NODE(node14, 14, Node::REGION, 7, INVALID_INDEX, INVALID_INDEX);
  const auto &node15 = triangulator._nodes[15];
  EXPECT_NODE(node15, 15, Node::REGION, 5, INVALID_INDEX, INVALID_INDEX);
  const auto &node16 = triangulator._nodes[16];
  EXPECT_NODE(node16, 16, Node::REGION, 8, INVALID_INDEX, INVALID_INDEX);
  const auto &node17 = triangulator._nodes[17];
  EXPECT_NODE(node17, 17, Node::REGION, 4, INVALID_INDEX, INVALID_INDEX);

  // todo: validate other data.

  auto results = triangulator.Triangulate();
  EXPECT_EQ(results.size(), 2);
}