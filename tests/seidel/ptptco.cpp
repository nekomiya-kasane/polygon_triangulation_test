#include <gtest/gtest.h>

#include "seidel/triangulator.h"

#include "common.h"

TEST(PointCoincidence, OnePairPointCoCCW)
{
  // unverified -> auto generated case
  Vec2Set points = {{-2, 2}, {-2, -2}, {0, -2}, {-1, 0}, {1, 0}, {0, -2}, {2, -2}, {2, 2}};

  Triangulator triangulator;
  triangulator.config.useGivenSeed = true;
  triangulator.config.seed         = 1;

  triangulator.AddPolygon(points, true);
  triangulator.Build();

  // check size
  EXPECT_EQ(triangulator._nodes.Size(), 41);
  EXPECT_EQ(triangulator._vertices.Size(), 8);
  EXPECT_EQ(triangulator._segments.Size(), 8);
  EXPECT_EQ(triangulator._regions.Size(), 17);
  EXPECT_EQ(triangulator._prevVertices.Size(), 8);
  EXPECT_EQ(triangulator._endVertices.Size(), 8);

  // check the trapezoid map
  const auto &node0 = triangulator._nodes[0];
  EXPECT_NODE(node0, 0, Node::VERTEX, 3, 2, 1);
  const auto &node1 = triangulator._nodes[1];
  EXPECT_NODE(node1, 1, Node::VERTEX, 4, 4, 3);
  const auto &node2 = triangulator._nodes[2];
  EXPECT_NODE(node2, 2, Node::VERTEX, 0, 19, 18);
  const auto &node3 = triangulator._nodes[3];
  EXPECT_NODE(node3, 3, Node::VERTEX, 5, 8, 7);
  const auto &node4 = triangulator._nodes[4];
  EXPECT_NODE(node4, 4, Node::SEGMENT, 3, 6, 5);
  const auto &node5 = triangulator._nodes[5];
  EXPECT_NODE(node5, 5, Node::SEGMENT, 6, 34, 32);
  const auto &node6 = triangulator._nodes[6];
  EXPECT_NODE(node6, 6, Node::SEGMENT, 2, 16, 15);
  const auto &node7 = triangulator._nodes[7];
  EXPECT_NODE(node7, 7, Node::VERTEX, 6, 10, 9);
  const auto &node8 = triangulator._nodes[8];
  EXPECT_NODE(node8, 8, Node::VERTEX, 2, 14, 13);
  const auto &node9 = triangulator._nodes[9];
  EXPECT_NODE(node9, 9, Node::REGION, 5, Nil, Nil);
  const auto &node10 = triangulator._nodes[10];
  EXPECT_NODE(node10, 10, Node::SEGMENT, 5, 12, 11);
  const auto &node11 = triangulator._nodes[11];
  EXPECT_NODE(node11, 11, Node::SEGMENT, 6, 37, 32);
  const auto &node12 = triangulator._nodes[12];
  EXPECT_NODE(node12, 12, Node::REGION, 4, Nil, Nil);
  const auto &node13 = triangulator._nodes[13];
  EXPECT_NODE(node13, 13, Node::SEGMENT, 6, 36, 32);
  const auto &node14 = triangulator._nodes[14];
  EXPECT_NODE(node14, 14, Node::SEGMENT, 2, 16, 17);
  const auto &node15 = triangulator._nodes[15];
  EXPECT_NODE(node15, 15, Node::REGION, 8, Nil, Nil);
  const auto &node16 = triangulator._nodes[16];
  EXPECT_NODE(node16, 16, Node::VERTEX, 1, 25, 24);
  const auto &node17 = triangulator._nodes[17];
  EXPECT_NODE(node17, 17, Node::SEGMENT, 6, 35, 32);
  const auto &node18 = triangulator._nodes[18];
  EXPECT_NODE(node18, 18, Node::VERTEX, 7, 21, 20);
  const auto &node19 = triangulator._nodes[19];
  EXPECT_NODE(node19, 19, Node::REGION, 0, Nil, Nil);
  const auto &node20 = triangulator._nodes[20];
  EXPECT_NODE(node20, 20, Node::SEGMENT, 0, 27, 28);
  const auto &node21 = triangulator._nodes[21];
  EXPECT_NODE(node21, 21, Node::SEGMENT, 7, 23, 22);
  const auto &node22 = triangulator._nodes[22];
  EXPECT_NODE(node22, 22, Node::REGION, 11, Nil, Nil);
  const auto &node23 = triangulator._nodes[23];
  EXPECT_NODE(node23, 23, Node::SEGMENT, 0, 27, 26);
  const auto &node24 = triangulator._nodes[24];
  EXPECT_NODE(node24, 24, Node::SEGMENT, 1, 31, 30);
  const auto &node25 = triangulator._nodes[25];
  EXPECT_NODE(node25, 25, Node::SEGMENT, 0, 27, 29);
  const auto &node26 = triangulator._nodes[26];
  EXPECT_NODE(node26, 26, Node::REGION, 13, Nil, Nil);
  const auto &node27 = triangulator._nodes[27];
  EXPECT_NODE(node27, 27, Node::REGION, 9, Nil, Nil);
  const auto &node28 = triangulator._nodes[28];
  EXPECT_NODE(node28, 28, Node::SEGMENT, 6, 33, 32);
  const auto &node29 = triangulator._nodes[29];
  EXPECT_NODE(node29, 29, Node::REGION, 1, Nil, Nil);
  const auto &node30 = triangulator._nodes[30];
  EXPECT_NODE(node30, 30, Node::REGION, 14, Nil, Nil);
  const auto &node31 = triangulator._nodes[31];
  EXPECT_NODE(node31, 31, Node::REGION, 12, Nil, Nil);
  const auto &node32 = triangulator._nodes[32];
  EXPECT_NODE(node32, 32, Node::REGION, 15, Nil, Nil);
  const auto &node33 = triangulator._nodes[33];
  EXPECT_NODE(node33, 33, Node::REGION, 10, Nil, Nil);
  const auto &node34 = triangulator._nodes[34];
  EXPECT_NODE(node34, 34, Node::REGION, 3, Nil, Nil);
  const auto &node35 = triangulator._nodes[35];
  EXPECT_NODE(node35, 35, Node::SEGMENT, 4, 39, 38);
  const auto &node36 = triangulator._nodes[36];
  EXPECT_NODE(node36, 36, Node::SEGMENT, 4, 40, 38);
  const auto &node37 = triangulator._nodes[37];
  EXPECT_NODE(node37, 37, Node::REGION, 6, Nil, Nil);
  const auto &node38 = triangulator._nodes[38];
  EXPECT_NODE(node38, 38, Node::REGION, 16, Nil, Nil);
  const auto &node39 = triangulator._nodes[39];
  EXPECT_NODE(node39, 39, Node::REGION, 2, Nil, Nil);
  const auto &node40 = triangulator._nodes[40];
  EXPECT_NODE(node40, 40, Node::REGION, 7, Nil, Nil);

  // check regions
  const auto &s0 = triangulator._regions[0];
  EXPECT_REGION(s0, 19, Inf, 0, Inf, Inf, 9, 11, Inf, Inf, 0);
  const auto &s1 = triangulator._regions[1];
  EXPECT_REGION(s1, 29, 3, 1, 0, 2, 14, 14, 10, 10, 1);
  const auto &s2 = triangulator._regions[2];
  EXPECT_REGION(s2, 39, 4, 2, 2, 4, 7, 7, 8, 8, 0);
  const auto &s3 = triangulator._regions[3];
  EXPECT_REGION(s3, 34, 3, 4, 3, 6, 16, 16, 10, 10, 1);
  const auto &s4 = triangulator._regions[4];
  EXPECT_REGION(s4, 12, 5, 6, Inf, 5, 5, 5, 7, 7, 0);
  const auto &s5 = triangulator._regions[5];
  EXPECT_REGION(s5, 9, 6, Inf, Inf, Inf, Inf, Inf, 4, 15, 0);
  const auto &s6 = triangulator._regions[6];
  EXPECT_REGION(s6, 37, 5, 6, 5, 6, Nil, Nil, 16, 16, 1);
  const auto &s7 = triangulator._regions[7];
  EXPECT_REGION(s7, 40, 2, 5, Inf, 4, 4, 4, 12, 2, 0);
  const auto &s8 = triangulator._regions[8];
  EXPECT_REGION(s8, 15, 3, 4, 2, 3, 2, 2, Nil, Nil, 0);
  const auto &s9 = triangulator._regions[9];
  EXPECT_REGION(s9, 27, 0, 1, Inf, 0, 12, 12, 0, 0, 0);
  const auto &s10 = triangulator._regions[10];
  EXPECT_REGION(s10, 33, 7, 3, 0, 6, 1, 3, 13, 13, 1);
  const auto &s11 = triangulator._regions[11];
  EXPECT_REGION(s11, 22, 0, 7, 7, Inf, 15, 15, 0, 0, 0);
  const auto &s12 = triangulator._regions[12];
  EXPECT_REGION(s12, 31, 1, 2, Inf, 1, 7, 7, 9, 9, 0);
  const auto &s13 = triangulator._regions[13];
  EXPECT_REGION(s13, 26, 0, 7, 0, 7, 10, 10, Nil, Nil, 1);
  const auto &s14 = triangulator._regions[14];
  EXPECT_REGION(s14, 30, 1, 2, 1, 2, Nil, Nil, 1, 1, 1);
  const auto &s15 = triangulator._regions[15];
  EXPECT_REGION(s15, 32, 7, 6, 6, Inf, 5, 5, 11, 11, 0);
  const auto &s16 = triangulator._regions[16];
  EXPECT_REGION(s16, 38, 4, 5, 4, 6, 6, 6, 3, 3, 1);

  // check low neighbors of vertices
  const auto &n0 = triangulator._lowNeighbors[0];
  EXPECT_LOW_NEIGHBOR(n0, 9, 13, 11);
  const auto &n1 = triangulator._lowNeighbors[1];
  EXPECT_LOW_NEIGHBOR(n1, 12, Nil, 14);
  const auto &n2 = triangulator._lowNeighbors[2];
  EXPECT_LOW_NEIGHBOR(n2, 7, Nil, Nil);
  const auto &n3 = triangulator._lowNeighbors[3];
  EXPECT_LOW_NEIGHBOR(n3, 1, 8, 3);
  const auto &n4 = triangulator._lowNeighbors[4];
  EXPECT_LOW_NEIGHBOR(n4, 2, Nil, 16);
  const auto &n5 = triangulator._lowNeighbors[5];
  EXPECT_LOW_NEIGHBOR(n5, 4, Nil, 6);
  const auto &n6 = triangulator._lowNeighbors[6];
  EXPECT_LOW_NEIGHBOR(n6, 5, Nil, Nil);
  const auto &n7 = triangulator._lowNeighbors[7];
  EXPECT_LOW_NEIGHBOR(n7, 10, Nil, 15);
}