#include "common.h"

// Demonstrate some basic assertions.
TEST(BasicTest, DiamondCWMergeRight)
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
  EXPECT_REGION(s0, 8, Inf, 1, Inf, Inf, 4, 6, Inf, Inf, 0);
  const auto &s1 = triangulator._regions[1];
  EXPECT_REGION(s1, 6, 0, 3, Inf, 3, 2, 2, 4, 4, 0);
  const auto &s2 = triangulator._regions[2];
  EXPECT_REGION(s2, 3, 3, Inf, Inf, Inf, Inf, Inf, 1, 7, 0);
  const auto &s3 = triangulator._regions[3];
  EXPECT_REGION(s3, 13, 0, 2, 3, 1, 5, 5, 8, 8, 1);
  const auto &s4 = triangulator._regions[4];
  EXPECT_REGION(s4, 17, 1, 0, Inf, 0, 1, 1, 0, 0, 0);
  const auto &s5 = triangulator._regions[5];
  EXPECT_REGION(s5, 15, 2, 3, 3, 2, Nil, Nil, 3, 3, 1);
  const auto &s6 = triangulator._regions[6];
  EXPECT_REGION(s6, 11, 1, 2, 1, Inf, 7, 7, 0, 0, 0);
  const auto &s7 = triangulator._regions[7];
  EXPECT_REGION(s7, 14, 2, 3, 2, Inf, 2, 2, 6, 6, 0);
  const auto &s8 = triangulator._regions[8];
  EXPECT_REGION(s8, 16, 1, 0, 0, 1, 3, 3, Nil, Nil, 1);

  auto results = triangulator.Triangulate();
  EXPECT_EQ(results.size(), 2);
}

TEST(BasicTest, BoatCCWMergeRight)
{
  Vec2Set points = {{0, 2}, {0, 1}, {-1, 0}, {4, -4}, {4, -1}};
  Triangulator triangulator;
  triangulator.config.useGivenSeed = true;
  triangulator.config.seed         = 1;
  triangulator.AddPolygon(points, true);
  triangulator.Build();

  // check size
  EXPECT_EQ(triangulator._nodes.Size(), 24);
  EXPECT_EQ(triangulator._vertices.Size(), 5);
  EXPECT_EQ(triangulator._segments.Size(), 5);
  EXPECT_EQ(triangulator._regions.Size(), 11);
  EXPECT_EQ(triangulator._prevVertices.Size(), 5);
  EXPECT_EQ(triangulator._endVertices.Size(), 5);

  // check the trapezoid map
  const auto &node0 = triangulator._nodes[0];
  EXPECT_NODE(node0, 0, Node::VERTEX, 4, 2, 1);
  const auto &node1 = triangulator._nodes[1];
  EXPECT_NODE(node1, 1, Node::VERTEX, 3, 4, 3);
  const auto &node2 = triangulator._nodes[2];
  EXPECT_NODE(node2, 2, Node::VERTEX, 1, 8, 7);
  const auto &node3 = triangulator._nodes[3];
  EXPECT_NODE(node3, 3, Node::REGION, 2, Nil, Nil);
  const auto &node4 = triangulator._nodes[4];
  EXPECT_NODE(node4, 4, Node::SEGMENT, 3, 6, 5);
  const auto &node5 = triangulator._nodes[5];
  EXPECT_NODE(node5, 5, Node::REGION, 3, Nil, Nil);
  const auto &node6 = triangulator._nodes[6];
  EXPECT_NODE(node6, 6, Node::SEGMENT, 2, 14, 15);
  const auto &node7 = triangulator._nodes[7];
  EXPECT_NODE(node7, 7, Node::VERTEX, 2, 10, 9);
  const auto &node8 = triangulator._nodes[8];
  EXPECT_NODE(node8, 8, Node::VERTEX, 0, 17, 16);
  const auto &node9 = triangulator._nodes[9];
  EXPECT_NODE(node9, 9, Node::SEGMENT, 2, 14, 13);
  const auto &node10 = triangulator._nodes[10];
  EXPECT_NODE(node10, 10, Node::SEGMENT, 1, 12, 11);
  const auto &node11 = triangulator._nodes[11];
  EXPECT_NODE(node11, 11, Node::SEGMENT, 4, 20, 18);
  const auto &node12 = triangulator._nodes[12];
  EXPECT_NODE(node12, 12, Node::REGION, 4, Nil, Nil);
  const auto &node13 = triangulator._nodes[13];
  EXPECT_NODE(node13, 13, Node::SEGMENT, 4, 21, 18);
  const auto &node14 = triangulator._nodes[14];
  EXPECT_NODE(node14, 14, Node::REGION, 5, Nil, Nil);
  const auto &node15 = triangulator._nodes[15];
  EXPECT_NODE(node15, 15, Node::REGION, 1, Nil, Nil);
  const auto &node16 = triangulator._nodes[16];
  EXPECT_NODE(node16, 16, Node::SEGMENT, 4, 19, 18);
  const auto &node17 = triangulator._nodes[17];
  EXPECT_NODE(node17, 17, Node::REGION, 0, Nil, Nil);
  const auto &node18 = triangulator._nodes[18];
  EXPECT_NODE(node18, 18, Node::REGION, 9, Nil, Nil);
  const auto &node19 = triangulator._nodes[19];
  EXPECT_NODE(node19, 19, Node::SEGMENT, 0, 23, 22);
  const auto &node20 = triangulator._nodes[20];
  EXPECT_NODE(node20, 20, Node::REGION, 6, Nil, Nil);
  const auto &node21 = triangulator._nodes[21];
  EXPECT_NODE(node21, 21, Node::REGION, 7, Nil, Nil);
  const auto &node22 = triangulator._nodes[22];
  EXPECT_NODE(node22, 22, Node::REGION, 10, Nil, Nil);
  const auto &node23 = triangulator._nodes[23];
  EXPECT_NODE(node23, 23, Node::REGION, 8, Nil, Nil);

  // check low neighbors of vertices
  const auto &n0 = triangulator._lowNeighbors[0];
  EXPECT_LOW_NEIGHBOR(n0, 8, 10, 9);
  const auto &n1 = triangulator._lowNeighbors[1];
  EXPECT_LOW_NEIGHBOR(n1, 4, Nil, 6);
  const auto &n2 = triangulator._lowNeighbors[2];
  EXPECT_LOW_NEIGHBOR(n2, 5, Nil, 7);
  const auto &n3 = triangulator._lowNeighbors[3];
  EXPECT_LOW_NEIGHBOR(n3, 2, Nil, Nil);
  const auto &n4 = triangulator._lowNeighbors[4];
  EXPECT_LOW_NEIGHBOR(n4, 1, Nil, 3);

  // check regions
  const auto &s0 = triangulator._regions[0];
  EXPECT_REGION(s0, 17, Inf, 0, Inf, Inf, 8, 9, Inf, Inf, 0);
  const auto &s1 = triangulator._regions[1];
  EXPECT_REGION(s1, 15, 4, 3, 2, 3, Nil, Nil, 7, 7, 1);
  const auto &s2 = triangulator._regions[2];
  EXPECT_REGION(s2, 3, 3, Inf, Inf, Inf, Inf, Inf, 5, 3, 0);
  const auto &s3 = triangulator._regions[3];
  EXPECT_REGION(s3, 5, 4, 3, 3, Inf, 2, 2, 9, 9, 0);
  const auto &s4 = triangulator._regions[4];
  EXPECT_REGION(s4, 12, 1, 2, Inf, 1, 5, 5, 8, 8, 0);
  const auto &s5 = triangulator._regions[5];
  EXPECT_REGION(s5, 14, 2, 3, Inf, 2, 2, 2, 4, 4, 0);
  const auto &s6 = triangulator._regions[6];
  EXPECT_REGION(s6, 20, 1, 2, 1, 4, 7, 7, 10, 10, 1);
  const auto &s7 = triangulator._regions[7];
  EXPECT_REGION(s7, 21, 2, 4, 2, 4, 1, 1, 6, 6, 1);
  const auto &s8 = triangulator._regions[8];
  EXPECT_REGION(s8, 23, 0, 1, Inf, 0, 4, 4, 0, 0, 0);
  const auto &s9 = triangulator._regions[9];
  EXPECT_REGION(s9, 18, 0, 4, 4, Inf, 3, 3, 0, 0, 0);
  const auto &s10 = triangulator._regions[10];
  EXPECT_REGION(s10, 22, 0, 1, 0, 4, 6, 6, Nil, Nil, 1);

  auto results = triangulator.Triangulate();
  EXPECT_EQ(results.size(), 3);
}

TEST(BasicTest, BoatCCWMergeRight2SameY)
{
  Vec2Set points = {{0, 2}, {0, 1}, {-1, 0}, {4, 0}, {4, 1}};
  Triangulator triangulator;
  triangulator.config.useGivenSeed = true;
  triangulator.config.seed         = 1;
  triangulator.AddPolygon(points, true);
  triangulator.Build();

  // check size
  EXPECT_EQ(triangulator._nodes.Size(), 23);
  EXPECT_EQ(triangulator._vertices.Size(), 5);
  EXPECT_EQ(triangulator._segments.Size(), 5);
  EXPECT_EQ(triangulator._regions.Size(), 11);
  EXPECT_EQ(triangulator._prevVertices.Size(), 5);
  EXPECT_EQ(triangulator._endVertices.Size(), 5);

  // check the trapezoid map
  const auto &node0 = triangulator._nodes[0];
  EXPECT_NODE(node0, 0, Node::VERTEX, 4, 2, 1);
  const auto &node1 = triangulator._nodes[1];
  EXPECT_NODE(node1, 1, Node::VERTEX, 3, 4, 3);
  const auto &node2 = triangulator._nodes[2];
  EXPECT_NODE(node2, 2, Node::VERTEX, 1, 8, 7);
  const auto &node3 = triangulator._nodes[3];
  EXPECT_NODE(node3, 3, Node::REGION, 2, Nil, Nil);
  const auto &node4 = triangulator._nodes[4];
  EXPECT_NODE(node4, 4, Node::SEGMENT, 3, 6, 5);
  const auto &node5 = triangulator._nodes[5];
  EXPECT_NODE(node5, 5, Node::REGION, 3, Nil, Nil);
  const auto &node6 = triangulator._nodes[6];
  EXPECT_NODE(node6, 6, Node::VERTEX, 2, 10, 9);
  const auto &node7 = triangulator._nodes[7];
  EXPECT_NODE(node7, 7, Node::SEGMENT, 1, 12, 11);
  const auto &node8 = triangulator._nodes[8];
  EXPECT_NODE(node8, 8, Node::VERTEX, 0, 17, 16);
  const auto &node9 = triangulator._nodes[9];
  EXPECT_NODE(node9, 9, Node::SEGMENT, 2, 15, 14);
  const auto &node10 = triangulator._nodes[10];
  EXPECT_NODE(node10, 10, Node::SEGMENT, 1, 12, 13);
  const auto &node11 = triangulator._nodes[11];
  EXPECT_NODE(node11, 11, Node::SEGMENT, 4, 20, 18);
  const auto &node12 = triangulator._nodes[12];
  EXPECT_NODE(node12, 12, Node::REGION, 4, Nil, Nil);
  const auto &node13 = triangulator._nodes[13];
  EXPECT_NODE(node13, 13, Node::REGION, 1, Nil, Nil);
  const auto &node14 = triangulator._nodes[14];
  EXPECT_NODE(node14, 14, Node::REGION, 7, Nil, Nil);
  const auto &node15 = triangulator._nodes[15];
  EXPECT_NODE(node15, 15, Node::REGION, 5, Nil, Nil);
  const auto &node16 = triangulator._nodes[16];
  EXPECT_NODE(node16, 16, Node::SEGMENT, 4, 19, 18);
  const auto &node17 = triangulator._nodes[17];
  EXPECT_NODE(node17, 17, Node::REGION, 0, Nil, Nil);
  const auto &node18 = triangulator._nodes[18];
  EXPECT_NODE(node18, 18, Node::REGION, 9, Nil, Nil);
  const auto &node19 = triangulator._nodes[19];
  EXPECT_NODE(node19, 19, Node::SEGMENT, 0, 22, 21);
  const auto &node20 = triangulator._nodes[20];
  EXPECT_NODE(node20, 20, Node::REGION, 6, Nil, Nil);
  const auto &node21 = triangulator._nodes[21];
  EXPECT_NODE(node21, 21, Node::REGION, 10, Nil, Nil);
  const auto &node22 = triangulator._nodes[22];
  EXPECT_NODE(node22, 22, Node::REGION, 8, Nil, Nil);

  // check low neighbors of vertices
  const auto &n0 = triangulator._lowNeighbors[0];
  EXPECT_LOW_NEIGHBOR(n0, 8, 10, 9);
  const auto &n1 = triangulator._lowNeighbors[1];
  EXPECT_LOW_NEIGHBOR(n1, 4, Nil, 6);
  const auto &n2 = triangulator._lowNeighbors[2];
  EXPECT_LOW_NEIGHBOR(n2, 5, Nil, 7);
  const auto &n3 = triangulator._lowNeighbors[3];
  EXPECT_LOW_NEIGHBOR(n3, 2, Nil, Nil);
  const auto &n4 = triangulator._lowNeighbors[4];
  EXPECT_LOW_NEIGHBOR(n4, 1, Nil, 3);

  // check regions
  const auto &s0 = triangulator._regions[0];
  EXPECT_REGION(s0, 17, Inf, 0, Inf, Inf, 8, 9, Inf, Inf, 0);
  const auto &s1 = triangulator._regions[1];
  EXPECT_REGION(s1, 13, 4, 2, 1, 3, 7, 7, 6, 6, 1);
  const auto &s2 = triangulator._regions[2];
  EXPECT_REGION(s2, 3, 3, Inf, Inf, Inf, Inf, Inf, 5, 3, 0);
  const auto &s3 = triangulator._regions[3];
  EXPECT_REGION(s3, 5, 4, 3, 3, Inf, 2, 2, 9, 9, 0);
  const auto &s4 = triangulator._regions[4];
  EXPECT_REGION(s4, 12, 1, 2, Inf, 1, 5, 5, 8, 8, 0);
  const auto &s5 = triangulator._regions[5];
  EXPECT_REGION(s5, 15, 2, 3, Inf, 2, 2, 2, 4, 4, 0);
  const auto &s6 = triangulator._regions[6];
  EXPECT_REGION(s6, 20, 1, 4, 1, 4, 1, 1, 10, 10, 1);
  const auto &s7 = triangulator._regions[7];
  EXPECT_REGION(s7, 14, 2, 3, 2, 3, Nil, Nil, 1, 1, 1);
  const auto &s8 = triangulator._regions[8];
  EXPECT_REGION(s8, 22, 0, 1, Inf, 0, 4, 4, 0, 0, 0);
  const auto &s9 = triangulator._regions[9];
  EXPECT_REGION(s9, 18, 0, 4, 4, Inf, 3, 3, 0, 0, 0);
  const auto &s10 = triangulator._regions[10];
  EXPECT_REGION(s10, 21, 0, 1, 0, 4, 6, 6, Nil, Nil, 1);

  auto results = triangulator.Triangulate();
  EXPECT_EQ(results.size(), 3);
}

TEST(BasicTest, BoatCCWMergeLeft)
{
  Vec2Set points = {{53, 131}, {122, 238}, {204, 167}, {239, 269}, {93, 353}};
  Triangulator triangulator;
  triangulator.config.useGivenSeed = true;
  triangulator.config.seed         = 1;

  triangulator.AddPolygon(points, true);
  triangulator.Build();

  // check size
  EXPECT_EQ(triangulator._nodes.Size(), 26);
  EXPECT_EQ(triangulator._vertices.Size(), 5);
  EXPECT_EQ(triangulator._segments.Size(), 5);
  EXPECT_EQ(triangulator._regions.Size(), 11);
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
  EXPECT_NODE(node6, 6, Node::SEGMENT, 4, 19, 18);
  const auto &node7 = triangulator._nodes[7];
  EXPECT_NODE(node7, 7, Node::VERTEX, 2, 10, 9);
  const auto &node8 = triangulator._nodes[8];
  EXPECT_NODE(node8, 8, Node::SEGMENT, 2, 14, 13);
  const auto &node9 = triangulator._nodes[9];
  EXPECT_NODE(node9, 9, Node::VERTEX, 0, 17, 16);
  const auto &node10 = triangulator._nodes[10];
  EXPECT_NODE(node10, 10, Node::SEGMENT, 1, 12, 11);
  const auto &node11 = triangulator._nodes[11];
  EXPECT_NODE(node11, 11, Node::SEGMENT, 2, 15, 13);
  const auto &node12 = triangulator._nodes[12];
  EXPECT_NODE(node12, 12, Node::SEGMENT, 4, 19, 21);
  const auto &node13 = triangulator._nodes[13];
  EXPECT_NODE(node13, 13, Node::REGION, 7, Nil, Nil);
  const auto &node14 = triangulator._nodes[14];
  EXPECT_NODE(node14, 14, Node::SEGMENT, 4, 19, 20);
  const auto &node15 = triangulator._nodes[15];
  EXPECT_NODE(node15, 15, Node::REGION, 6, Nil, Nil);
  const auto &node16 = triangulator._nodes[16];
  EXPECT_NODE(node16, 16, Node::REGION, 8, Nil, Nil);
  const auto &node17 = triangulator._nodes[17];
  EXPECT_NODE(node17, 17, Node::SEGMENT, 4, 19, 22);
  const auto &node18 = triangulator._nodes[18];
  EXPECT_NODE(node18, 18, Node::REGION, 9, Nil, Nil);
  const auto &node19 = triangulator._nodes[19];
  EXPECT_NODE(node19, 19, Node::REGION, 1, Nil, Nil);
  const auto &node20 = triangulator._nodes[20];
  EXPECT_NODE(node20, 20, Node::REGION, 2, Nil, Nil);
  const auto &node21 = triangulator._nodes[21];
  EXPECT_NODE(node21, 21, Node::SEGMENT, 0, 24, 23);
  const auto &node22 = triangulator._nodes[22];
  EXPECT_NODE(node22, 22, Node::SEGMENT, 0, 24, 25);
  const auto &node23 = triangulator._nodes[23];
  EXPECT_NODE(node23, 23, Node::REGION, 10, Nil, Nil);
  const auto &node24 = triangulator._nodes[24];
  EXPECT_NODE(node24, 24, Node::REGION, 4, Nil, Nil);
  const auto &node25 = triangulator._nodes[25];
  EXPECT_NODE(node25, 25, Node::REGION, 5, Nil, Nil);

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
  EXPECT_REGION(s0, 2, Inf, 4, Inf, Inf, 1, 3, Inf, Inf, 0);
  const auto &s1 = triangulator._regions[1];
  EXPECT_REGION(s1, 19, 4, 0, Inf, 4, 8, 8, 0, 0, 0);
  const auto &s2 = triangulator._regions[2];
  EXPECT_REGION(s2, 20, 3, 1, 4, 2, 4, 6, 9, 9, 1);
  const auto &s3 = triangulator._regions[3];
  EXPECT_REGION(s3, 5, 4, 3, 3, Inf, 7, 7, 0, 0, 0);
  const auto &s4 = triangulator._regions[4];
  EXPECT_REGION(s4, 24, 1, 0, 4, 0, Nil, Nil, 2, 2, 1);
  const auto &s5 = triangulator._regions[5];
  EXPECT_REGION(s5, 25, 2, 0, 0, Inf, 8, 8, 10, 7, 0);
  const auto &s6 = triangulator._regions[6];
  EXPECT_REGION(s6, 15, 1, 2, 1, 2, Nil, Nil, 2, 2, 1);
  const auto &s7 = triangulator._regions[7];
  EXPECT_REGION(s7, 13, 3, 2, 2, Inf, 5, 5, 3, 3, 0);
  const auto &s8 = triangulator._regions[8];
  EXPECT_REGION(s8, 16, 0, Inf, Inf, Inf, Inf, Inf, 1, 5, 0);
  const auto &s9 = triangulator._regions[9];
  EXPECT_REGION(s9, 18, 4, 3, 4, 3, 2, 2, Nil, Nil, 1);
  const auto &s10 = triangulator._regions[10];
  EXPECT_REGION(s10, 23, 1, 2, 0, 1, 5, 5, Nil, Nil, 0);

  auto results = triangulator.Triangulate();
  EXPECT_EQ(results.size(), 3);
}

TEST(BasicTest, ArrowSameYDiffX)
{
  Vec2Set points = {{-1, 0}, {0, 2}, {1, 0}, {0, 1}};

  Triangulator triangulator;
  triangulator.config.useGivenSeed = true;
  triangulator.config.seed         = 1;

  triangulator.AddPolygon(points, true);
  triangulator.Build();

  // check size
  EXPECT_EQ(triangulator._nodes.Size(), 21);
  EXPECT_EQ(triangulator._vertices.Size(), 4);
  EXPECT_EQ(triangulator._segments.Size(), 4);
  EXPECT_EQ(triangulator._regions.Size(), 9);
  EXPECT_EQ(triangulator._prevVertices.Size(), 4);
  EXPECT_EQ(triangulator._endVertices.Size(), 4);

  // check the trapezoid map
  const auto &node0 = triangulator._nodes[0];
  EXPECT_NODE(node0, 0, Node::VERTEX, 3, 2, 1);
  const auto &node1 = triangulator._nodes[1];
  EXPECT_NODE(node1, 1, Node::VERTEX, 0, 4, 3);
  const auto &node2 = triangulator._nodes[2];
  EXPECT_NODE(node2, 2, Node::VERTEX, 1, 8, 7);
  const auto &node3 = triangulator._nodes[3];
  EXPECT_NODE(node3, 3, Node::VERTEX, 2, 10, 9);
  const auto &node4 = triangulator._nodes[4];
  EXPECT_NODE(node4, 4, Node::SEGMENT, 3, 6, 5);
  const auto &node5 = triangulator._nodes[5];
  EXPECT_NODE(node5, 5, Node::SEGMENT, 1, 13, 11);
  const auto &node6 = triangulator._nodes[6];
  EXPECT_NODE(node6, 6, Node::SEGMENT, 0, 19, 20);
  const auto &node7 = triangulator._nodes[7];
  EXPECT_NODE(node7, 7, Node::SEGMENT, 1, 12, 11);
  const auto &node8 = triangulator._nodes[8];
  EXPECT_NODE(node8, 8, Node::REGION, 0, Nil, Nil);
  const auto &node9 = triangulator._nodes[9];
  EXPECT_NODE(node9, 9, Node::REGION, 5, Nil, Nil);
  const auto &node10 = triangulator._nodes[10];
  EXPECT_NODE(node10, 10, Node::SEGMENT, 1, 14, 11);
  const auto &node11 = triangulator._nodes[11];
  EXPECT_NODE(node11, 11, Node::REGION, 6, Nil, Nil);
  const auto &node12 = triangulator._nodes[12];
  EXPECT_NODE(node12, 12, Node::SEGMENT, 0, 19, 18);
  const auto &node13 = triangulator._nodes[13];
  EXPECT_NODE(node13, 13, Node::SEGMENT, 2, 16, 15);
  const auto &node14 = triangulator._nodes[14];
  EXPECT_NODE(node14, 14, Node::SEGMENT, 2, 17, 15);
  const auto &node15 = triangulator._nodes[15];
  EXPECT_NODE(node15, 15, Node::REGION, 7, Nil, Nil);
  const auto &node16 = triangulator._nodes[16];
  EXPECT_NODE(node16, 16, Node::REGION, 3, Nil, Nil);
  const auto &node17 = triangulator._nodes[17];
  EXPECT_NODE(node17, 17, Node::REGION, 2, Nil, Nil);
  const auto &node18 = triangulator._nodes[18];
  EXPECT_NODE(node18, 18, Node::REGION, 8, Nil, Nil);
  const auto &node19 = triangulator._nodes[19];
  EXPECT_NODE(node19, 19, Node::REGION, 4, Nil, Nil);
  const auto &node20 = triangulator._nodes[20];
  EXPECT_NODE(node20, 20, Node::REGION, 1, Nil, Nil);

  // check low neighbors of vertices
  const auto &n0 = triangulator._lowNeighbors[0];
  EXPECT_LOW_NEIGHBOR(n0, 2, Nil, Nil);
  const auto &n1 = triangulator._lowNeighbors[1];
  EXPECT_LOW_NEIGHBOR(n1, 4, 8, 6);
  const auto &n2 = triangulator._lowNeighbors[2];
  EXPECT_LOW_NEIGHBOR(n2, 5, Nil, Nil);
  const auto &n3 = triangulator._lowNeighbors[3];
  EXPECT_LOW_NEIGHBOR(n3, 1, 3, 7);

  // check regions
  const auto &s0 = triangulator._regions[0];
  EXPECT_REGION(s0, 8, Inf, 1, Inf, Inf, 4, 6, Inf, Inf, 0);
  const auto &s1 = triangulator._regions[1];
  EXPECT_REGION(s1, 20, 3, 0, 0, 3, Nil, Nil, 8, 8, 1);
  const auto &s2 = triangulator._regions[2];
  EXPECT_REGION(s2, 17, 0, 2, Inf, 2, 5, 5, 4, 3, 0);
  const auto &s3 = triangulator._regions[3];
  EXPECT_REGION(s3, 16, 3, 0, 3, 2, 2, 2, Nil, Nil, 0);
  const auto &s4 = triangulator._regions[4];
  EXPECT_REGION(s4, 19, 1, 0, Inf, 0, 2, 2, 0, 0, 0);
  const auto &s5 = triangulator._regions[5];
  EXPECT_REGION(s5, 9, 2, Inf, Inf, Inf, Inf, Inf, 2, 6, 0);
  const auto &s6 = triangulator._regions[6];
  EXPECT_REGION(s6, 11, 1, 2, 1, Inf, 5, 5, 0, 0, 0);
  const auto &s7 = triangulator._regions[7];
  EXPECT_REGION(s7, 15, 3, 2, 2, 1, Nil, Nil, 8, 8, 1);
  const auto &s8 = triangulator._regions[8];
  EXPECT_REGION(s8, 18, 1, 3, 0, 1, 1, 7, Nil, Nil, 1);
}

TEST(BasicTest, CrossMerge)
{
  //  |...........\.....|
  //  |.added.--->.\....|
  //  |.............\...|
  //  |-------------*---|
  //  |            /    |

  // unverified -> auto generated case
  Vec2Set points = {{67, 200},  {267, 315}, {170, 209}, {333, 200},
                    {167, 173}, {107, 117}, {276, 66},  {58, 85}};

  Triangulator triangulator;
  triangulator.config.useGivenSeed = true;
  triangulator.config.seed         = 1;

  triangulator.AddPolygon(points, true);
  triangulator.Build();

  // check size
  EXPECT_EQ(triangulator._nodes.Size(), 37);
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
  EXPECT_NODE(node2, 2, Node::VERTEX, 2, 14, 13);
  const auto &node3 = triangulator._nodes[3];
  EXPECT_NODE(node3, 3, Node::VERTEX, 5, 8, 7);
  const auto &node4 = triangulator._nodes[4];
  EXPECT_NODE(node4, 4, Node::SEGMENT, 3, 6, 5);
  const auto &node5 = triangulator._nodes[5];
  EXPECT_NODE(node5, 5, Node::REGION, 3, Nil, Nil);
  const auto &node6 = triangulator._nodes[6];
  EXPECT_NODE(node6, 6, Node::SEGMENT, 7, 22, 23);
  const auto &node7 = triangulator._nodes[7];
  EXPECT_NODE(node7, 7, Node::VERTEX, 6, 10, 9);
  const auto &node8 = triangulator._nodes[8];
  EXPECT_NODE(node8, 8, Node::SEGMENT, 7, 22, 24);
  const auto &node9 = triangulator._nodes[9];
  EXPECT_NODE(node9, 9, Node::REGION, 5, Nil, Nil);
  const auto &node10 = triangulator._nodes[10];
  EXPECT_NODE(node10, 10, Node::SEGMENT, 5, 12, 11);
  const auto &node11 = triangulator._nodes[11];
  EXPECT_NODE(node11, 11, Node::REGION, 6, Nil, Nil);
  const auto &node12 = triangulator._nodes[12];
  EXPECT_NODE(node12, 12, Node::VERTEX, 7, 20, 19);
  const auto &node13 = triangulator._nodes[13];
  EXPECT_NODE(node13, 13, Node::SEGMENT, 2, 16, 15);
  const auto &node14 = triangulator._nodes[14];
  EXPECT_NODE(node14, 14, Node::VERTEX, 1, 27, 26);
  const auto &node15 = triangulator._nodes[15];
  EXPECT_NODE(node15, 15, Node::REGION, 8, Nil, Nil);
  const auto &node16 = triangulator._nodes[16];
  EXPECT_NODE(node16, 16, Node::VERTEX, 0, 18, 17);
  const auto &node17 = triangulator._nodes[17];
  EXPECT_NODE(node17, 17, Node::SEGMENT, 7, 22, 21);
  const auto &node18 = triangulator._nodes[18];
  EXPECT_NODE(node18, 18, Node::SEGMENT, 0, 29, 30);
  const auto &node19 = triangulator._nodes[19];
  EXPECT_NODE(node19, 19, Node::SEGMENT, 6, 34, 33);
  const auto &node20 = triangulator._nodes[20];
  EXPECT_NODE(node20, 20, Node::SEGMENT, 7, 22, 25);
  const auto &node21 = triangulator._nodes[21];
  EXPECT_NODE(node21, 21, Node::REGION, 11, Nil, Nil);
  const auto &node22 = triangulator._nodes[22];
  EXPECT_NODE(node22, 22, Node::REGION, 9, Nil, Nil);
  const auto &node23 = triangulator._nodes[23];
  EXPECT_NODE(node23, 23, Node::REGION, 1, Nil, Nil);
  const auto &node24 = triangulator._nodes[24];
  EXPECT_NODE(node24, 24, Node::SEGMENT, 4, 36, 35);
  const auto &node25 = triangulator._nodes[25];
  EXPECT_NODE(node25, 25, Node::REGION, 4, Nil, Nil);
  const auto &node26 = triangulator._nodes[26];
  EXPECT_NODE(node26, 26, Node::SEGMENT, 0, 29, 28);
  const auto &node27 = triangulator._nodes[27];
  EXPECT_NODE(node27, 27, Node::REGION, 0, Nil, Nil);
  const auto &node28 = triangulator._nodes[28];
  EXPECT_NODE(node28, 28, Node::SEGMENT, 1, 32, 31);
  const auto &node29 = triangulator._nodes[29];
  EXPECT_NODE(node29, 29, Node::REGION, 12, Nil, Nil);
  const auto &node30 = triangulator._nodes[30];
  EXPECT_NODE(node30, 30, Node::REGION, 7, Nil, Nil);
  const auto &node31 = triangulator._nodes[31];
  EXPECT_NODE(node31, 31, Node::REGION, 14, Nil, Nil);
  const auto &node32 = triangulator._nodes[32];
  EXPECT_NODE(node32, 32, Node::REGION, 13, Nil, Nil);
  const auto &node33 = triangulator._nodes[33];
  EXPECT_NODE(node33, 33, Node::REGION, 15, Nil, Nil);
  const auto &node34 = triangulator._nodes[34];
  EXPECT_NODE(node34, 34, Node::REGION, 10, Nil, Nil);
  const auto &node35 = triangulator._nodes[35];
  EXPECT_NODE(node35, 35, Node::REGION, 16, Nil, Nil);
  const auto &node36 = triangulator._nodes[36];
  EXPECT_NODE(node36, 36, Node::REGION, 2, Nil, Nil);

  // check regions
  const auto &s0 = triangulator._regions[0];
  EXPECT_REGION(s0, 27, Inf, 1, Inf, Inf, 12, 14, Inf, Inf, 0);
  const auto &s1 = triangulator._regions[1];
  EXPECT_REGION(s1, 23, 3, 4, 7, 3, 2, 2, 11, 11, 1);
  const auto &s2 = triangulator._regions[2];
  EXPECT_REGION(s2, 36, 4, 5, 7, 4, 4, 4, 1, 1, 1);
  const auto &s3 = triangulator._regions[3];
  EXPECT_REGION(s3, 5, 3, 4, 3, Inf, 16, 16, 8, 8, 0);
  const auto &s4 = triangulator._regions[4];
  EXPECT_REGION(s4, 25, 5, 7, 7, 5, 15, 15, 2, 2, 1);
  const auto &s5 = triangulator._regions[5];
  EXPECT_REGION(s5, 9, 6, Inf, Inf, Inf, Inf, Inf, 10, 6, 0);
  const auto &s6 = triangulator._regions[6];
  EXPECT_REGION(s6, 11, 5, 6, 5, Inf, 5, 5, 16, 16, 0);
  const auto &s7 = triangulator._regions[7];
  EXPECT_REGION(s7, 30, 2, 0, 0, 2, 11, 11, 13, 13, 1);
  const auto &s8 = triangulator._regions[8];
  EXPECT_REGION(s8, 15, 2, 3, 2, Inf, 3, 3, 14, 14, 0);
  const auto &s9 = triangulator._regions[9];
  EXPECT_REGION(s9, 22, 0, 7, Inf, 7, 10, 10, 12, 12, 0);
  const auto &s10 = triangulator._regions[10];
  EXPECT_REGION(s10, 34, 7, 6, Inf, 6, 5, 5, 9, 9, 0);
  const auto &s11 = triangulator._regions[11];
  EXPECT_REGION(s11, 21, 0, 3, 7, 2, 1, 1, 7, 7, 1);
  const auto &s12 = triangulator._regions[12];
  EXPECT_REGION(s12, 29, 1, 0, Inf, 0, 9, 9, 0, 0, 0);
  const auto &s13 = triangulator._regions[13];
  EXPECT_REGION(s13, 32, 1, 2, 0, 1, 7, 7, Nil, Nil, 1);
  const auto &s14 = triangulator._regions[14];
  EXPECT_REGION(s14, 31, 1, 2, 1, Inf, 8, 8, 0, 0, 0);
  const auto &s15 = triangulator._regions[15];
  EXPECT_REGION(s15, 33, 7, 6, 6, 5, Nil, Nil, 4, 4, 1);
  const auto &s16 = triangulator._regions[16];
  EXPECT_REGION(s16, 35, 4, 5, 4, Inf, 6, 6, 3, 3, 0);

  // check low neighbors of vertices
  const auto &n0 = triangulator._lowNeighbors[0];
  EXPECT_LOW_NEIGHBOR(n0, 9, Nil, 11);
  const auto &n1 = triangulator._lowNeighbors[1];
  EXPECT_LOW_NEIGHBOR(n1, 12, 13, 14);
  const auto &n2 = triangulator._lowNeighbors[2];
  EXPECT_LOW_NEIGHBOR(n2, 7, Nil, 8);
  const auto &n3 = triangulator._lowNeighbors[3];
  EXPECT_LOW_NEIGHBOR(n3, 1, Nil, 3);
  const auto &n4 = triangulator._lowNeighbors[4];
  EXPECT_LOW_NEIGHBOR(n4, 2, Nil, 16);
  const auto &n5 = triangulator._lowNeighbors[5];
  EXPECT_LOW_NEIGHBOR(n5, 4, Nil, 6);
  const auto &n6 = triangulator._lowNeighbors[6];
  EXPECT_LOW_NEIGHBOR(n6, 5, Nil, Nil);
  const auto &n7 = triangulator._lowNeighbors[7];
  EXPECT_LOW_NEIGHBOR(n7, 10, Nil, 15);
}

TEST(BasicTest, RightButterfly)
{
  // unverified -> auto generated case
  Vec2Set points = {{45, 878}, {774, 973}, {925, 391}, {782, 85},
                    {764, 32}, {715, 529}, {686, 490}, {96, 150}};

  Triangulator triangulator;
  triangulator.config.useGivenSeed = true;
  triangulator.config.seed         = 1;

  triangulator.AddPolygon(points, true);
  triangulator.Build();

  // check size
  EXPECT_EQ(triangulator._nodes.Size(), 44);
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
  EXPECT_NODE(node2, 2, Node::VERTEX, 5, 8, 7);
  const auto &node3 = triangulator._nodes[3];
  EXPECT_NODE(node3, 3, Node::REGION, 2, Nil, Nil);
  const auto &node4 = triangulator._nodes[4];
  EXPECT_NODE(node4, 4, Node::SEGMENT, 3, 6, 5);
  const auto &node5 = triangulator._nodes[5];
  EXPECT_NODE(node5, 5, Node::REGION, 3, Nil, Nil);
  const auto &node6 = triangulator._nodes[6];
  EXPECT_NODE(node6, 6, Node::SEGMENT, 4, 42, 43);
  const auto &node7 = triangulator._nodes[7];
  EXPECT_NODE(node7, 7, Node::VERTEX, 6, 10, 9);
  const auto &node8 = triangulator._nodes[8];
  EXPECT_NODE(node8, 8, Node::VERTEX, 0, 18, 17);
  const auto &node9 = triangulator._nodes[9];
  EXPECT_NODE(node9, 9, Node::VERTEX, 2, 14, 13);
  const auto &node10 = triangulator._nodes[10];
  EXPECT_NODE(node10, 10, Node::SEGMENT, 5, 12, 11);
  const auto &node11 = triangulator._nodes[11];
  EXPECT_NODE(node11, 11, Node::SEGMENT, 1, 33, 30);
  const auto &node12 = triangulator._nodes[12];
  EXPECT_NODE(node12, 12, Node::SEGMENT, 7, 22, 23);
  const auto &node13 = triangulator._nodes[13];
  EXPECT_NODE(node13, 13, Node::SEGMENT, 2, 16, 15);
  const auto &node14 = triangulator._nodes[14];
  EXPECT_NODE(node14, 14, Node::SEGMENT, 7, 22, 24);
  const auto &node15 = triangulator._nodes[15];
  EXPECT_NODE(node15, 15, Node::REGION, 8, Nil, Nil);
  const auto &node16 = triangulator._nodes[16];
  EXPECT_NODE(node16, 16, Node::VERTEX, 7, 20, 19);
  const auto &node17 = triangulator._nodes[17];
  EXPECT_NODE(node17, 17, Node::SEGMENT, 7, 22, 21);
  const auto &node18 = triangulator._nodes[18];
  EXPECT_NODE(node18, 18, Node::VERTEX, 1, 27, 26);
  const auto &node19 = triangulator._nodes[19];
  EXPECT_NODE(node19, 19, Node::SEGMENT, 4, 42, 41);
  const auto &node20 = triangulator._nodes[20];
  EXPECT_NODE(node20, 20, Node::SEGMENT, 7, 22, 25);
  const auto &node21 = triangulator._nodes[21];
  EXPECT_NODE(node21, 21, Node::SEGMENT, 1, 32, 30);
  const auto &node22 = triangulator._nodes[22];
  EXPECT_NODE(node22, 22, Node::REGION, 9, Nil, Nil);
  const auto &node23 = triangulator._nodes[23];
  EXPECT_NODE(node23, 23, Node::REGION, 4, Nil, Nil);
  const auto &node24 = triangulator._nodes[24];
  EXPECT_NODE(node24, 24, Node::SEGMENT, 1, 34, 30);
  const auto &node25 = triangulator._nodes[25];
  EXPECT_NODE(node25, 25, Node::SEGMENT, 6, 36, 37);
  const auto &node26 = triangulator._nodes[26];
  EXPECT_NODE(node26, 26, Node::SEGMENT, 0, 29, 28);
  const auto &node27 = triangulator._nodes[27];
  EXPECT_NODE(node27, 27, Node::REGION, 0, Nil, Nil);
  const auto &node28 = triangulator._nodes[28];
  EXPECT_NODE(node28, 28, Node::SEGMENT, 1, 31, 30);
  const auto &node29 = triangulator._nodes[29];
  EXPECT_NODE(node29, 29, Node::REGION, 12, Nil, Nil);
  const auto &node30 = triangulator._nodes[30];
  EXPECT_NODE(node30, 30, Node::REGION, 14, Nil, Nil);
  const auto &node31 = triangulator._nodes[31];
  EXPECT_NODE(node31, 31, Node::REGION, 13, Nil, Nil);
  const auto &node32 = triangulator._nodes[32];
  EXPECT_NODE(node32, 32, Node::REGION, 11, Nil, Nil);
  const auto &node33 = triangulator._nodes[33];
  EXPECT_NODE(node33, 33, Node::SEGMENT, 4, 39, 38);
  const auto &node34 = triangulator._nodes[34];
  EXPECT_NODE(node34, 34, Node::SEGMENT, 6, 36, 35);
  const auto &node35 = triangulator._nodes[35];
  EXPECT_NODE(node35, 35, Node::SEGMENT, 4, 40, 38);
  const auto &node36 = triangulator._nodes[36];
  EXPECT_NODE(node36, 36, Node::REGION, 5, Nil, Nil);
  const auto &node37 = triangulator._nodes[37];
  EXPECT_NODE(node37, 37, Node::SEGMENT, 4, 40, 41);
  const auto &node38 = triangulator._nodes[38];
  EXPECT_NODE(node38, 38, Node::REGION, 16, Nil, Nil);
  const auto &node39 = triangulator._nodes[39];
  EXPECT_NODE(node39, 39, Node::REGION, 6, Nil, Nil);
  const auto &node40 = triangulator._nodes[40];
  EXPECT_NODE(node40, 40, Node::REGION, 15, Nil, Nil);
  const auto &node41 = triangulator._nodes[41];
  EXPECT_NODE(node41, 41, Node::REGION, 7, Nil, Nil);
  const auto &node42 = triangulator._nodes[42];
  EXPECT_NODE(node42, 42, Node::REGION, 10, Nil, Nil);
  const auto &node43 = triangulator._nodes[43];
  EXPECT_NODE(node43, 43, Node::REGION, 1, Nil, Nil);

  // check regions
  const auto &s0 = triangulator._regions[0];
  EXPECT_REGION(s0, 27, Inf, 1, Inf, Inf, 12, 14, Inf, Inf, 0);
  const auto &s1 = triangulator._regions[1];
  EXPECT_REGION(s1, 43, 3, 4, 4, 3, Nil, Nil, 7, 7, 1);
  const auto &s2 = triangulator._regions[2];
  EXPECT_REGION(s2, 3, 4, Inf, Inf, Inf, Inf, Inf, 10, 3, 0);
  const auto &s3 = triangulator._regions[3];
  EXPECT_REGION(s3, 5, 3, 4, 3, Inf, 2, 2, 8, 8, 0);
  const auto &s4 = triangulator._regions[4];
  EXPECT_REGION(s4, 23, 5, 6, 7, 5, 5, 5, 11, 11, 1);
  const auto &s5 = triangulator._regions[5];
  EXPECT_REGION(s5, 36, 6, 7, 7, 6, Nil, Nil, 4, 4, 1);
  const auto &s6 = triangulator._regions[6];
  EXPECT_REGION(s6, 39, 5, 6, 5, 4, 15, 15, Nil, Nil, 0);
  const auto &s7 = triangulator._regions[7];
  EXPECT_REGION(s7, 41, 2, 3, 4, 2, 1, 1, 16, 16, 1);
  const auto &s8 = triangulator._regions[8];
  EXPECT_REGION(s8, 15, 2, 3, 2, Inf, 3, 3, 14, 14, 0);
  const auto &s9 = triangulator._regions[9];
  EXPECT_REGION(s9, 22, 0, 7, Inf, 7, 10, 10, 12, 12, 0);
  const auto &s10 = triangulator._regions[10];
  EXPECT_REGION(s10, 42, 7, 4, Inf, 4, 2, 2, 9, 15, 0);
  const auto &s11 = triangulator._regions[11];
  EXPECT_REGION(s11, 32, 0, 5, 7, 1, 4, 16, 13, 13, 1);
  const auto &s12 = triangulator._regions[12];
  EXPECT_REGION(s12, 29, 1, 0, Inf, 0, 9, 9, 0, 0, 0);
  const auto &s13 = triangulator._regions[13];
  EXPECT_REGION(s13, 31, 1, 0, 0, 1, 11, 11, Nil, Nil, 1);
  const auto &s14 = triangulator._regions[14];
  EXPECT_REGION(s14, 30, 1, 2, 1, Inf, 8, 8, 0, 0, 0);
  const auto &s15 = triangulator._regions[15];
  EXPECT_REGION(s15, 40, 6, 7, 6, 4, 10, 10, 6, 6, 0);
  const auto &s16 = triangulator._regions[16];
  EXPECT_REGION(s16, 38, 5, 2, 4, 1, 7, 7, 11, 11, 1);

  // check low neighbors of vertices
  const auto &n0 = triangulator._lowNeighbors[0];
  EXPECT_LOW_NEIGHBOR(n0, 9, Nil, 11);
  const auto &n1 = triangulator._lowNeighbors[1];
  EXPECT_LOW_NEIGHBOR(n1, 12, 13, 14);
  const auto &n2 = triangulator._lowNeighbors[2];
  EXPECT_LOW_NEIGHBOR(n2, 7, Nil, 8);
  const auto &n3 = triangulator._lowNeighbors[3];
  EXPECT_LOW_NEIGHBOR(n3, 1, Nil, 3);
  const auto &n4 = triangulator._lowNeighbors[4];
  EXPECT_LOW_NEIGHBOR(n4, 2, Nil, Nil);
  const auto &n5 = triangulator._lowNeighbors[5];
  EXPECT_LOW_NEIGHBOR(n5, 4, 6, 16);
  const auto &n6 = triangulator._lowNeighbors[6];
  EXPECT_LOW_NEIGHBOR(n6, 5, Nil, 15);
  const auto &n7 = triangulator._lowNeighbors[7];
  EXPECT_LOW_NEIGHBOR(n7, 10, Nil, Nil);
}

TEST(BasicTest, LeftButterfly)
{
  // unverified -> auto generated case
  Vec2Set points = {{2, 661},   {87, 488},  {113, 997}, {709, 935}, {770, 831},
                    {392, 181}, {273, 256}, {164, 46},  {13, 299}};

  Triangulator triangulator;
  triangulator.config.useGivenSeed = true;
  triangulator.config.seed         = 1;

  triangulator.AddPolygon(points, true);
  triangulator.Build();

  // check size
  EXPECT_EQ(triangulator._nodes.Size(), 48);
  EXPECT_EQ(triangulator._vertices.Size(), 9);
  EXPECT_EQ(triangulator._segments.Size(), 9);
  EXPECT_EQ(triangulator._regions.Size(), 19);
  EXPECT_EQ(triangulator._prevVertices.Size(), 9);
  EXPECT_EQ(triangulator._endVertices.Size(), 9);

  // check the trapezoid map
  const auto &node0 = triangulator._nodes[0];
  EXPECT_NODE(node0, 0, Node::VERTEX, 3, 2, 1);
  const auto &node1 = triangulator._nodes[1];
  EXPECT_NODE(node1, 1, Node::VERTEX, 4, 4, 3);
  const auto &node2 = triangulator._nodes[2];
  EXPECT_NODE(node2, 2, Node::VERTEX, 2, 14, 13);
  const auto &node3 = triangulator._nodes[3];
  EXPECT_NODE(node3, 3, Node::VERTEX, 6, 8, 7);
  const auto &node4 = triangulator._nodes[4];
  EXPECT_NODE(node4, 4, Node::SEGMENT, 3, 6, 5);
  const auto &node5 = triangulator._nodes[5];
  EXPECT_NODE(node5, 5, Node::REGION, 3, Nil, Nil);
  const auto &node6 = triangulator._nodes[6];
  EXPECT_NODE(node6, 6, Node::SEGMENT, 1, 44, 45);
  const auto &node7 = triangulator._nodes[7];
  EXPECT_NODE(node7, 7, Node::VERTEX, 5, 10, 9);
  const auto &node8 = triangulator._nodes[8];
  EXPECT_NODE(node8, 8, Node::VERTEX, 8, 18, 17);
  const auto &node9 = triangulator._nodes[9];
  EXPECT_NODE(node9, 9, Node::VERTEX, 7, 20, 19);
  const auto &node10 = triangulator._nodes[10];
  EXPECT_NODE(node10, 10, Node::SEGMENT, 5, 12, 11);
  const auto &node11 = triangulator._nodes[11];
  EXPECT_NODE(node11, 11, Node::SEGMENT, 4, 42, 37);
  const auto &node12 = triangulator._nodes[12];
  EXPECT_NODE(node12, 12, Node::SEGMENT, 7, 22, 23);
  const auto &node13 = triangulator._nodes[13];
  EXPECT_NODE(node13, 13, Node::SEGMENT, 2, 16, 15);
  const auto &node14 = triangulator._nodes[14];
  EXPECT_NODE(node14, 14, Node::REGION, 0, Nil, Nil);
  const auto &node15 = triangulator._nodes[15];
  EXPECT_NODE(node15, 15, Node::REGION, 8, Nil, Nil);
  const auto &node16 = triangulator._nodes[16];
  EXPECT_NODE(node16, 16, Node::SEGMENT, 1, 44, 43);
  const auto &node17 = triangulator._nodes[17];
  EXPECT_NODE(node17, 17, Node::SEGMENT, 7, 22, 21);
  const auto &node18 = triangulator._nodes[18];
  EXPECT_NODE(node18, 18, Node::VERTEX, 0, 26, 25);
  const auto &node19 = triangulator._nodes[19];
  EXPECT_NODE(node19, 19, Node::REGION, 10, Nil, Nil);
  const auto &node20 = triangulator._nodes[20];
  EXPECT_NODE(node20, 20, Node::SEGMENT, 7, 22, 24);
  const auto &node21 = triangulator._nodes[21];
  EXPECT_NODE(node21, 21, Node::SEGMENT, 4, 41, 37);
  const auto &node22 = triangulator._nodes[22];
  EXPECT_NODE(node22, 22, Node::REGION, 9, Nil, Nil);
  const auto &node23 = triangulator._nodes[23];
  EXPECT_NODE(node23, 23, Node::SEGMENT, 6, 35, 34);
  const auto &node24 = triangulator._nodes[24];
  EXPECT_NODE(node24, 24, Node::SEGMENT, 6, 35, 36);
  const auto &node25 = triangulator._nodes[25];
  EXPECT_NODE(node25, 25, Node::VERTEX, 1, 28, 27);
  const auto &node26 = triangulator._nodes[26];
  EXPECT_NODE(node26, 26, Node::SEGMENT, 4, 38, 37);
  const auto &node27 = triangulator._nodes[27];
  EXPECT_NODE(node27, 27, Node::SEGMENT, 8, 32, 33);
  const auto &node28 = triangulator._nodes[28];
  EXPECT_NODE(node28, 28, Node::SEGMENT, 0, 30, 29);
  const auto &node29 = triangulator._nodes[29];
  EXPECT_NODE(node29, 29, Node::SEGMENT, 4, 39, 37);
  const auto &node30 = triangulator._nodes[30];
  EXPECT_NODE(node30, 30, Node::SEGMENT, 8, 32, 31);
  const auto &node31 = triangulator._nodes[31];
  EXPECT_NODE(node31, 31, Node::REGION, 15, Nil, Nil);
  const auto &node32 = triangulator._nodes[32];
  EXPECT_NODE(node32, 32, Node::REGION, 12, Nil, Nil);
  const auto &node33 = triangulator._nodes[33];
  EXPECT_NODE(node33, 33, Node::SEGMENT, 4, 40, 37);
  const auto &node34 = triangulator._nodes[34];
  EXPECT_NODE(node34, 34, Node::REGION, 16, Nil, Nil);
  const auto &node35 = triangulator._nodes[35];
  EXPECT_NODE(node35, 35, Node::REGION, 4, Nil, Nil);
  const auto &node36 = triangulator._nodes[36];
  EXPECT_NODE(node36, 36, Node::REGION, 5, Nil, Nil);
  const auto &node37 = triangulator._nodes[37];
  EXPECT_NODE(node37, 37, Node::REGION, 17, Nil, Nil);
  const auto &node38 = triangulator._nodes[38];
  EXPECT_NODE(node38, 38, Node::SEGMENT, 1, 44, 46);
  const auto &node39 = triangulator._nodes[39];
  EXPECT_NODE(node39, 39, Node::SEGMENT, 1, 47, 46);
  const auto &node40 = triangulator._nodes[40];
  EXPECT_NODE(node40, 40, Node::REGION, 13, Nil, Nil);
  const auto &node41 = triangulator._nodes[41];
  EXPECT_NODE(node41, 41, Node::REGION, 11, Nil, Nil);
  const auto &node42 = triangulator._nodes[42];
  EXPECT_NODE(node42, 42, Node::REGION, 6, Nil, Nil);
  const auto &node43 = triangulator._nodes[43];
  EXPECT_NODE(node43, 43, Node::REGION, 18, Nil, Nil);
  const auto &node44 = triangulator._nodes[44];
  EXPECT_NODE(node44, 44, Node::REGION, 7, Nil, Nil);
  const auto &node45 = triangulator._nodes[45];
  EXPECT_NODE(node45, 45, Node::REGION, 1, Nil, Nil);
  const auto &node46 = triangulator._nodes[46];
  EXPECT_NODE(node46, 46, Node::REGION, 2, Nil, Nil);
  const auto &node47 = triangulator._nodes[47];
  EXPECT_NODE(node47, 47, Node::REGION, 14, Nil, Nil);

  // check regions
  const auto &s0 = triangulator._regions[0];
  EXPECT_REGION(s0, 14, Inf, 2, Inf, Inf, 7, 8, Inf, Inf, 0);
  const auto &s1 = triangulator._regions[1];
  EXPECT_REGION(s1, 45, 3, 4, 1, 3, 2, 2, 18, 18, 1);
  const auto &s2 = triangulator._regions[2];
  EXPECT_REGION(s2, 46, 4, 1, 1, 4, 13, 13, 1, 1, 1);
  const auto &s3 = triangulator._regions[3];
  EXPECT_REGION(s3, 5, 3, 4, 3, Inf, 17, 17, 8, 8, 0);
  const auto &s4 = triangulator._regions[4];
  EXPECT_REGION(s4, 35, 6, 7, 7, 6, Nil, Nil, 11, 11, 1);
  const auto &s5 = triangulator._regions[5];
  EXPECT_REGION(s5, 36, 5, 7, 6, Inf, 10, 10, 16, 17, 0);
  const auto &s6 = triangulator._regions[6];
  EXPECT_REGION(s6, 42, 6, 5, 5, 4, Nil, Nil, 11, 11, 1);
  const auto &s7 = triangulator._regions[7];
  EXPECT_REGION(s7, 44, 2, 0, Inf, 1, 12, 14, 0, 0, 0);
  const auto &s8 = triangulator._regions[8];
  EXPECT_REGION(s8, 15, 2, 3, 2, Inf, 3, 3, 0, 0, 0);
  const auto &s9 = triangulator._regions[9];
  EXPECT_REGION(s9, 22, 8, 7, Inf, 7, 10, 10, 12, 12, 0);
  const auto &s10 = triangulator._regions[10];
  EXPECT_REGION(s10, 19, 7, Inf, Inf, Inf, Inf, Inf, 9, 5, 0);
  const auto &s11 = triangulator._regions[11];
  EXPECT_REGION(s11, 41, 8, 6, 7, 4, 4, 6, 13, 13, 1);
  const auto &s12 = triangulator._regions[12];
  EXPECT_REGION(s12, 32, 0, 8, Inf, 8, 9, 9, 7, 7, 0);
  const auto &s13 = triangulator._regions[13];
  EXPECT_REGION(s13, 40, 1, 8, 8, 4, 11, 11, 15, 2, 1);
  const auto &s14 = triangulator._regions[14];
  EXPECT_REGION(s14, 47, 0, 1, 0, 1, Nil, Nil, 7, 7, 0);
  const auto &s15 = triangulator._regions[15];
  EXPECT_REGION(s15, 31, 0, 1, 8, 0, 13, 13, Nil, Nil, 1);
  const auto &s16 = triangulator._regions[16];
  EXPECT_REGION(s16, 34, 6, 5, 6, 5, 5, 5, Nil, Nil, 0);
  const auto &s17 = triangulator._regions[17];
  EXPECT_REGION(s17, 37, 4, 5, 4, Inf, 5, 5, 3, 3, 0);
  const auto &s18 = triangulator._regions[18];
  EXPECT_REGION(s18, 43, 2, 3, 1, 2, 1, 1, Nil, Nil, 1);

  // check low neighbors of vertices
  const auto &n0 = triangulator._lowNeighbors[0];
  EXPECT_LOW_NEIGHBOR(n0, 12, 15, 14);
  const auto &n1 = triangulator._lowNeighbors[1];
  EXPECT_LOW_NEIGHBOR(n1, 13, Nil, Nil);
  const auto &n2 = triangulator._lowNeighbors[2];
  EXPECT_LOW_NEIGHBOR(n2, 7, 18, 8);
  const auto &n3 = triangulator._lowNeighbors[3];
  EXPECT_LOW_NEIGHBOR(n3, 1, Nil, 3);
  const auto &n4 = triangulator._lowNeighbors[4];
  EXPECT_LOW_NEIGHBOR(n4, 2, Nil, 17);
  const auto &n5 = triangulator._lowNeighbors[5];
  EXPECT_LOW_NEIGHBOR(n5, 5, Nil, Nil);
  const auto &n6 = triangulator._lowNeighbors[6];
  EXPECT_LOW_NEIGHBOR(n6, 4, 16, 6);
  const auto &n7 = triangulator._lowNeighbors[7];
  EXPECT_LOW_NEIGHBOR(n7, 10, Nil, Nil);
  const auto &n8 = triangulator._lowNeighbors[8];
  EXPECT_LOW_NEIGHBOR(n8, 9, Nil, 11);
}

TEST(BasicTest, Triangle)
{
  // unverified -> auto generated case
  Vec2Set points = {{125, 26},  {638, 185}, {770, 350}, {971, 393},
                    {678, 441}, {530, 625}, {307, 907}, {155, 184}};

  Triangulator triangulator;
  triangulator.config.useGivenSeed = true;
  triangulator.config.seed         = 1;

  triangulator.AddPolygon(points, true);
  triangulator.Build();

  // check size
  EXPECT_EQ(triangulator._nodes.Size(), 39);
  EXPECT_EQ(triangulator._vertices.Size(), 8);
  EXPECT_EQ(triangulator._segments.Size(), 8);
  EXPECT_EQ(triangulator._regions.Size(), 17);
  EXPECT_EQ(triangulator._prevVertices.Size(), 8);
  EXPECT_EQ(triangulator._endVertices.Size(), 8);

  // check the trapezoid map
  const auto &node0 = triangulator._nodes[0];
  EXPECT_NODE(node0, 0, Node::VERTEX, 4, 2, 1);
  const auto &node1 = triangulator._nodes[1];
  EXPECT_NODE(node1, 1, Node::VERTEX, 3, 4, 3);
  const auto &node2 = triangulator._nodes[2];
  EXPECT_NODE(node2, 2, Node::VERTEX, 6, 8, 7);
  const auto &node3 = triangulator._nodes[3];
  EXPECT_NODE(node3, 3, Node::VERTEX, 2, 14, 13);
  const auto &node4 = triangulator._nodes[4];
  EXPECT_NODE(node4, 4, Node::SEGMENT, 3, 6, 5);
  const auto &node5 = triangulator._nodes[5];
  EXPECT_NODE(node5, 5, Node::REGION, 3, Nil, Nil);
  const auto &node6 = triangulator._nodes[6];
  EXPECT_NODE(node6, 6, Node::SEGMENT, 6, 31, 33);
  const auto &node7 = triangulator._nodes[7];
  EXPECT_NODE(node7, 7, Node::VERTEX, 5, 10, 9);
  const auto &node8 = triangulator._nodes[8];
  EXPECT_NODE(node8, 8, Node::REGION, 0, Nil, Nil);
  const auto &node9 = triangulator._nodes[9];
  EXPECT_NODE(node9, 9, Node::SEGMENT, 6, 31, 32);
  const auto &node10 = triangulator._nodes[10];
  EXPECT_NODE(node10, 10, Node::SEGMENT, 5, 12, 11);
  const auto &node11 = triangulator._nodes[11];
  EXPECT_NODE(node11, 11, Node::REGION, 6, Nil, Nil);
  const auto &node12 = triangulator._nodes[12];
  EXPECT_NODE(node12, 12, Node::SEGMENT, 6, 31, 30);
  const auto &node13 = triangulator._nodes[13];
  EXPECT_NODE(node13, 13, Node::VERTEX, 7, 18, 17);
  const auto &node14 = triangulator._nodes[14];
  EXPECT_NODE(node14, 14, Node::SEGMENT, 2, 16, 15);
  const auto &node15 = triangulator._nodes[15];
  EXPECT_NODE(node15, 15, Node::REGION, 8, Nil, Nil);
  const auto &node16 = triangulator._nodes[16];
  EXPECT_NODE(node16, 16, Node::SEGMENT, 6, 31, 34);
  const auto &node17 = triangulator._nodes[17];
  EXPECT_NODE(node17, 17, Node::VERTEX, 0, 20, 19);
  const auto &node18 = triangulator._nodes[18];
  EXPECT_NODE(node18, 18, Node::VERTEX, 1, 24, 23);
  const auto &node19 = triangulator._nodes[19];
  EXPECT_NODE(node19, 19, Node::REGION, 10, Nil, Nil);
  const auto &node20 = triangulator._nodes[20];
  EXPECT_NODE(node20, 20, Node::SEGMENT, 7, 22, 21);
  const auto &node21 = triangulator._nodes[21];
  EXPECT_NODE(node21, 21, Node::SEGMENT, 0, 27, 25);
  const auto &node22 = triangulator._nodes[22];
  EXPECT_NODE(node22, 22, Node::REGION, 9, Nil, Nil);
  const auto &node23 = triangulator._nodes[23];
  EXPECT_NODE(node23, 23, Node::SEGMENT, 0, 26, 25);
  const auto &node24 = triangulator._nodes[24];
  EXPECT_NODE(node24, 24, Node::SEGMENT, 1, 29, 28);
  const auto &node25 = triangulator._nodes[25];
  EXPECT_NODE(node25, 25, Node::REGION, 13, Nil, Nil);
  const auto &node26 = triangulator._nodes[26];
  EXPECT_NODE(node26, 26, Node::SEGMENT, 6, 31, 36);
  const auto &node27 = triangulator._nodes[27];
  EXPECT_NODE(node27, 27, Node::REGION, 11, Nil, Nil);
  const auto &node28 = triangulator._nodes[28];
  EXPECT_NODE(node28, 28, Node::REGION, 14, Nil, Nil);
  const auto &node29 = triangulator._nodes[29];
  EXPECT_NODE(node29, 29, Node::SEGMENT, 6, 31, 35);
  const auto &node30 = triangulator._nodes[30];
  EXPECT_NODE(node30, 30, Node::REGION, 15, Nil, Nil);
  const auto &node31 = triangulator._nodes[31];
  EXPECT_NODE(node31, 31, Node::REGION, 4, Nil, Nil);
  const auto &node32 = triangulator._nodes[32];
  EXPECT_NODE(node32, 32, Node::SEGMENT, 4, 38, 37);
  const auto &node33 = triangulator._nodes[33];
  EXPECT_NODE(node33, 33, Node::REGION, 1, Nil, Nil);
  const auto &node34 = triangulator._nodes[34];
  EXPECT_NODE(node34, 34, Node::REGION, 2, Nil, Nil);
  const auto &node35 = triangulator._nodes[35];
  EXPECT_NODE(node35, 35, Node::REGION, 7, Nil, Nil);
  const auto &node36 = triangulator._nodes[36];
  EXPECT_NODE(node36, 36, Node::REGION, 12, Nil, Nil);
  const auto &node37 = triangulator._nodes[37];
  EXPECT_NODE(node37, 37, Node::REGION, 16, Nil, Nil);
  const auto &node38 = triangulator._nodes[38];
  EXPECT_NODE(node38, 38, Node::REGION, 5, Nil, Nil);

  // check regions
  const auto &s0 = triangulator._regions[0];
  EXPECT_REGION(s0, 8, Inf, 6, Inf, Inf, 4, 6, Inf, Inf, 0);
  const auto &s1 = triangulator._regions[1];
  EXPECT_REGION(s1, 33, 4, 3, 6, 3, 2, 2, 5, 5, 1);
  const auto &s2 = triangulator._regions[2];
  EXPECT_REGION(s2, 34, 3, 2, 6, 2, 7, 7, 1, 1, 1);
  const auto &s3 = triangulator._regions[3];
  EXPECT_REGION(s3, 5, 4, 3, 3, Inf, 8, 8, 16, 16, 0);
  const auto &s4 = triangulator._regions[4];
  EXPECT_REGION(s4, 31, 6, 7, Inf, 6, 9, 9, 0, 0, 0);
  const auto &s5 = triangulator._regions[5];
  EXPECT_REGION(s5, 38, 5, 4, 6, 4, 1, 1, 15, 15, 1);
  const auto &s6 = triangulator._regions[6];
  EXPECT_REGION(s6, 11, 6, 5, 5, Inf, 16, 16, 0, 0, 0);
  const auto &s7 = triangulator._regions[7];
  EXPECT_REGION(s7, 35, 2, 1, 6, 1, 12, 12, 2, 2, 1);
  const auto &s8 = triangulator._regions[8];
  EXPECT_REGION(s8, 15, 3, 2, 2, Inf, 14, 14, 3, 3, 0);
  const auto &s9 = triangulator._regions[9];
  EXPECT_REGION(s9, 22, 7, 0, Inf, 7, 10, 10, 4, 4, 0);
  const auto &s10 = triangulator._regions[10];
  EXPECT_REGION(s10, 19, 0, Inf, Inf, Inf, Inf, Inf, 9, 13, 0);
  const auto &s11 = triangulator._regions[11];
  EXPECT_REGION(s11, 27, 7, 0, 7, 0, Nil, Nil, 12, 12, 1);
  const auto &s12 = triangulator._regions[12];
  EXPECT_REGION(s12, 36, 1, 7, 6, 0, 11, 11, 7, 7, 1);
  const auto &s13 = triangulator._regions[13];
  EXPECT_REGION(s13, 25, 1, 0, 0, Inf, 10, 10, 14, 14, 0);
  const auto &s14 = triangulator._regions[14];
  EXPECT_REGION(s14, 28, 2, 1, 1, Inf, 13, 13, 8, 8, 0);
  const auto &s15 = triangulator._regions[15];
  EXPECT_REGION(s15, 30, 6, 5, 6, 5, 5, 5, Nil, Nil, 1);
  const auto &s16 = triangulator._regions[16];
  EXPECT_REGION(s16, 37, 5, 4, 4, Inf, 3, 3, 6, 6, 0);

  // check low neighbors of vertices
  const auto &n0 = triangulator._lowNeighbors[0];
  EXPECT_LOW_NEIGHBOR(n0, 10, Nil, Nil);
  const auto &n1 = triangulator._lowNeighbors[1];
  EXPECT_LOW_NEIGHBOR(n1, 12, Nil, 13);
  const auto &n2 = triangulator._lowNeighbors[2];
  EXPECT_LOW_NEIGHBOR(n2, 7, Nil, 14);
  const auto &n3 = triangulator._lowNeighbors[3];
  EXPECT_LOW_NEIGHBOR(n3, 2, Nil, 8);
  const auto &n4 = triangulator._lowNeighbors[4];
  EXPECT_LOW_NEIGHBOR(n4, 1, Nil, 3);
  const auto &n5 = triangulator._lowNeighbors[5];
  EXPECT_LOW_NEIGHBOR(n5, 5, Nil, 16);
  const auto &n6 = triangulator._lowNeighbors[6];
  EXPECT_LOW_NEIGHBOR(n6, 4, 15, 6);
  const auto &n7 = triangulator._lowNeighbors[7];
  EXPECT_LOW_NEIGHBOR(n7, 9, Nil, 11);
}
