TEST(LineIntersection, DownDownRightCross)
{

  // unverified -> auto generated case
  Vec2Set points = {{-1, -0.5}, {-1, 1.5}, {1, -1}, {1, 1}};

  Triangulator triangulator;
  triangulator.config.useGivenSeed = true;
  triangulator.config.seed         = 1;

  triangulator.AddPolygon(points, true);
  triangulator.Build();

  // check size
  EXPECT_EQ(triangulator._nodes.Size(), 27);
  EXPECT_EQ(triangulator._vertices.Size(), 6);
  EXPECT_EQ(triangulator._segments.Size(), 6);
  EXPECT_EQ(triangulator._regions.Size(), 12);
  EXPECT_EQ(triangulator._prevVertices.Size(), 6);
  EXPECT_EQ(triangulator._endVertices.Size(), 6);

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
  EXPECT_NODE(node5, 5, Node::VERTEX, 5, 16, 15);
  const auto &node6 = triangulator._nodes[6];
  EXPECT_NODE(node6, 6, Node::VERTEX, 4, 14, 13);
  const auto &node7 = triangulator._nodes[7];
  EXPECT_NODE(node7, 7, Node::SEGMENT, 1, 12, 11);
  const auto &node8 = triangulator._nodes[8];
  EXPECT_NODE(node8, 8, Node::REGION, 0, Nil, Nil);
  const auto &node9 = triangulator._nodes[9];
  EXPECT_NODE(node9, 9, Node::REGION, 5, Nil, Nil);
  const auto &node10 = triangulator._nodes[10];
  EXPECT_NODE(node10, 10, Node::SEGMENT, 5, 21, 19);
  const auto &node11 = triangulator._nodes[11];
  EXPECT_NODE(node11, 11, Node::REGION, 6, Nil, Nil);
  const auto &node12 = triangulator._nodes[12];
  EXPECT_NODE(node12, 12, Node::SEGMENT, 0, 26, 25);
  const auto &node13 = triangulator._nodes[13];
  EXPECT_NODE(node13, 13, Node::SEGMENT, 0, 26, 27);
  const auto &node14 = triangulator._nodes[14];
  EXPECT_NODE(node14, 14, Node::SEGMENT, 1, 12, 18);
  const auto &node15 = triangulator._nodes[15];
  EXPECT_NODE(node15, 15, Node::SEGMENT, 5, 20, 19);
  const auto &node16 = triangulator._nodes[16];
  EXPECT_NODE(node16, 16, Node::SEGMENT, 2, 23, 22);
  const auto &node17 = triangulator._nodes[17];
  EXPECT_NODE(node17, 17, Node::REGION, 9, Nil, Nil);
  const auto &node18 = triangulator._nodes[18];
  EXPECT_NODE(node18, 18, Node::REGION, 1, Nil, Nil);
  const auto &node19 = triangulator._nodes[19];
  EXPECT_NODE(node19, 19, Node::SEGMENT, 2, 24, 22);
  const auto &node20 = triangulator._nodes[20];
  EXPECT_NODE(node20, 20, Node::REGION, 8, Nil, Nil);
  const auto &node21 = triangulator._nodes[21];
  EXPECT_NODE(node21, 21, Node::REGION, 2, Nil, Nil);
  const auto &node22 = triangulator._nodes[22];
  EXPECT_NODE(node22, 22, Node::REGION, 11, Nil, Nil);
  const auto &node23 = triangulator._nodes[23];
  EXPECT_NODE(node23, 23, Node::REGION, 3, Nil, Nil);
  const auto &node24 = triangulator._nodes[24];
  EXPECT_NODE(node24, 24, Node::REGION, 10, Nil, Nil);
  const auto &node25 = triangulator._nodes[25];
  EXPECT_NODE(node25, 25, Node::REGION, 12, Nil, Nil);
  const auto &node26 = triangulator._nodes[26];
  EXPECT_NODE(node26, 26, Node::REGION, 4, Nil, Nil);
  const auto &node27 = triangulator._nodes[27];
  EXPECT_NODE(node27, 27, Node::REGION, 7, Nil, Nil);

  // check regions
  const auto &s0 = triangulator._regions[0];
  EXPECT_REGION(s0, 8, Inf, 1, Inf, Inf, 4, 6, Inf, Inf, 0);
  const auto &s1 = triangulator._regions[1];
  EXPECT_REGION(s1, 18, 3, 4, 1, 3, 9, 9, 6, 6, 0);
  const auto &s2 = triangulator._regions[2];
  EXPECT_REGION(s2, 21, 0, 2, Inf, 5, 5, 5, 4, 8, 0);
  const auto &s3 = triangulator._regions[3];
  EXPECT_REGION(s3, 23, 3, 5, 3, 2, 10, 10, Nil, Nil, 1);
  const auto &s4 = triangulator._regions[4];
  EXPECT_REGION(s4, 26, 1, 0, Inf, 0, 2, 2, 0, 0, 0);
  const auto &s5 = triangulator._regions[5];
  EXPECT_REGION(s5, 9, 2, Inf, Inf, Inf, Inf, Inf, 2, 11, 0);
  const auto &s6 = triangulator._regions[6];
  EXPECT_REGION(s6, 11, 1, 3, 1, Inf, 1, 11, 0, 0, 0);
  const auto &s7 = triangulator._regions[7];
  EXPECT_REGION(s7, 27, 4, 0, 0, 4, Nil, Nil, 12, 12, 1);
  const auto &s8 = triangulator._regions[8];
  EXPECT_REGION(s8, 20, 5, 0, 4, 5, 2, 2, 9, 9, 0);
  const auto &s9 = triangulator._regions[9];
  EXPECT_REGION(s9, 17, 4, 5, 4, 3, 8, 8, 1, 1, 0);
  const auto &s10 = triangulator._regions[10];
  EXPECT_REGION(s10, 24, 5, 2, 5, 2, Nil, Nil, 3, 3, 1);
  const auto &s11 = triangulator._regions[11];
  EXPECT_REGION(s11, 22, 3, 2, 2, Inf, 5, 5, 6, 6, 0);
  const auto &s12 = triangulator._regions[12];
  EXPECT_REGION(s12, 25, 1, 4, 0, 1, 7, 7, Nil, Nil, 1);

  // check low neighbors of vertices
  const auto &n0 = triangulator._lowNeighbors[0];
  EXPECT_LOW_NEIGHBOR(n0, 2, Nil, Nil);
  const auto &n1 = triangulator._lowNeighbors[1];
  EXPECT_LOW_NEIGHBOR(n1, 4, 12, 6);
  const auto &n2 = triangulator._lowNeighbors[2];
  EXPECT_LOW_NEIGHBOR(n2, 5, Nil, Nil);
  const auto &n3 = triangulator._lowNeighbors[3];
  EXPECT_LOW_NEIGHBOR(n3, 1, 3, 11);
  const auto &n4 = triangulator._lowNeighbors[4];
  EXPECT_LOW_NEIGHBOR(n4, 7, Nil, 9);
  const auto &n5 = triangulator._lowNeighbors[5];
  EXPECT_LOW_NEIGHBOR(n5, 8, Nil, 10);
}