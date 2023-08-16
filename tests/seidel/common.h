#pragma once

#include <gtest/gtest.h>

#include "seidel/triangulator.h"

constexpr AnyID Nil = INVALID_INDEX, Inf = INFINITY_INDEX;

#define EXPECT_NODE(NODE, ID, TYPE, VAL, LEFT, RIGHT) \
  EXPECT_EQ(NODE.id, ID);                             \
  EXPECT_EQ(NODE.type, TYPE);                         \
  EXPECT_EQ(NODE.value, VAL);                         \
  EXPECT_EQ(NODE.left, LEFT);                         \
  EXPECT_EQ(NODE.right, RIGHT);

#define EXPECT_LOW_NEIGHBOR(INFO, LEFT, MID, RIGHT) \
  EXPECT_EQ(INFO.left, LEFT);                       \
  EXPECT_EQ(INFO.mid, MID);                         \
  EXPECT_EQ(INFO.right, RIGHT);

#define EXPECT_REGION(REGION, NODEID, VH, VL, SL, SR, LN0, LN1, HN0, HN1, DEPTH) \
  EXPECT_EQ(REGION.nodeID, NODEID);                                              \
  EXPECT_EQ(REGION.high, VH);                                                    \
  EXPECT_EQ(REGION.low, VL);                                                     \
  EXPECT_EQ(REGION.left, SL);                                                    \
  EXPECT_EQ(REGION.right, SR);                                                   \
  EXPECT_EQ(REGION.lowNeighbors[0], LN0);                                        \
  EXPECT_EQ(REGION.lowNeighbors[1], LN1);                                        \
  EXPECT_EQ(REGION.highNeighbors[0], HN0);                                       \
  EXPECT_EQ(REGION.highNeighbors[1], HN1);                                       \
  EXPECT_EQ(REGION.depth, DEPTH);
