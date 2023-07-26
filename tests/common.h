#pragma once

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