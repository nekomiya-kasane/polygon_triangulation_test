#include "primitives.h"

constexpr const char *const regionFiller()
{
  char *const result = new char[sizeof(Region)];

  for (size_t i = 0; i < sizeof(Region); ++i)
    result[i] = '\0';

  *(NodeID *)result = INVALID_INDEX;

  auto kk = offsetof(Region, high);

  *(VertexID *)&result[offsetof(Region, high)] = INVALID_INDEX;
  *(VertexID *)&result[offsetof(Region, low)]  = INVALID_INDEX;

  *(SegmentID *)&result[offsetof(Region, left)]  = INVALID_INDEX;
  *(SegmentID *)&result[offsetof(Region, right)] = INVALID_INDEX;

  *(Depth *)&result[offsetof(Region, depth)] = INVALID_DEPTH;

  *(RegionID *)&result[offsetof(Region, highNeighbors[0])] = INVALID_INDEX;
  *(RegionID *)&result[offsetof(Region, highNeighbors[1])] = INVALID_INDEX;
  *(RegionID *)&result[offsetof(Region, lowNeighbors[0])]  = INVALID_INDEX;
  *(RegionID *)&result[offsetof(Region, lowNeighbors[1])]  = INVALID_INDEX;

  return result;
}

#pragma warning(push)
#pragma warning(disable : 26495)
Region::Region()
{
  static const char *const data = regionFiller();
  memcpy(this, data, sizeof(Region));
}
#pragma warning(pop)