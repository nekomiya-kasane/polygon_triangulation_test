#include "primitives.h"

#include <sstream>

constexpr const char *const regionFiller()
{
  char *const result = new char[sizeof(Region)];

  for (size_t i = 0; i < sizeof(Region); ++i)
    result[i] = '\0';

#define _FILL_DATA_(TYPE, MEMBER, VAL) *(TYPE *)&result[offsetof(Region, MEMBER)] = VAL;
  _FILL_DATA_(NodeID, nodeID, INVALID_INDEX);

  _FILL_DATA_(VertexID, high, INVALID_INDEX);
  _FILL_DATA_(VertexID, low, INVALID_INDEX);

  _FILL_DATA_(SegmentID, left, INVALID_INDEX);
  _FILL_DATA_(SegmentID, right, INVALID_INDEX);

  _FILL_DATA_(Depth, depth, INVALID_DEPTH);

  _FILL_DATA_(RegionID, highNeighbors[0], INVALID_INDEX);
  _FILL_DATA_(RegionID, highNeighbors[1], INVALID_INDEX);
  _FILL_DATA_(RegionID, lowNeighbors[0], INVALID_INDEX);
  _FILL_DATA_(RegionID, lowNeighbors[1], INVALID_INDEX);
#undef _FILL_DATA_

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

std::string Node::ToString() const
{
  std::stringstream ss;
  ss << "(" << id << ", ";
  switch (type)
  {
    case Node::NONE:
      ss << "None, ";
      break;
    case Node::VERTEX:
      ss << "Vertex, ";
      break;
    case Node::SEGMENT:
      ss << "Segment, ";
      break;
    case Node::REGION:
      ss << "Region, ";
      break;
  }
  ss << value << ", [" << left << ", " << right << "])";
  return ss.str();
}

std::string Region::ToString() const
{
  std::stringstream ss;
  ss << "(Node(" << nodeID << "), Vert(" << high << ", " << low << "), Edge(" << left << ", " << right
     << "), Low(" << lowNeighbors[0] << ", " << lowNeighbors[1] << "), High(" << highNeighbors[0] << ", "
     << highNeighbors[1] << "), Depth(" << depth << "))";
  return ss.str();
}

std::string Segment::ToString() const
{
  std::stringstream ss;
  ss << "((" << highVertex << ", " << lowVertex << ", " << downward << "), (" << from() << ", " << to()
     << "))";

  return ss.str();
}