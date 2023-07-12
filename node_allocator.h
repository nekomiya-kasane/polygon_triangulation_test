#pragma once

#include "node.h"

class NodeAllocator
{
public:
  NodeAllocator(size_t nodeCount = 0)
  {
    if (nodeCount)
    {
      _dataNext = _data = (char *)malloc(nodeCount * NODE_SIZE);
      _dataTop          = _dataNext + nodeCount * NODE_SIZE;
    }
  }

  ~NodeAllocator()
  {
    free(_data);
    _data = _dataNext = _dataTop = nullptr;
  }

  template <Node::Type T, class... Args>
  inline NodeIndex CreateNode(const Args &...args) /* creating node may invalidate previous
                                                      pointers, hence index returned */
  {
    NodeTrait<T>::type *node = new (AllocNode()) NodeTrait<T>::type(_curNodeCount++, args);
    return node;
  }

  template <Node::Type T>
  inline NodeTrait<T>::type *RecastNode(Node *node)
  {
    NodeTrait<T>::type *transnode = reinterpret_cast<NodeTrait<T>::type *>(node);
    transnode->type               = T;
    return transnode;
  }

  template <Node::Type T>
  inline NodeTrait<T>::type *RecastNode(NodeIndex id)
  {
    return RecastNode<T>(GetNode(id));
  }

  template <Node::Type T, class... Args>
  inline NodeIndex DuplicateNode(Node *node)
  {
    Node *copy = AllocNode();
    NodeIndex newID = copy->nodeID;
    memcpy(copy, node, NODE_SIZE);
    copy->nodeID = newID;
    return copy->nodeID;
  }

  template <Node::Type T, class... Args>
  inline NodeIndex DuplicateNode(NodeIndex id)
  {
    return DuplicateNode<T>(GetNode(id));
  }

  template <Node::Type T = Node::NONE>
  inline NodeTrait<T>::type *GetNode(NodeIndex index)
  {
    return reinterpret_cast<NodeTrait<T>::type *>(_data + index * NODE_SIZE);
  }

  inline size_t GetCurrentNodeCount() const { return _curNodeCount; }

  inline void ReserveInc(size_t count)
  {
    size_t newSize = sizeof(char) * (_dataTop - _data) + count * NODE_SIZE;
    _data          = (char *)realloc(_data, newSize);
    _dataTop       = _data + newSize;
    _dataNext      = _data + _curNodeCount * NODE_SIZE;
  }

private:
  inline Node *AllocNode()
  {
    if (_dataNext == _dataTop)
    {
      ReserveInc(_curNodeCount);
    }
    char *res                             = _dataNext;
    reinterpret_cast<Node *>(res)->nodeID = ++_curNodeCount; /* from 1 on */
    _dataNext += NODE_SIZE;
    return (Node *)res;
  }
  mutable size_t _curNodeCount = 0;

  char *_data = nullptr, *_dataTop = nullptr, mutable *_dataNext = nullptr;
};