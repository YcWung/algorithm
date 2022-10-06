#pragma once
#include "ntree.hpp"
#include "geom.hpp"

namespace alg
{

namespace octree
{

class OcTreeNode
{
    using Node = OcTreeNode;

public:
    Cube cube;

    void SetChild(int i, Node *node);
    std::vector<Node *> GetChildren();
};

class OcTree : public ntree::NTree<OcTreeNode, 8>
{
};

} // namespace octree

} // namespace alg