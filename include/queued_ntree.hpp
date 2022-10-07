/**
 * @file queued_ntree.hpp
 * @author Yongchao Wang (ycw.puzzle@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2022-10-07
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <deque>
#include <functional>
#include <type_traits>
#include <memory>
#include "ntree.hpp"

namespace alg
{

namespace ntree
{

/**
 * @brief A n-branch tree with all nodes managed in a deque of unique_ptr.
 *
 * @tparam Node
 * @tparam N
 */
template <preorder_buildable_node_t Node, int N>
class QueuedNTree : public NTree<Node, N>
{
    using Parent = NTree<Node, N>;

public:
    void CreatePreOrder(typename Node::PreorderBuildInfo &info)
    {
        Parent::CreatePreOrder(info);
        Parent::TraversePreOrder([&](Node *n) { q.emplace_back(n); });
    }
    size_t GetNumberOfNodes() const { return q.size(); }

protected:
    std::deque<std::unique_ptr<Node>> q;
};

} // namespace ntree

} // namespace alg
