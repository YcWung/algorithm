#pragma once
#include <vector>

namespace alg
{

/**
 * @brief constraints of n-tree node type
 *
 * @tparam Node
 */
template <typename Node>
concept ntree_node_t =
    requires(Node *n, typename Node::BuildInfo info,
             std::vector<typename Node::BuildInfo> children_infos)
{
    typename Node::BuildInfo;
    // clang-format off
    { n->GetChildren() } -> std::same_as<std::vector<Node *>>;
    { Node::Build(info, children_infos) } -> std::same_as<Node *>;
    // clang-format on
    n->SetChild((int)1, n);
};

/**
 * @brief n-branches tree
 *
 * @tparam Node
 * @tparam N
 */
template <ntree_node_t Node, int N> class NTree
{
    using ThisType = NTree<Node, N>;

public:
    using NodePath = std::vector<int>;

    /**
     * @brief Create by pre-order
     *
     * @param info
     */
    void CreatePreOrder(typename Node::BuildInfo &info)
    {
        root = CreatePreOrder_Impl(info);
    }

    /**
     * @brief traverse each node in pre-order
     *
     * @tparam F
     * @param callback
     */
    template <typename F> void TraversePreOrder(F callback)
    {
        TraversePreOrder(root, callback);
    }

    template <typename F> static void TraversePreOrder(Node *root, F callback);

    /**
     * @brief traverse the tree from root. The callback decides which children
     * to continue
     *
     * @tparam F
     * @param callback
     */
    template <typename F> void TraverseFromRoot(F callback)
    {
        TraverseFromRoot<F>(root, callback);
    }

    template <typename F> static void TraverseFromRoot(Node *root, F callback);

    Node *FindNode(const NodePath &path);

protected:
    Node *root = nullptr;

    Node *CreatePreOrder_Impl(typename Node::BuildInfo &info);
};

} // namespace alg

/*======================== implementation ==========================*/

namespace alg
{

template <ntree_node_t Node, int N>
Node *NTree<Node, N>::CreatePreOrder_Impl(typename Node::BuildInfo &info)
{
    std::vector<typename Node::BuildInfo> children_infos;
    auto node = Node::Build(info, children_infos);
    if (children_infos.size() == static_cast<size_t>(N))
    {
        for (int i = 0; i < N; ++i)
        {
            node->SetChild(i, CreatePreOrder_Impl(children_infos[i]));
        }
    }
    return node;
}

template <ntree_node_t Node, int N>
template <typename F>
void NTree<Node, N>::TraversePreOrder(Node *root, F callback)
{
    if (root == nullptr)
    {
        return;
    }
    callback(root);
    std::vector<Node *> children = root->GetChildren();
    for (Node *c : children)
    {
        TraversePreOrder(c, callback);
    }
}

template <ntree_node_t Node, int N>
template <typename F>
void NTree<Node, N>::TraverseFromRoot(Node *root, F callback)
{
    if (root == nullptr)
    {
        return;
    }
    std::vector<Node *> children = callback(root);
    for (Node *c : children)
    {
        TraverseFromRoot(c, callback);
    }
}

template <ntree_node_t Node, int N>
Node *NTree<Node, N>::FindNode(const NodePath &path)
{
    Node *n = root;
    for (int i : path)
    {
        if (nullptr == n)
        {
            return nullptr;
        }
        std::vector<Node *> children = n->GetChildren();
        if (i < 0 || i >= children.size())
        {
            return nullptr;
        }
        n = children[i];
    }
    return n;
}

} // namespace alg
