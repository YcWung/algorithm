/**
 * @file ntree.hpp
 * @author Yongchao Wang (ycw.puzzle@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2022-10-07
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma once
#include <vector>

namespace alg
{

namespace ntree
{

/*=============================== concepts =============================*/

/**
 * @brief constraints of n-tree node type
 *
 * @tparam Node
 */
template <typename Node>
concept ntree_node_t = requires(Node *n, std::vector<Node *> nodes)
{
    nodes = n->GetChildren();
    n->SetChild((int)1, n);
};

/**
 * @brief constraints of pre-order buildable n-tree node type
 *
 * @tparam Node
 */
template <typename Node>
concept preorder_buildable_node_t =
    requires(Node *n, typename Node::PreorderBuildInfo info,
             std::vector<typename Node::PreorderBuildInfo> children_infos)
{
    requires ntree_node_t<Node>;
    typename Node::PreorderBuildInfo;
    // clang-format off
    { Node::Build(info, children_infos) } -> std::same_as<Node *>;
    // clang-format on
};

/**
 * @brief callback types for traversal of all nodes
 *
 * @tparam Node
 * @tparam F
 */
template <typename Node, typename F>
concept node_cb_t = std::is_invocable_v<F, Node *>;

/**
 * @brief callback types for traversal of subtrees.
 * The return node array controls which children to access
 *
 * @tparam Node
 * @tparam F
 */
template <typename Node, typename F>
concept subtree_cb_t = std::is_invocable_r_v<std::vector<Node *>, F, Node *>;

/*==================================== N-tree ===============================*/

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
    template <typename BuildInfo = Node::PreorderBuildInfo>
    void
    CreatePreOrder(BuildInfo &info) requires preorder_buildable_node_t<Node>
    {
        static_assert(std::is_same_v<BuildInfo, Node::PreorderBuildInfo>);
        root = CreatePreOrder_Impl(info);
    }

    /**
     * @brief traverse each node in pre-order
     *
     * @tparam F
     * @param callback
     */
    template <typename F>
    void TraversePreOrder(F callback) requires node_cb_t<Node, F>
    {
        TraversePreOrder(root, callback);
    }

    template <typename F>
    static void TraversePreOrder(Node *root,
                                 F callback) requires node_cb_t<Node, F>;

    /**
     * @brief traverse each node in post-order
     *
     * @tparam F
     * @param callback
     */
    template <typename F>
    void TraversePostOrder(F callback) requires node_cb_t<Node, F>
    {
        TraversePostOrder(root, callback);
    }

    template <typename F>
    static void TraversePostOrder(Node *root,
                                  F callback) requires node_cb_t<Node, F>;

    /**
     * @brief traverse the tree from root. The callback decides which children
     * to continue
     *
     * @tparam F
     * @param callback
     */
    template <typename F>
    void TraverseFromRoot(F callback) requires subtree_cb_t<Node, F>
    {
        TraverseFromRoot<F>(root, callback);
    }

    template <typename F>
    static void TraverseFromRoot(Node *root,
                                 F callback) requires subtree_cb_t<Node, F>;

    /**
     * @brief Get the number of all nodes
     *
     * @return int
     */
    virtual int Count() const
    {
        int count = 0;
        const_cast<ThisType *>(this)->TraversePreOrder([&](Node *) mutable
                                                       { ++count; });
        return count;
    }

    /**
     * @brief destruct all nodes
     *
     * @return int
     */
    int DeleteAllNodes()
    {
        int count = 0;
        const_cast<ThisType *>(this)->TraversePostOrder(
            [&](Node *node) mutable
            {
                delete node;
                ++count;
            });
        root = nullptr;
        return count;
    }

    /**
     * @brief Find a node at a path
     *
     * @param path
     * @return Node*
     */
    Node *FindNode(const NodePath &path);

protected:
    Node *root = nullptr;

    template <typename BuildInfo = Node::PreorderBuildInfo>
    Node *CreatePreOrder_Impl(BuildInfo &info) requires
        preorder_buildable_node_t<Node>;
};

} // namespace ntree

} // namespace alg

/*======================== implementation ==========================*/

namespace alg
{

namespace ntree
{

template <ntree_node_t Node, int N>
template <typename BuildInfo>
Node *NTree<Node, N>::CreatePreOrder_Impl(BuildInfo &info) requires
    preorder_buildable_node_t<Node>
{
    static_assert(std::is_same_v<BuildInfo, Node::PreorderBuildInfo>);

    std::vector<typename Node::PreorderBuildInfo> children_infos;
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
void NTree<Node, N>::TraversePreOrder(Node *root,
                                      F callback) requires node_cb_t<Node, F>
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
void NTree<Node, N>::TraversePostOrder(Node *root,
                                       F callback) requires node_cb_t<Node, F>
{
    if (root == nullptr)
    {
        return;
    }
    std::vector<Node *> children = root->GetChildren();
    for (Node *c : children)
    {
        TraversePostOrder(c, callback);
    }
    callback(root);
}

template <ntree_node_t Node, int N>
template <typename F>
void NTree<Node, N>::TraverseFromRoot(Node *root,
                                      F callback) requires subtree_cb_t<Node, F>
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

} // namespace ntree

} // namespace alg
