#include <deque>
#include <functional>
#include <type_traits>
#include <memory>

namespace alg
{

template <typename Node, int N> class QueuedNTree
{
public:
    Node *CreatePreOrder(typename Node::BuildInfo &info);
    size_t GetNumberOfNodes() const { return q.size(); }

protected:
    std::deque<std::unique_ptr<Node>> q;
};

} // namespace alg

/*======================== implementation ==========================*/

namespace alg
{

template <typename Node, int N>
Node *QueuedNTree<Node, N>::CreatePreOrder(typename Node::BuildInfo &info)
{
    std::vector<typename Node::BuildInfo> children_infos;
    auto node = Node::Build(info, children_infos);
    if (node != nullptr)
    {
        q.emplace_back(node);
    }
    if (children_infos.size() == static_cast<size_t>(N))
    {
        for (int i = 0; i < N; ++i)
        {
            node->SetChild(i, CreatePreOrder(children_infos[i]));
        }
    }
    return node;
}

} // namespace alg