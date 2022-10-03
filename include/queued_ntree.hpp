#include <deque>
#include <functional>
#include <type_traits>

namespace algo
{

template <typename Node, int N> class QueuedNTree
{
public:
    int CreatePreOrder(typename Node::BuildInfo &info);

protected:
    std::deque<Node> q;
    auto AllocNode = [&]() -> Node *
    {
        q.emplace_back();
        auto &n = q.back();
        return &n;
    };
};

} // namespace algo

/*======================== implementation ==========================*/

namespace algo
{

template <typename Node, int N>
int QueuedNTree<Node, N>::CreatePreOrder(typename Node::BuildInfo &info)
{
    auto node = Node::Create(info, AllocNode);
    if (node == nullptr)
    {
        return -1;
    }
    auto infos = node.template GetChildrenBuildInfos();
}

} // namespace algo