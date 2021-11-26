#include <vector>

template<typename ScalarT, int Dim, typename IndexT>
struct KdTreeNode{
  enum {
    Leaf = 0,
    Container
  } type;

  union {
    KdTreeNode** children;        // two children
    IndexT* indices;                         // num and start
  } content;

  int dim;
  
};

template<typename ScalarT, int Dim, typename IndexT>
class KdTree{
 public:
  using Node = KdTreeNode<ScalarT, Dim, IndexT>;
  
  KdTree(ScalarT *points_data, IndexT points_num, IndexT points_step)
      : data(points_data),
        num(points_num), step(points_step) {
    indices.resize(num);
    for (IndexT i = 0; i < num; ++i) {
      indices[i] = i;
    }
    root = new Node(Node::Container, nullptr);
    build(root, 0, 0, num);
  }

  void knn(ScalarT* pt, IndexT k, std::vector<IndexT>& k_indices, std::vector<ScalarT>& k_sqr_distances){
    return knn_impl(pt, k, root, k_indices, k_sqr_distances);
  }

 protected:
  ScalarT* data;
  IndexT num;
  IndexT step;
  KdTreeNode* root;
  std::vector<IndexT> indices

  ScalarT &at(IndexT point_id, int dim) {
    return data[point_id * step + dim];
  }

  void build(Node* node, int d, IndexT start, IndexT num){
    if (num <= 0) { return; }
    ScalarT min = std::numeric_limits<ScalarT>::max(), max = std::numeric_limits<ScalarT>::min();
    for (IndexT i = 0; i < num; ++i) {
      ScalarT val = at(i, d);
      if (val < min) {
        min = val;
      }
      if (val > max) {
        max = val;
      }
    }
    if (max - min < eps) {
      node->type = Node::Leaf;
      node->contents.indices = new IndexT[2]({start, num});
      return;
    }
    ScalarT mid = 0.5 * max + 0.5 * min;
    IndexT i1 = start, i2 = start+ num - 1;
    while(i1 < i2){
      while(at(i1, d) < mid) ++i1;
      while(at(i2, d) > mid) ++i2;
      if (i1 < i2) {
        std::swap(at(i1, d), at(i2, d));
      }
    }
    IndexT num1 = i1, num2 = num - num2;
    
    node->type = Node::Container;
    node->contents.children = new Node[2];
    IndexT next_d = d+1;
    if (next_d >= Dim) {next_d = 0;}
    build(node->contents.children[0], next_d, start, num1);
    build(node->contents.children[0], next_d, start + num1, num2);
  }

  void knn_impl(ScalarT *pt, IndexT k, Node *node,
                std::vector<IndexT> &k_indices,
                std::vector<ScalarT> &k_sqr_distances) {
    if (node->type == Node::Leaf) {
      auto& num = node->contents.indices[0];
      auto& s = node->contents.indices[1];
      std::vector<IndexT> iv(num), iv2(num);
      std::vector<ScalarT> dv(num), dv2(num);
      for (IndexT i = 0; i < num; ++i) {
        iv[i] = i;
        dv[i] = Dis2(&at(indices[s + i], 0), pt);
      }
      std::sort(iv.begin(), iv.end(), [&dv](IndexT x, IndexT y){ return dv[x] < dv[y];});
      for (IndexT i = 0; i < num; ++i) {
        dv2[i] = dv[iv[i]];
        iv2[i] = indices[s + i];
      }
      if (num <= k) {
        k_indices.resize(num);
        k_sqr_distances.resize(num);
        for (auto i = 0; i < num; ++i) {
          k_indices[i] = indices[s + i];
          k_sqr_distances[i] = Dis2(&at(k_indices[i], 0), pt);
        }
      } else {
        // todo
      }
      return;
    }

    if (node->contents.children == nullptr) {
      k_indices.resize(0);
      k_sqr_distances.resize(0);
      return;
    }

    std::vector<IndexT> indices[2];
    std::vector<ScalarT> distances[2];
    for (int i = 0; i < 2; ++i) {
      knn_impl(pt, k, node->contents.children[i], indices[i], distances[i]);
    }

    // todo
  }

  ScalarT Dis2(const ScalarT* p1, const ScalarT* p2) {
    // todo
  }
};

