 /*
 * Copyright (C) David Redondo
 *
 * Released under the GPL version 3.
 *
 */

//simple Graph class that only does stuff that is needed by me
#ifndef __DAVID_GRAPH_H__
#define __DAVID_GRAPH_H__

#include <algorithm>
#include <iostream>
#include <functional>
#include <set>
#include <stack>
#include <tuple>
#include <vector>
#include <map>
#include <utility>

namespace david_graph {

template <typename V, typename W>
struct edge{
    V u;
    V v;
    W w;
};



template <typename V, typename W = std::tuple<>, typename Comp=std::less<V>>
class graph;

template <typename W>
using grid_graph  = graph<std::pair<int, int>, W>;

template <typename W = std::tuple<>>
grid_graph<W> make_grid_graph(int rows, int cols, bool wrap = false, W def = W{}){
    graph<std::pair<int, int>, W> g;
    for(int row = 0; row < rows - 1; ++row) {
        for(int col = 0; col < cols - 1; ++col) {
            g.add_edge({row, col}, {row, col+1}, def);
            g.add_edge({row, col}, {row+1, col}, def);
        }
    }
    for(int row = 0; row < rows - 1; ++row) {
        g.add_edge({row, cols-1}, {row+1, cols-1}, def);
        if(wrap) g.add_edge({row, cols-1}, {row, 0}, def);
    }
    for(int col = 0; col < cols - 1; ++col){
        g.add_edge({rows-1, col}, {rows-1, col+1}, def);
        if(wrap) g.add_edge({rows-1, col}, {0, col}, def);
    }
    if(wrap) {
        g.add_edge({rows-1, cols-1}, {0, cols-1}, def);
        g.add_edge({rows-1, cols-1}, {rows-1, 0}, def);
    }
    return g;
}


//specialisation for std::tuple<> at the end of the file as it needs the complete class
template <typename W>
void print_grid_graph(const grid_graph<W>& g)
{
    std::cout << "strict graph {\n";
    for(auto v: g.adj_list) {
        std::cout << "\"(" << v.first.first << ',' << v.first.second << ")\"\n";
        for(auto e: v.second){
            std::cout << "\"(" << v.first.first << ',' << v.first.second << ")\" -- \"(" << e.first.first << ','
                << e.first.second << ")\"[label=" << e.second << "]\n";
        }
    }
    std::cout << "}\n";
}

//undirected graph class
template <typename V, typename W, typename Comp>
class graph
{
friend void print_grid_graph<>(const graph<V, W, Comp>&);
//friend graph<V, W> make_grid_graph<W>(int, int, bool);
private:
    std::map<V, std::vector<std::pair<V, W>>, Comp> adj_list;
    std::set<V, Comp> _vertices;
public:
    using Edge = edge<V, W>;
    using vertex_type = V;
    using comparator = Comp;
    using weight_type = W;
    graph(){};
    void add_edge(const Edge& e)
    {
        add_edge(e.u, e.v, e.w);
    }
    void add_edge(const V& u, const V& v, const W& w)
    {
        _vertices.emplace(u);
        _vertices.emplace(v);
        adj_list[u].emplace_back(v, w);
        adj_list[v].emplace_back(u, w);

    }

    template <typename Pred>
    void remove_vertices(Pred&& pred)
    {
        for(auto it = _vertices.cbegin(); it != _vertices.cend(); ) {
            if(pred(*it)) {
                remove_edges(*it);
                it = _vertices.erase(it);
            }
            else {
                ++it;
            }
        }
    }

    void remove_vertex(const V& v)
    {
        remove_edges(v);
        _vertices.erase(v);
    }

    void remove_edge(const V& u, const V& v)
    {
        auto edges = find_edge(u, v);
        adj_list[u].erase(edges.first);
        adj_list[v].erase(edges.second);
    }


    template <typename Pred>
    void remove_edges(Pred&& pred)
    {
        for(const V& vertex : _vertices) {
            auto& edges = adj_list[vertex];
            edges.erase(std::remove_if(edges.begin(), edges.end(),
                [&](std::pair<V, W> e){return pred(vertex, e.first, e.second);}), edges.end());
        }
    }

    void set_weight(const Edge& e)
    {
        set_weight(e.u, e.v, e.w);
    }
    void set_weight(const V& u, const V& v, const W& w)
    {
        auto edges = find_edge(u, v);
        edges.second->second = edges.first->second = w;

    }

     std::vector<graph<V, W, Comp>> connected_components() const
	{
        std::vector<graph<V, W, Comp>> components;
        std::map<std::reference_wrapper<const V>, bool, Comp> visited;
        for(const V& vertex : _vertices) {
            if(!visited[vertex]) {
                components.emplace_back();
                auto& current_component = components.back();
                dfs_cc(vertex, current_component, visited);
            }
        }
        return components;
    }

    const std::set<V>&  vertices() const
    {
        return _vertices;
    }

    const std::vector<std::pair<V, W>>& edges(const V& v) const
    {
        return adj_list.at(v);
    }
private:
    using edge_it = typename decltype(adj_list)::mapped_type::iterator;
    using edge_c_it = typename decltype(adj_list)::mapped_type::const_iterator;
    std::pair<edge_it, edge_it> find_edge(const V& u, const V& v)
    {
        auto& u_edges = adj_list[u];
        auto& v_edges = adj_list[v];
        auto u_edge = std::find_if(u_edges.begin(), u_edges.end(), [&](const std::pair<V, W>& edge){return edge.first == v;});
        auto v_edge = std::find_if(v_edges.begin(), v_edges.end(), [&](const std::pair<V, W>& edge){return edge.first == u;});
        return {u_edge, v_edge};
    }
    void dfs_cc(const V& vertex, graph<V, W, Comp>& component, std::map<std::reference_wrapper<const V>, bool, Comp>& visited) const
    {
        std::stack<std::reference_wrapper<const V>> S;
        S.push(vertex);
        visited[vertex] = true;
        component._vertices.emplace(vertex);
        component.adj_list[vertex] = adj_list.at(vertex);
        while(!S.empty()) {
            const V& v = S.top();S.pop();
            for(const auto& edge : adj_list.at(v)){
                const V& to_vertex = edge.first;
                if(!visited[to_vertex]) {
                    S.push(to_vertex);
                    visited[to_vertex] = true;
                    component._vertices.emplace(to_vertex);
                    component.adj_list[to_vertex] = adj_list.at(to_vertex);
                }
            }
        }
    }

    void remove_edges(const V& v)
    {
        auto edges = adj_list[v];
        for(const auto& e : edges) {
            const V& dest = e.first;
            adj_list[dest].erase(std::find_if(adj_list[dest].begin(), adj_list[dest].end(),
                                [&](const std::pair<V, W>& edge){return edge.first == v;}));
        }
        adj_list.erase(v);
    };

};

template <>
inline void print_grid_graph(const grid_graph<std::tuple<>>& g)
{
    std::cout << "strict graph {\n";
    for(auto v: g.adj_list ) {
        std::cout << "\"(" << v.first.first << ',' << v.first.second << ")\"\n";
        for(auto e: v.second){
            std::cout << "\"(" << v.first.first << ',' << v.first.second << ")\" -- \"(" << e.first.first << ',' << e.first.second << ")\"\n";
        }
    }
    std::cout << "}\n";
}

}
#endif
