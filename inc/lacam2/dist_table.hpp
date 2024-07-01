/*
 * distance table with lazy evaluation, using BFS
 */
#pragma once

#include "graph.hpp"
#include "lacam_instance.hpp"
#include "utils.hpp"

struct DistTable {
  const uint V_size;  // number of vertices
  std::vector<std::vector<uint> >
      table;          // distance table, index: LACAMAgent-id & vertex-id
  std::vector<std::queue<Vertex*> > OPEN;  // search queue

  inline uint get(uint i, uint v_id);      // LACAMAgent, vertex-id
  uint get(uint i, Vertex* v);             // LACAMAgent, vertex

  DistTable(const LACAMInstance& ins);
  DistTable(const LACAMInstance* ins);

  void setup(const LACAMInstance* ins);  // initialization
};
