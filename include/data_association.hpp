#ifndef DATA_ASSOCIATION_HPP
#define DATA_ASSOCIATION_HPP

#include <iostream>
#include <set>
#include <utility>
#include <vector>
#include "Line.hpp"
#include "hungarian.h"

namespace Project {

std::pair<std::vector<std::pair<Line, int> >, std::vector<Line> > associate_data(
    std::vector<Line> lines, std::vector<Line> landmarks) {
  // TODO: output type will change to void
  // TODO: tune these constants
  const double MATCH_THRESHOLD = .1;
  const double MAX_DISTANCE = 10000;
  int num_nodes =
      lines.size() > landmarks.size() ? lines.size() : landmarks.size();

  // add edges from lines to landmarks
  // add all edges and threshold in the solution step for new landmarks
  std::vector<WeightedBipartiteEdge> graph =
      std::vector<WeightedBipartiteEdge>();
  for (int i = 0; i < lines.size(); i++) {
    Line new_line = lines[i];
    for (int j = 0; j < landmarks.size(); j++) {
      Line landmark = landmarks[j];
      double d = new_line.distance(landmark);
      graph.push_back(WeightedBipartiteEdge(i, j, int(10000 * d)));
    }
  }

  // add dummy nodes on right or left to balance number of nodes
  for (int i = lines.size(); i < num_nodes; i++) {
    for (int j = 0; j < num_nodes; j++) {
      graph.push_back(WeightedBipartiteEdge(i, j, int(10000 * MAX_DISTANCE)));
    }
  }
  for (int j = landmarks.size(); j < num_nodes; j++) {
    for (int i = 0; i < num_nodes; i++) {
      graph.push_back(WeightedBipartiteEdge(i, j, int(10000 * MAX_DISTANCE)));
    }
  }

  std::vector<int> matching =
      hungarianMinimumWeightPerfectMatching(num_nodes, graph);
  if (matching.size() == 0) {
    std::vector<std::pair<Line, int> > ps;
    return std::pair<std::vector<std::pair<Line, int> >, std::vector<Line> >(
        ps, lines);
  }
  std::vector<Line> new_lines = std::vector<Line>();
  std::vector<std::pair<Line, int> > matches =
      std::vector<std::pair<Line, int> >();
  for (int i = 0; i < lines.size(); i++) {
    if (matching[i] < landmarks.size()) {
      double distance = lines[i].distance(landmarks[matching[i]]);
      if (distance > MATCH_THRESHOLD) {
        new_lines.push_back(lines[i]);
        continue;
      }
      matches.push_back(std::pair<Line, int>(lines[i], matching[i]));
    } else {
      new_lines.push_back(lines[i]);
    }
  }

  return std::pair<std::vector<std::pair<Line, int> >, std::vector<Line> >(
      matches, new_lines);
}
}  // namespace Project

#endif
