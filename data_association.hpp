#ifndef DATA_ASSOCIATION_HPP
#define DATA_ASSOCIATION_HPP

#include "hungarian.h"
#include "line.hpp"
#include <vector>
#include <utility>
#include <set>

namespace Project{

std::pair<std::vector<std::pair<int, int> >, std::vector<Line> > associate_data(std::vector<Line> lines, std::vector<Line> landmarks){
    // TODO: output type will change to void
    // TODO: tune these constants
    const double MATCH_THRESHOLD = 100;
    const double MAX_DISTANCE = 1000000;

    int num_nodes = lines.size() > landmarks.size() ? lines.size() : landmarks.size();

    // add edges from lines to landmarks iff they're close enough
    std::vector<WeightedBipartiteEdge> graph = std::vector<WeightedBipartiteEdge>();
    for (int i = 0; i < lines.size(); i++) {
        Line new_line = lines[i];
        for (int j = 0; j < landmarks.size(); j++) {
            Line landmark = landmarks[i];
            double d = new_line.distance(landmark);
            if (d < MATCH_THRESHOLD) {
                graph.push_back(WeightedBipartiteEdge(i, j, int(100 * d)));
            }
        }
    }

    // add dummy nodes on right or left to balance number of nodes
    for (int i = lines.size(); i < num_nodes; i++) {
        for (int j = 0; j < num_nodes; j++) {
            graph.push_back(WeightedBipartiteEdge(i, j, int(100 * MAX_DISTANCE)));
        }
    }
    for (int j = landmarks.size(); j < num_nodes; j++) {
        for (int i = 0; i < num_nodes; i++) {
            graph.push_back(WeightedBipartiteEdge(i, j, int(100 * MAX_DISTANCE)));
        }
    }

    std::vector<int> matching = hungarianMinimumWeightPerfectMatching(num_nodes, graph);
    if (matching.size() == 0) {
        throw std::runtime_error("invalid graph");
    }
    std::vector<Line> new_lines = std::vector<Line>();
    std::vector<std::pair<int, int> > matches = std::vector<std::pair<int, int> >();
    for (int i = 0; i < num_nodes; i++) {
        if (matching[i] < landmarks.size()) {
            matches.push_back(std::pair<int, int>(i, matching[i]));
        } else {
            new_lines.push_back(lines[i]);
        }
    }

    return std::pair<std::vector<std::pair<int, int> >, std::vector<Line> >(matches, new_lines);
}
}

#endif
