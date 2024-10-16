#pragma once
#include <unordered_set>
// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include <queue>
// Include the correct homework headers
#include "hw/HW6.h"

class MyAStarAlgo : public amp::AStar {
    public:
        virtual GraphSearchResult search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) override;
};