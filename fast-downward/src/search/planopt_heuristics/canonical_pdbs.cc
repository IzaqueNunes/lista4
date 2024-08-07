#include "canonical_pdbs.h"

#include "../algorithms/max_cliques.h"
#include <limits> 
#include <vector>

using namespace std;

namespace planopt_heuristics {

bool affects_pattern(const TNFOperator &op, const Pattern &pattern) {
    for (TNFOperatorEntry entry : op.entries) {
        for (int var_id : pattern) {
            if (entry.variable_id == var_id && entry.precondition_value != entry.effect_value) {
                return true;
            }
        }
    }
    return false;
}

vector<vector<int>> build_compatibility_graph(const vector<Pattern> &patterns, const TNFTask &task) {
    vector<vector<int>> graph(patterns.size());

    for (size_t i = 0; i < patterns.size(); ++i) {
        for (size_t j = i + 1; j < patterns.size(); ++j) {
            bool are_additive = true;
            
            for (const TNFOperator &op : task.operators) {
                if (affects_pattern(op, patterns[i]) && affects_pattern(op, patterns[j])) {
                    are_additive = false;
                    break;
                }
            }
            
            if (are_additive) {
                graph[i].push_back(j);
                graph[j].push_back(i);
            }
        }
    }

    return graph;
}

CanonicalPatternDatabases::CanonicalPatternDatabases(
    const TNFTask &task, const vector<Pattern> &patterns) {
    for (const Pattern &pattern : patterns) {
        pdbs.emplace_back(task, pattern);
    }

    vector<vector<int>> compatibility_graph = build_compatibility_graph(patterns, task);
    max_cliques::compute_max_cliques(compatibility_graph, maximal_additive_sets);
}

int CanonicalPatternDatabases::compute_heuristic(const TNFState &original_state) {
    vector<int> heuristic_values;
    heuristic_values.reserve(pdbs.size());
    for (const PatternDatabase &pdb : pdbs) {
        heuristic_values.push_back(pdb.lookup_distance(original_state));
        if (heuristic_values.back() == numeric_limits<int>::max()) {
            return numeric_limits<int>::max();
        }
    }

    int h = 0;
    for (const vector<int> &clique : maximal_additive_sets) {
        int clique_sum = 0;
        for (int pdb_index : clique) {
            clique_sum += heuristic_values[pdb_index];
        }
        h = max(h, clique_sum);
    }

    return h;
}
}
