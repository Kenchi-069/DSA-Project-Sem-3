import json
from math import fabs

# -------------------------------
# Utilities
# -------------------------------

def edge_set(path):
    """Convert a node path into an undirected edge set."""
    edges = []
    for i in range(len(path) - 1):
        u, v = path[i], path[i+1]
        edges.append((min(u, v), max(u, v)))
    return edges

def overlap_percent(pathA, pathB):
    """Compute percentage of edges in A that also appear in B."""
    edgesA = set(edge_set(pathA))
    edgesB = set(edge_set(pathB))
    if not edgesA:
        return 0.0
    overlap = len(edgesA & edgesB)
    return 100.0 * overlap / len(edgesA)

def compute_penalty(paths, lengths, overlap_threshold):
    """
    Compute penalties for a single query.
    
    paths:   list of node-paths
    lengths: list of real distances
    """
    k = len(paths)
    L1 = lengths[0]  # real shortest path length

    # Precompute pairwise overlaps
    overlap_matrix = [[0]*k for _ in range(k)]
    for i in range(k):
        for j in range(k):
            overlap_matrix[i][j] = overlap_percent(paths[i], paths[j])

    totalPenalty = 0.0
    perPath = []

    for i in range(k):
        # Overlap penalty: count paths whose overlap% > threshold
        overlapPenalty = sum(
            1 for j in range(k) if overlap_matrix[i][j] > overlap_threshold
        )

        # Distance penalty: |Li - L1| / L1 + 0.1
        Li = lengths[i]
        distancePenalty = abs(Li - L1) / L1 + 0.1

        penalty_i = overlapPenalty * distancePenalty
        totalPenalty += penalty_i

        perPath.append({
            "path_index": i,
            "overlapPenalty": overlapPenalty,
            "distancePenalty": distancePenalty,
            "penalty": penalty_i
        })

    return totalPenalty, perPath

# -------------------------------
# MAIN FUNCTION
# -------------------------------

def compute_all_penalties(events_file, results_file):
    with open(events_file, "r") as f:
        events_data = json.load(f)

    with open(results_file, "r") as f:
        results_data = json.load(f)

    # Map query id → overlap_threshold
    threshold_map = {}
    for ev in events_data["events"]:
        if ev["type"] == "k_shortest_paths_heuristic":
            threshold_map[ev["id"]] = ev["overlap_threshold"]

    output = {}

    for res in results_data["results"]:
        qid = res["id"]

        # Skip queries that do not appear in events
        if qid not in threshold_map:
            continue
        
        overlap_threshold = threshold_map[qid]

        # Get paths
        path_entries = res.get("paths", [])

        # ❗ Skip entries where no paths were returned
        if not path_entries:
            output[qid] = {
                "error": "No paths returned by algorithm",
                "overlap_threshold": overlap_threshold,
                "totalPenalty": None,
                "perPath": []
            }
            continue

        paths = [p["path"] for p in path_entries]
        lengths = [p["length"] for p in path_entries]

        # ❗ Safety: lengths must also be non-empty
        if not lengths:
            output[qid] = {
                "error": "No lengths found",
                "overlap_threshold": overlap_threshold,
                "totalPenalty": None,
                "perPath": []
            }
            continue

        totalPenalty, perPath = compute_penalty(paths, lengths, overlap_threshold)

        output[qid] = {
            "overlap_threshold": overlap_threshold,
            "totalPenalty": totalPenalty,
            "perPath": perPath
        }

    return output

# -------------------------------
# Example usage
# -------------------------------
if __name__ == "__main__":
    events_file = "Testcases/Teja/heur_queries.json"    # replace with your path
    results_file = "outheur.json"  # replace with your path

    penalties = compute_all_penalties(events_file, results_file)

    import pprint
    pprint.pprint(penalties)
