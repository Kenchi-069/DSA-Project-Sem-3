#!/usr/bin/env python3
import json
import random
import math

VALID_POIS = ["restaurant", "petrol station", "hospital", "pharmacy", "hotel", "atm"]
VALID_ROAD_TYPES = ["primary", "secondary", "tertiary", "local", "expressway"]

def generate_graph(num_nodes=10, num_edges=20, with_speed_profile=True):
    graph = {
        "meta": {
            "id": f"test_n{num_nodes}_e{num_edges}",
            "nodes": num_nodes,
            "description": f"Test graph with {num_nodes} nodes"
        },
        "nodes": [],
        "edges": []
    }
    
    base_lat = 19.0
    base_lon = 72.8
    
    # Create grid-like node layout
    for i in range(num_nodes):
        row = i // int(math.sqrt(num_nodes) + 1)
        col = i % int(math.sqrt(num_nodes) + 1)
        node = {
            "id": i,
            "lat": base_lat + row * 0.01,
            "lon": base_lon + col * 0.01,
            "pois": random.sample(VALID_POIS, random.randint(0, 2))
        }
        graph["nodes"].append(node)
    
    # Create edges ensuring connectivity
    edge_id = 1000
    edges_created = set()
    
    # First create a spanning tree for connectivity
    for i in range(1, num_nodes):
        u = random.randint(0, i - 1)
        v = i
        if (u, v) not in edges_created and (v, u) not in edges_created:
            edges_created.add((u, v))
    
    # Add remaining edges randomly
    while len(edges_created) < num_edges:
        u = random.randint(0, num_nodes - 1)
        v = random.randint(0, num_nodes - 1)
        if u != v and (u, v) not in edges_created and (v, u) not in edges_created:
            edges_created.add((u, v))
    
    # Build edge objects
    for u, v in edges_created:
        lat1, lon1 = graph["nodes"][u]["lat"], graph["nodes"][u]["lon"]
        lat2, lon2 = graph["nodes"][v]["lat"], graph["nodes"][v]["lon"]
        distance = math.sqrt((lat2 - lat1)**2 + (lon2 - lon1)**2) * 111000
        
        avg_speed = random.uniform(20, 60)
        edge = {
            "id": edge_id,
            "u": u,
            "v": v,
            "length": round(distance, 2),
            "average_time": round(distance / avg_speed, 2),
            "oneway": random.choice([True, False]),
            "road_type": random.choice(VALID_ROAD_TYPES)
        }
        
        if with_speed_profile:
            speed_profile = [max(10, avg_speed + random.gauss(0, 5)) for _ in range(96)]
            edge["speed_profile"] = [round(s, 2) for s in speed_profile]
        
        graph["edges"].append(edge)
        edge_id += 1
    
    return graph

def generate_phase1_queries(graph, num_queries=15):
    queries = {"meta": {"id": "phase1_test_queries"}, "events": []}
    node_ids = [n["id"] for n in graph["nodes"]]
    edge_ids = [e["id"] for e in graph["edges"]]
    query_id = 1
    
    # Shortest path queries (40%)
    for _ in range(int(num_queries * 0.4)):
        source, target = random.sample(node_ids, 2)
        mode = random.choice(["distance", "time"])
        query = {
            "type": "shortest_path",
            "id": query_id,
            "source": source,
            "target": target,
            "mode": mode
        }
        if random.random() < 0.3:
            query["constraints"] = {
                "forbidden_nodes": random.sample(node_ids, min(2, len(node_ids) // 2)),
                "forbidden_road_types": random.sample(VALID_ROAD_TYPES, random.randint(0, 2))
            }
        queries["events"].append(query)
        query_id += 1
    
    # KNN queries (30%)
    for _ in range(int(num_queries * 0.3)):
        poi = random.choice(VALID_POIS)
        # Ensure at least one node has this POI
        if not any(poi in n.get("pois", []) for n in graph["nodes"]):
            random.choice(graph["nodes"])["pois"].append(poi)
        
        query = {
            "type": "knn",
            "id": query_id,
            "poi": poi,
            "query_point": {
                "lat": 19.0 + random.random() * 0.05,
                "lon": 72.8 + random.random() * 0.05
            },
            "k": random.randint(2, 5),
            "metric": random.choice(["euclidean", "shortest_path"])
        }
        queries["events"].append(query)
        query_id += 1
    
    # Dynamic updates (30%)
    for _ in range(int(num_queries * 0.3)):
        if random.random() < 0.5 and edge_ids:
            queries["events"].append({
                "id": query_id,
                "type": "remove_edge",
                "edge_id": random.choice(edge_ids)
            })
        else:
            patch = {}
            if random.random() < 0.7:
                patch["length"] = round(random.uniform(100, 1000), 2)
            if random.random() < 0.5:
                patch["average_time"] = round(random.uniform(5, 50), 2)
            
            queries["events"].append({
                "id": query_id,
                "type": "modify_edge",
                "edge_id": random.choice(edge_ids) if edge_ids else 1000,
                "patch": patch
            })
        query_id += 1
    
    return queries

def generate_phase2_queries(graph, num_queries=12):
    queries = {"meta": {"id": "phase2_test_queries"}, "events": []}
    node_ids = [n["id"] for n in graph["nodes"]]
    query_id = 1
    
    # K shortest paths exact (40%)
    for _ in range(int(num_queries * 0.4)):
        source, target = random.sample(node_ids, 2)
        queries["events"].append({
            "type": "k_shortest_paths",
            "id": query_id,
            "source": source,
            "target": target,
            "k": random.randint(2, 10),
            "mode": "distance"
        })
        query_id += 1
    
    # K shortest paths heuristic (30%)
    for _ in range(int(num_queries * 0.3)):
        source, target = random.sample(node_ids, 2)
        queries["events"].append({
            "type": "k_shortest_paths_heuristic",
            "id": query_id,
            "source": source,
            "target": target,
            "k": random.randint(2, 7),
            "overlap_threshold": random.randint(40, 80)
        })
        query_id += 1
    
    # Approximate shortest paths (30%)
    for _ in range(int(num_queries * 0.3)):
        batch_queries = []
        for _ in range(random.randint(10, 20)):
            source, target = random.sample(node_ids, 2)
            batch_queries.append({"source": source, "target": target})
        
        queries["events"].append({
            "type": "approx_shortest_path",
            "id": query_id,
            "queries": batch_queries,
            "time_budget_ms": random.randint(100, 500),
            "acceptable_error_pct": random.choice([5.0, 10.0, 15.0])
        })
        query_id += 1
    
    return queries

def main():
    print("Generating test cases...")
    
    # Small graph for Phase 1 and 2
    print("- Small graph (20 nodes, 40 edges)")
    graph_small = generate_graph(num_nodes=20, num_edges=40, with_speed_profile=True)
    with open("test_graph.json", "w") as f:
        json.dump(graph_small, f, indent=2)
    
    print("- Phase 1 queries")
    p1_queries = generate_phase1_queries(graph_small, num_queries=20)
    with open("test_queries_phase1.json", "w") as f:
        json.dump(p1_queries, f, indent=2)
    
    print("- Phase 2 queries")
    p2_queries = generate_phase2_queries(graph_small, num_queries=15)
    with open("test_queries_phase2.json", "w") as f:
        json.dump(p2_queries, f, indent=2)
    
    # Medium graph
    print("- Medium graph (100 nodes, 250 edges)")
    graph_medium = generate_graph(num_nodes=100, num_edges=250, with_speed_profile=False)
    with open("test_graph_medium.json", "w") as f:
        json.dump(graph_medium, f, indent=2)
    
    # Large graph for stress testing (without speed profiles to reduce size)
    print("- Large graph (1000 nodes, 3000 edges)")
    graph_large = generate_graph(num_nodes=1000, num_edges=3000, with_speed_profile=False)
    with open("test_graph_large.json", "w") as f:
        json.dump(graph_large, f, indent=2)
    
    print("\nTest cases generated successfully!")
    print("Files created:")
    print("  - test_graph.json (small, with speed profiles)")
    print("  - test_graph_medium.json")
    print("  - test_graph_large.json")
    print("  - test_queries_phase1.json")
    print("  - test_queries_phase2.json")

if __name__ == "__main__":
    main()
