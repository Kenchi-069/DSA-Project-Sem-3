#!/usr/bin/env python3
import json
import random
import math

def generate_test_graph(num_nodes=20, num_edges=40, filename="test_graph.json"):
    """Generate a random test graph"""
    
    # Generate nodes in a grid around Mumbai coordinates
    base_lat, base_lon = 19.0760, 72.8777
    nodes = []
    
    for i in range(num_nodes):
        node = {
            "id": i,
            "lat": base_lat + random.uniform(-0.05, 0.05),
            "lon": base_lon + random.uniform(-0.05, 0.05),
            "pois": random.sample(["Restaurant", "Hospital", "School", "Park"], k=random.randint(0, 2))
        }
        nodes.append(node)
    
    # Generate edges
    edges = []
    edge_id = 1000
    
    for _ in range(num_edges):
        u = random.randint(0, num_nodes - 1)
        v = random.randint(0, num_nodes - 1)
        
        if u == v:
            continue
        
        # Calculate approximate distance
        lat_diff = (nodes[u]["lat"] - nodes[v]["lat"]) * 111000
        lon_diff = (nodes[u]["lon"] - nodes[v]["lon"]) * 111000
        length = math.sqrt(lat_diff**2 + lon_diff**2)
        
        edge = {
            "id": edge_id,
            "u": u,
            "v": v,
            "length": round(length, 2),
            "average_time": round(length / random.uniform(10, 20), 2),
            "speed_profile": [random.uniform(8, 25) for _ in range(96)],
            "oneway": random.choice([True, False]),
            "road_type": random.choice(["primary", "secondary", "expressway"])
        }
        edges.append(edge)
        edge_id += 1
    
    graph = {
        "meta": {
            "id": "test_case_1",
            "nodes": num_nodes,
            "description": "Auto-generated test graph"
        },
        "nodes": nodes,
        "edges": edges
    }
    
    with open(filename, 'w') as f:
        json.dump(graph, f, indent=2)
    
    print(f"Generated {filename}")

def generate_test_queries(num_nodes=20, filename="test_queries.json"):
    """Generate test queries"""
    
    events = []
    query_id = 1
    
    # Shortest path queries
    for _ in range(3):
        events.append({
            "type": "shortest_path",
            "id": query_id,
            "source": random.randint(0, num_nodes - 1),
            "target": random.randint(0, num_nodes - 1),
            "mode": random.choice(["time", "distance"]),
            "constraints": {
                "forbidden_nodes": random.sample(range(num_nodes), k=random.randint(0, 2)),
                "forbidden_road_types": random.sample(["primary", "secondary"], k=random.randint(0, 1))
            }
        })
        query_id += 1
    
    # KNN queries
    for _ in range(2):
        events.append({
            "type": "knn",
            "id": query_id,
            "type": random.choice(["Restaurant", "Hospital"]),
            "query_point": {
                "lat": 19.0760 + random.uniform(-0.05, 0.05),
                "lon": 72.8777 + random.uniform(-0.05, 0.05)
            },
            "k": random.randint(3, 5),
            "metric": random.choice(["euclidean", "shortest_path"])
        })
        query_id += 1
    
    queries = {
        "meta": {"id": "test_queries"},
        "events": events
    }
    
    with open(filename, 'w') as f:
        json.dump(queries, f, indent=2)
    
    print(f"Generated {filename}")

if __name__ == "__main__":
    generate_test_graph()
    generate_test_queries()
    print("Test data generated successfully!")
