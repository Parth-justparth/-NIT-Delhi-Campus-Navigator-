import heapq
import math

# REAL NIT DELHI COORDS (from your Google Maps link)
import heapq
import math

# =======================
# NODE POSITIONS (approx) — center taken from your Google Maps link:
# center = 28.8165378, 77.1329659
# These coordinates were estimated to match the pins you marked.
# =======================

NODE_POS = {
    "temple": (28.817619, 77.133069),
    "badminton court": (28.817349, 77.132555),
    "basketball court": (28.817078, 77.132144),
    "tennis court": (28.816988, 77.132349),
    "fountain": (28.816898, 77.132863),
    "parking 1": (28.817259, 77.133582),
    "parking 2": (28.816628, 77.133685),
    "director home": (28.815457, 77.132915),
    "admin entrance 1": (28.816177, 77.133069),
    "admin entrance 2": (28.816268, 77.133171),
    "academic block 1": (28.816268, 77.133377),
    "football ground": (28.815997, 77.132863),
    "volleyball court 1": (28.816177, 77.133069),
    "kabbadi court": (28.816268, 77.133120),
    "phase 1 hostel": (28.815817, 77.131733),
    "boys hostel": (28.817799, 77.132966),
    "girls hostel": (28.817259, 77.132658)
}

# =======================
# GRAPH (DISTANCES in meters) — simple connections for demo.
# You can tweak weights or add/remove edges.
# =======================
SAMPLE_GRAPH = {
    "temple": {"badminton court": 140, "boys hostel": 220},
    "badminton court": {"temple": 140, "basketball court": 30, "fountain": 80},
    "basketball court": {"badminton court": 30, "tennis court": 20},
    "tennis court": {"basketball court": 20, "fountain": 40},
    "fountain": {"tennis court": 40, "badminton court": 80, "admin entrance 1": 90},
    "parking 1": {"parking 2": 80, "main_gate": 120} ,  # main_gate not in node list — optional
    "parking 2": {"parking 1": 80, "admin entrance 2": 140},
    "director home": {"football ground": 100},
    "admin entrance 1": {"fountain": 90, "admin entrance 2": 40, "academic block 1": 70},
    "admin entrance 2": {"admin entrance 1": 40, "academic block 1": 50, "kabbadi court": 60},
    "academic block 1": {"admin entrance 1": 70, "admin entrance 2": 50, "volleyball court 1": 60},
    "football ground": {"director home": 100, "volleyball court 1": 80},
    "volleyball court 1": {"football ground": 80, "academic block 1": 60, "kabbadi court": 40},
    "kabbadi court": {"volleyball court 1": 40, "admin entrance 2": 60},
    "phase 1 hostel": {"teachers quater": 120},  # teachers quater not in nodes — fine as placeholder
    "boys hostel": {"temple": 220, "fountain": 200},
    "girls hostel": {"badminton court": 140, "fountain": 120}
}

# Ensure graph nodes include all keys (some placeholders were used above).
# Remove any edges pointing to missing nodes or add them if you want completeness.

# =======================
# DIJKSTRA
# =======================
def dijkstra(graph, start, goal):
    pq = [(0, start)]
    distances = {node: float('inf') for node in graph}
    distances[start] = 0
    parents = {start: None}
    visited = set()

    while pq:
        current_dist, node = heapq.heappop(pq)
        if node in visited:
            continue
        visited.add(node)

        if node == goal:
            return reconstruct_path(parents, start, goal), current_dist

        for neighbor, weight in graph.get(node, {}).items():
            new_dist = current_dist + weight
            if new_dist < distances.get(neighbor, float('inf')):
                distances[neighbor] = new_dist
                parents[neighbor] = node
                heapq.heappush(pq, (new_dist, neighbor))

    return None, float('inf')


# =======================
# A* (geo heuristic using node positions)
# =======================
def heuristic(a, b):
    (lat1, lon1) = a
    (lat2, lon2) = b
    # rough meters approximation
    return math.hypot((lat1 - lat2) * 111000, (lon1 - lon2) * 111000 * math.cos(math.radians((lat1+lat2)/2)))


def a_star(graph, start, goal, positions):
    open_set = [(0, start)]
    g = {node: float('inf') for node in graph}
    f = {node: float('inf') for node in graph}

    g[start] = 0
    f[start] = heuristic(positions[start], positions[goal])
    parents = {start: None}
    visited = set()

    while open_set:
        _, node = heapq.heappop(open_set)
        if node in visited:
            continue
        visited.add(node)

        if node == goal:
            return reconstruct_path(parents, start, goal), g[goal]

        for neighbor, weight in graph.get(node, {}).items():
            temp_g = g[node] + weight
            if temp_g < g.get(neighbor, float('inf')):
                g[neighbor] = temp_g
                parents[neighbor] = node
                f[neighbor] = temp_g + heuristic(positions[neighbor], positions[goal])
                heapq.heappush(open_set, (f[neighbor], neighbor))

    return None, float('inf')


# =======================
def reconstruct_path(parents, start, goal):
    path = []
    cur = goal
    while cur is not None:
        path.append(cur)
        cur = parents.get(cur)
    path.reverse()
    return path

def reconstruct(parents, start, goal):
    path = []
    node = goal
    while node is not None:
        path.append(node)
        node = parents.get(node)
    return path[::-1]

def dijkstra(graph, start, goal):
    pq = [(0, start)]
    dist = {n: float('inf') for n in graph}
    dist[start] = 0
    parents = {start: None}
    visited = set()

    while pq:
        cost, node = heapq.heappop(pq)
        if node in visited:
            continue
        visited.add(node)

        if node == goal:
            return reconstruct(parents, start, goal), cost

        for nei, w in graph[node].items():
            new_cost = cost + w
            if new_cost < dist[nei]:
                dist[nei] = new_cost
                parents[nei] = node
                heapq.heappush(pq, (new_cost, nei))

    return None, float('inf')

def heuristic(a, b):
    return math.hypot((a[0]-b[0])*111000, (a[1]-b[1])*111000)

def a_star(graph, start, goal, positions):
    open_set = [(0, start)]
    g = {n: float('inf') for n in graph}
    g[start] = 0
    f = {n: float('inf') for n in graph}
    f[start] = heuristic(positions[start], positions[goal])
    parents = {start: None}

    while open_set:
        _, node = heapq.heappop(open_set)

        if node == goal:
            return reconstruct(parents, start, goal), g[goal]

        for nei, w in graph[node].items():
            temp_g = g[node] + w

            if temp_g < g[nei]:
                g[nei] = temp_g
                parents[nei] = node
                f[nei] = temp_g + heuristic(positions[nei], positions[goal])
                heapq.heappush(open_set, (f[nei], nei))

    return None, float('inf')
