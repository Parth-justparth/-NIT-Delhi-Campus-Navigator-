"""
pathfinding.py

Final pathfinding module for NIT Delhi campus:

- Uses only Dijkstra (no A*)
- Road nodes: road 1 .. road 19 (as per user's list)
- Road subgraph: each road connected to its k nearest road neighbors (k=2)
  to form a usable road-network backbone.
- Each POI (non-road node) is automatically connected to its single nearest road node (Option A).
- All edges are bidirectional with weights in meters (approx).
- Exports: dijkstra, GRAPH, NODE_POS, cleaned_graph
"""

import heapq
import math
from typing import Dict, Tuple, List, Any

# -----------------------
# NODE POSITIONS (POIs)
# -----------------------
NODE_POS: Dict[str, Tuple[float, float]] = {
    # POIs (final list confirmed by user)
    "temple": (28.81742681403296, 77.13315253451646),
    "badminton court": (28.816837, 77.132703),
    "basketball court": (28.815455837787475, 77.13173569750464),
    "tennis court": (28.81563164187394, 77.13191842362096),
    "fountain": (28.815513, 77.132521),
    "NIT fountain": (28.815475348009638, 77.13255764206696),
    "parking 1": (28.81654801641594, 77.13356997055654),
    "parking 2": (28.815811446230132, 77.13377289085467),
    "director home": (28.813633505502203, 77.13276432182609),
    "admin entrance 1": (28.81126698105518, 77.13301995864423),
    "admin entrance 2": (28.811600654939394, 77.13322476656818),
    "admin entrance 3": (28.81147307387458, 77.13264234403442),
    "admin entrance 4": (28.81175627558716, 77.13287915319651),
    "academic block 1": (28.81208798738834, 77.13374363131756),
    "football ground": (28.812562129288704, 77.13253558604944),
    "volleyball court (Boys)": (28.81304417717569, 77.13303046202812),
    "volleyball court (Girls)": (28.812476659399422, 77.13315069449247),
    "kabbadi court (Girls)": (28.81344857418351, 77.13289859416399),
    "kabaddi court (Boys)": (28.8120712058252, 77.13317260393805),
    "phase 1 Boys hostel": (28.81202859088489, 77.13135305897349),
    "boys hostel": (28.816753, 77.132855),
    "girls hostel": (28.816149618250876, 77.13248544482505),
    "Teachers quarter": (28.811431041246195, 77.13211026872494),
    "HK cafe": (28.811700, 77.132250),
    "open air theatre": (28.816443, 77.132876),
    "Mechanical department": (28.815626655356724, 77.13231164577681),
    "Mini campus entrance 1": (28.816167496827983, 77.13339725619927),
    "Mini campus entrance 2": (28.81704336719094, 77.13328127576588),
    "Mini campus entrance 3": (28.816521623238057, 77.1327298329018),
    "Mini campus entrance 4": (28.81598419065432, 77.13253664680329),
    "Main Entrance 1": (28.81609364969359, 77.1337764647734),
    "Main Entrance 2": (28.811032, 77.133154),
    "Amul canteen": (28.816071113039055, 77.13227234870737),
    "Nit delhi cafeteria": (28.815987177581793, 77.13218364691596),
    "Bus Stand":(28.816288947496187, 77.13431787679824),
}

# -----------------------
# ROAD NODES (user-provided)
# names intentionally include space: "road 1", "road 2", ...
# -----------------------
ROAD_NODES: Dict[str, Tuple[float, float]] = {
    "road 1": (28.812014, 77.131909),
    "road 2": (28.811771, 77.132476),
    "road 3": (28.811947, 77.133228),
    "road 4": (28.811111, 77.133498),
    "road 5": (28.812002, 77.133365),
    "road 6": (28.812169, 77.133326),
    "road 7": (28.812542, 77.133228),
    "road 8": (28.812865, 77.133128),
    "road 9": (28.813089, 77.133084),
    "road 10": (28.813536, 77.132953),
    "road 11": (28.813947, 77.132831),
    "road 12": (28.815615, 77.132350),
    "road 13": (28.815824, 77.132291),
    "road 14": (28.816164, 77.133623),
    "road 15": (28.816160, 77.132425),
    "road 16": (28.816560, 77.132698),
    "road 17": (28.816756, 77.132836),
    "road 18": (28.816861, 77.133410),
    "road 19": (28.816725, 77.133444),
    "road 20": (28.815961002208777, 77.13301529649424),
    "Road 21": (28.815862064092975, 77.13251743672802),
    "road 22": (28.815962307803677, 77.13220493534588),
    "road 23": (28.816021373516996, 77.13327290491463),
    "road 24": (28.812178405294215, 77.1319395378534),
    "road 25": (28.812711802829273, 77.13176560616927),
    "road 26": (28.813372195995843, 77.13206129001794),
    "road 27": (28.813458554792344, 77.13237436704937),
    "road 28": (28.81360079265402, 77.13293094843856),
    "road 29": (28.811011840277935, 77.13267865101366),
    "road 30": (28.811712884324976, 77.13241195576468),
    'road 31': (28.817248046543547, 77.13328586297297),
    'road 32': (28.816820585402905, 77.13285660453181),
    "road 33": (28.817130450492733, 77.1331348952264),
    "road 34": (28.81658183626223, 77.13346536542625),
    "road 35": (28.816301032455165, 77.13348652907462),
    "road 36": (28.816140032205528, 77.13333128818611),
    "road 37": (28.816177, 77.132418),
    "road 38": (28.81575043547552, 77.13236118246934),
    "road 39":(28.815658118969353, 77.13235330485384),
    "road 40":(28.816468891592496, 77.13267119398371),
    "road 41":(28.81183271715359, 77.13291365746468),
    "road 42":(28.815859681828815, 77.132493195875),
    "road 43":(28.81566603847016, 77.13236018051836),
    
}

# Add roads into NODE_POS so positions dict contains everything
NODE_POS.update(ROAD_NODES)

# -----------------------
# Utility: approximate meters between two lat/lon points
# (simple equirectangular-ish with cos(mean_lat) scaling)
# -----------------------
def meters_between(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    lat1, lon1 = a
    lat2, lon2 = b
    mean_lat = math.radians((lat1 + lat2) / 2.0)
    dx = (lat1 - lat2) * 111000.0
    dy = (lon1 - lon2) * 111000.0 * math.cos(mean_lat)
    return math.hypot(dx, dy)

# -----------------------
# Build empty GRAPH (all nodes present) and helper to add bidirectional edge
# -----------------------
GRAPH: Dict[str, Dict[str, float]] = {n: {} for n in NODE_POS.keys()}

def _add_bidir(u: str, v: str) -> None:
    """Add bidirectional weighted edge between u and v using NODE_POS distances."""
    if u not in NODE_POS:
        raise KeyError(f"Unknown node '{u}'")
    if v not in NODE_POS:
        raise KeyError(f"Unknown node '{v}'")
    d = round(meters_between(NODE_POS[u], NODE_POS[v]), 3)
    GRAPH.setdefault(u, {})[v] = d
    GRAPH.setdefault(v, {})[u] = d

# -----------------------
# Build ROAD-ROAD connectivity: connect each road to its k nearest road neighbors
# (k = 2) — creates a local road graph backbone.
# -----------------------
road_keys = sorted(ROAD_NODES.keys())
k = 2

for r in road_keys:
    # compute distances to other roads
    others = []
    for o in road_keys:
        if o == r:
            continue
        d = meters_between(NODE_POS[r], NODE_POS[o])
        others.append((d, o))
    others.sort(key=lambda x: x[0])
    for d, o in others[:k]:
        _add_bidir(r, o)

# Ensure the road subgraph is connected: if multiple components exist,
# connect components with shortest bridging edges until single component.
def _road_components() -> List[List[str]]:
    seen = set()
    comps: List[List[str]] = []
    for r in road_keys:
        if r in seen:
            continue
        stack = [r]
        comp = []
        while stack:
            x = stack.pop()
            if x in seen:
                continue
            seen.add(x)
            comp.append(x)
            for y in GRAPH.get(x, {}).keys():
                if y.startswith("road") and y not in seen:
                    stack.append(y)
        comps.append(comp)
    return comps

comps = _road_components()
while len(comps) > 1:
    best = None
    bestd = float("inf")
    for i in range(len(comps)):
        for j in range(i + 1, len(comps)):
            for a in comps[i]:
                for b in comps[j]:
                    d = meters_between(NODE_POS[a], NODE_POS[b])
                    if d < bestd:
                        bestd = d
                        best = (a, b, d)
    if best is None:
        break
    a, b, d = best
    _add_bidir(a, b)
    comps = _road_components()

# -----------------------
# AUTO-CONNECT EACH POI (non-road) to its single NEAREST ROAD node (Option A)
# -----------------------
poi_nodes = [n for n in NODE_POS.keys() if not str(n).lower().startswith("road")]
for poi in poi_nodes:
    # find nearest road
    best_road = None
    best_dist = float("inf")
    for r in road_keys:
        d = meters_between(NODE_POS[poi], NODE_POS[r])
        if d < best_dist:
            best_dist = d
            best_road = r
    if best_road is not None:
        _add_bidir(poi, best_road)

# -----------------------
# cleaned_graph: for UI selectors — remove road nodes & edges to roads
# -----------------------
def cleaned_graph(graph: Dict[str, Dict[str, float]]) -> Dict[str, Dict[str, float]]:
    """
    Return a copy of the graph with road nodes removed from the returned structure.
    This is intended for UI selectors so users cannot pick internal 'road X' nodes.
    """
    newg: Dict[str, Dict[str, float]] = {}
    for u, nbrs in graph.items():
        if str(u).lower().startswith("road"):
            continue
        # keep only neighbors that are not road nodes (so UI shows POI->POI edges only)
        filtered = {v: w for v, w in nbrs.items() if not str(v).lower().startswith("road")}
        newg[u] = filtered
    return newg

# -----------------------
# DIJKSTRA implementation (standard)
# -----------------------
def reconstruct(parents: Dict[str, Any], start: str, goal: str) -> List[str]:
    path: List[str] = []
    cur = goal
    while cur is not None:
        path.append(cur)
        cur = parents.get(cur)
    path.reverse()
    return path

def dijkstra(graph: Dict[str, Dict[str, float]], start: str, goal: str) -> Tuple[List[str], float]:
    """
    Returns (path_list, cost) where path_list includes road nodes.
    If start or goal are not in graph, returns ([], inf).
    """
    if start not in graph or goal not in graph:
        return [], float("inf")

    pq: List[Tuple[float, str]] = [(0.0, start)]
    dist: Dict[str, float] = {n: float("inf") for n in graph}
    dist[start] = 0.0
    parents: Dict[str, Any] = {start: None}
    visited = set()

    while pq:
        cost, node = heapq.heappop(pq)
        if node in visited:
            continue
        visited.add(node)

        if node == goal:
            return reconstruct(parents, start, goal), cost

        for nei, w in graph.get(node, {}).items():
            nc = cost + float(w)
            if nc < dist.get(nei, float("inf")):
                dist[nei] = nc
                parents[nei] = node
                heapq.heappush(pq, (nc, nei))

    return [], float("inf")

# -----------------------
# Export API
# -----------------------
_all_ = ["dijkstra", "GRAPH", "NODE_POS", "cleaned_graph"]

# -----------------------
# prepare GRAPH variable (already filled while building)
# -----------------------
# GRAPH already populated via _add_bidir calls above.
# No CLI or sample execution code included — safe to import in Flask.