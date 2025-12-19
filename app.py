from flask import Flask, render_template, request, jsonify
from pathfinding import dijkstra, GRAPH, NODE_POS, cleaned_graph
import math

app = Flask(__name__)


def meters_between(a, b):
    """Same approx as in pathfinding.py (so weights stay consistent)."""
    lat1, lon1 = a
    lat2, lon2 = b
    mean_lat = math.radians((lat1 + lat2) / 2.0)
    dx = (lat1 - lat2) * 111000.0
    dy = (lon1 - lon2) * 111000.0 * math.cos(mean_lat)
    return math.hypot(dx, dy)


@app.route('/')
def index():
    # hide internal 'road' nodes from the UI selectors
    nodes = sorted([n for n in GRAPH.keys() if not str(n).lower().startswith('road')])
    return render_template(
        'index.html',
        nodes=nodes,
        node_pos=NODE_POS,
        graph=cleaned_graph(GRAPH),
        map_key=" "  # <<--- put your real key here
    )


@app.route('/api/route', methods=['POST'])
def route():
    data = request.get_json()
    start = data.get('start')
    end = data.get('end')
    algo = data.get('algorithm', 'dijkstra')
    user_loc = data.get('userLocation')  # [lat, lon] from frontend or None

    # Clean previous temp user node if exists
    if "__user__" in GRAPH:
        GRAPH.pop("__user__", None)
    if "__user__" in NODE_POS:
        NODE_POS.pop("__user__", None)

    # If start is user location, create a temp '__user__' node and hook to nearest road
    if start == "__user_location__":
        if not user_loc or len(user_loc) != 2:
            return jsonify({'error': 'User location not provided properly'}), 400

        ux, uy = float(user_loc[0]), float(user_loc[1])
        temp_node = "__user__"

        # add position
        NODE_POS[temp_node] = (ux, uy)
        GRAPH[temp_node] = {}

        # find nearest road node
        nearest_road = None
        nearest_dist = float("inf")

        for name, pos in NODE_POS.items():
            if str(name).lower().startswith("road"):
                d = meters_between((ux, uy), pos)
                if d < nearest_dist:
                    nearest_dist = d
                    nearest_road = name

        if nearest_road is None:
            return jsonify({'error': 'No road nodes found for linking user location'}), 500

        # connect temp user node to nearest road
        GRAPH[temp_node][nearest_road] = round(nearest_dist, 3)
        GRAPH[nearest_road][temp_node] = round(nearest_dist, 3)

        # update start to use temp node
        start = temp_node

    # Use a cleaned graph for UI validation (hide internal road nodes there)
    ui_graph = cleaned_graph(GRAPH)

    # block selecting internal roads directly
    if str(start).lower().startswith('road') or str(end).lower().startswith('road'):
        return jsonify({'error': 'Invalid start or end (road nodes are internal)'}), 400

    if start not in ui_graph or end not in ui_graph:
        return jsonify({'error': 'Invalid start or end'}), 400

    # Perform pathfinding on the full internal GRAPH so returned path includes road nodes
    search_graph = GRAPH
    path, cost = dijkstra(search_graph, start, end)

    # Build a simplified display path (hide internal road nodes)
    display_nodes = [n for n in path if not str(n).lower().startswith('road')]
    if len(display_nodes) >= 2:
        display_path = [display_nodes[0], display_nodes[-1]]
    else:
        display_path = display_nodes

    return jsonify({'path': path, 'display_path': display_path, 'cost': cost})


if __name__ == '__main__':
    app.run(debug=True)
