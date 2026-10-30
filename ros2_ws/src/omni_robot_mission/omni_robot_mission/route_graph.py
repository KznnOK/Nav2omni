"""Waypoint graph: nearest straight-line-reachable entry node and A* search."""
import heapq
import math


class RouteGraph:

    def __init__(self, waypoint_lines, edge_lines):
        self.nodes = {}
        for line in waypoint_lines:
            name, x, y = line.split()
            self.nodes[name] = (float(x), float(y))
        self.neighbors = {name: set() for name in self.nodes}
        for line in edge_lines:
            a, b = line.split()
            if a not in self.nodes or b not in self.nodes:
                raise ValueError(f"edge with unknown waypoint: {line}")
            self.neighbors[a].add(b)
            self.neighbors[b].add(a)

    def distance(self, a, b):
        (ax, ay), (bx, by) = self.nodes[a], self.nodes[b]
        return math.hypot(ax - bx, ay - by)

    def nearest_reachable(self, x, y, is_clear):
        """Closest waypoint the robot can reach in a straight line.

        is_clear(start_xy, end_xy) checks the straight move against walls.
        """
        for name in sorted(self.nodes, key=lambda n: math.hypot(
                self.nodes[n][0] - x, self.nodes[n][1] - y)):
            if is_clear((x, y), self.nodes[name]):
                return name
        return None

    def shortest_path(self, start, goal):
        queue = [(0.0, start)]
        cost = {start: 0.0}
        parent = {start: None}
        while queue:
            _, node = heapq.heappop(queue)
            if node == goal:
                path = []
                while node is not None:
                    path.append(node)
                    node = parent[node]
                return path[::-1]
            for nxt in self.neighbors[node]:
                new_cost = cost[node] + self.distance(node, nxt)
                if new_cost < cost.get(nxt, float("inf")):
                    cost[nxt] = new_cost
                    parent[nxt] = node
                    heapq.heappush(
                        queue, (new_cost + self.distance(nxt, goal), nxt))
        return None

    def check_edges(self, is_clear):
        """Edges whose straight segment is blocked (for start-up validation)."""
        blocked = []
        for a in self.nodes:
            for b in self.neighbors[a]:
                if a < b and not is_clear(self.nodes[a], self.nodes[b]):
                    blocked.append((a, b))
        return blocked
