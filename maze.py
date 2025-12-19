from collections import deque
import heapq
import time

maze_grid = [
    [1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1],
    [1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1],
    [1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1],
    [1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1],
    [1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1],
    [1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1],
    [1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1],
    [1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1],
    [1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1],
    [1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1],
    [1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1],
    [1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
]

def print_maze(grid, path=None, visited=None):
    if path is None: path = []
    if visited is None: visited = []
    
    print("\nMaze:")
    for i in range(len(grid)):
        for j in range(len(grid[0])):
            if (i, j) in path:
                print("o", end=" ")
            elif (i, j) in visited:
                print("x", end=" ")
            elif grid[i][j] == 1:
                print("█", end=" ")
            else:
                print(".", end=" ")
        print()

def get_neighbors(grid, row, col):
    neighbors = []
    dirs = [(0,1), (1,0), (0,-1), (-1,0)]
    for dr, dc in dirs:
        r, c = row + dr, col + dc
        if 0 <= r < len(grid) and 0 <= c < len(grid[0]) and grid[r][c] == 0:
            neighbors.append((r, c))
    return neighbors

def build_graph(grid):
    graph = {}
    for r in range(len(grid)):
        for c in range(len(grid[0])):
            if grid[r][c] == 0:
                graph[(r, c)] = get_neighbors(grid, r, c)
    return graph


def dfs_graph(graph, start, end):
    visited = set()
    path = []
    all_visited = set()
    
    def dfs_recursive(current):
        nonlocal path, visited, all_visited
        
        visited.add(current)
        all_visited.add(current)
        path.append(current)
       
        if current == end:
            return True
        
        
        for neighbor in graph.get(current, []):
            if neighbor not in visited:
                if dfs_recursive(neighbor):
                    return True
        
        path.pop()
        visited.remove(current)
        return False
    
    success = dfs_recursive(start)
    
    if success:
        return path, all_visited
    else:
        return [], all_visited


def bfs_graph(graph, start, end):
    queue = deque()
    queue.append([start])
    visited = set([start])
    all_visited = set([start])
    
    while queue:
        current_path = queue.popleft()
        current_node = current_path[-1]
        
        if current_node == end:
            return current_path, all_visited

        for neighbor in graph.get(current_node, []):
            if neighbor not in visited:
                visited.add(neighbor)
                all_visited.add(neighbor)
                new_path = current_path.copy()
                new_path.append(neighbor)
                queue.append(new_path)
    
    return [], all_visited


def astar_graph(graph, start, end):
    def h(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    open_set = []
    heapq.heappush(open_set, (0, start))

    came_from = {start: None}
    g = {start: 0}
    visited = set()

    while open_set:
        _, current = heapq.heappop(open_set)
        visited.add(current)

        if current == end:
            path = []
            while current is not None:
                path.append(current)
                current = came_from[current]
            return path[::-1], visited

        for neigh in graph[current]:
            tentative = g[current] + 1
            if neigh not in g or tentative < g[neigh]:
                g[neigh] = tentative
                came_from[neigh] = current
                f = tentative + h(neigh, end)
                heapq.heappush(open_set, (f, neigh))

    return [], visited

def compare_algorithms(graph, start, end):
    print("Efficiency Metrics")

    results = {}

    # DFS
    t0 = time.perf_counter()
    dfs_path, dfs_vis = dfs_graph(graph, start, end)
    t1 = time.perf_counter()
    results["DFS"] = {
        "time": t1 - t0,
        "visited": len(dfs_vis),
        "path_len": len(dfs_path)
    }

    # BFS
    t0 = time.perf_counter()
    bfs_path, bfs_vis = bfs_graph(graph, start, end)
    t1 = time.perf_counter()
    results["BFS"] = {
        "time": t1 - t0,
        "visited": len(bfs_vis),
        "path_len": len(bfs_path)
    }

    # A*
    t0 = time.perf_counter()
    astar_path, astar_vis = astar_graph(graph, start, end)
    t1 = time.perf_counter()
    results["A*"] = {
        "time": t1 - t0,
        "visited": len(astar_vis),
        "path_len": len(astar_path)
    }

    # PRINT SUMMARY
    print("\nAlgorithm | Visited | Path Length | Time (sec)")
    print("-" * 50)

    for name, r in results.items():
        print(f"{name:8} | {r['visited']:9} | {r['path_len']:11} | {r['time']:.6f}")

    print("-" * 50)
    return results



def demo():
    start = (0, 1)
    end = (7, 1)
    
    graph = build_graph(maze_grid)
    for i in graph:
        print(i, " : ", graph[i])

    print_maze(maze_grid)

    dfs_path, dfs_vis = dfs_graph(graph, start, end)
    print("\nDFS:")
    print_maze(maze_grid, dfs_path, dfs_vis)

    bfs_path, bfs_vis = bfs_graph(graph, start, end)
    print("\nBFS:")
    print_maze(maze_grid, bfs_path, bfs_vis)

    astar_path, astar_vis = astar_graph(graph, start, end)
    print("\nA*:")
    print_maze(maze_grid, astar_path, astar_vis)
    
    compare_algorithms(graph, start, end)


if __name__ == "__main__":
    demo()
