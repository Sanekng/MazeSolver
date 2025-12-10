from collections import deque
import heapq

maze_grid = [
    [1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1],
    [1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1],
    [1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1],
    [1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
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
    stack = [start]
    visited = set([start])
    parent = {start: None}

    while stack:
        v = stack.pop()
        if v == end:
            path = []
            while v is not None:
                path.append(v)
                v = parent[v]
            return path[::-1], visited
        
        for neigh in graph[v]:
            if neigh not in visited:
                visited.add(neigh)
                parent[neigh] = v
                stack.append(neigh)

    return [], visited


def bfs_graph(graph, start, end):
    queue = deque([start])
    visited = set([start])
    parent = {start: None}

    while queue:
        v = queue.popleft()
        if v == end:
            path = []
            while v is not None:
                path.append(v)
                v = parent[v]
            return path[::-1], visited
        
        for neigh in graph[v]:
            if neigh not in visited:
                visited.add(neigh)
                parent[neigh] = v
                queue.append(neigh)

    return [], visited


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


def demo():
    start = (0, 1)
    end = (7, 19)
    
    graph = build_graph(maze_grid)

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


if __name__ == "__main__":
    demo()
