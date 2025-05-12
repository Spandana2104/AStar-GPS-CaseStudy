import javax.swing.*;
import java.awt.*;
import java.util.*;
import java.util.List;

// Class representing a node in the grid for the A* algorithm
class Node {
    int x, y; // Coordinates of the node
    double g, h, f; // Cost values: g = cost from start, h = heuristic, f = g + h
    Node parent; // Parent node to reconstruct the path

    public Node(int x, int y) {
        this.x = x;
        this.y = y;
        this.g = Double.MAX_VALUE;
        this.h = 0;
        this.f = Double.MAX_VALUE;
        this.parent = null;
    }
}

// Main visualization and implementation class for A* algorithm
public class AStarVisualization extends JPanel {
    private int GRID_SIZE; // Grid size defined by user input
    private static final int CELL_SIZE = 15; // Size of each grid cell
    private int[][] grid; // Grid representation (0 = free, 1 = obstacle)
    private List<Node> path = new ArrayList<>(); // Final path from start to goal
    private Set<Node> exploredNodes = new HashSet<>(); // Explored nodes for visualization
    private Node startNode, goalNode; // Start and goal nodes
    private Map<String, Node> nodeMap = new HashMap<>(); // Mapping of nodes by coordinates

    // Constructor to initialize the grid and run A* algorithm
    public AStarVisualization() {
        inputGridData();
        if (grid[startNode.x][startNode.y] == 1 || grid[goalNode.x][goalNode.y] == 1) {
            System.out.println("Start or Goal node is blocked. Adjusting obstacles.");
            grid[startNode.x][startNode.y] = 0;
            grid[goalNode.x][goalNode.y] = 0;
        }
        aStar(startNode, goalNode);
        if (path.isEmpty()) {
            System.out.println("No path found.");
        } else {
            System.out.println("Path found with " + path.size() + " steps.");
        }
        setPreferredSize(new Dimension(GRID_SIZE * CELL_SIZE, GRID_SIZE * CELL_SIZE));
        repaint();
    }

    // Method to take input for grid size, start node, goal node, and obstacles
    private void inputGridData() {
        Scanner sc = new Scanner(System.in);
        System.out.println("Enter Grid Size (e.g., 50 for a 50x50 grid):");
        GRID_SIZE = sc.nextInt();
        grid = new int[GRID_SIZE][GRID_SIZE];
        
        System.out.println("Enter Start Node (x, y):");
        startNode = getNode(sc.nextInt(), sc.nextInt());
        System.out.println("Enter Goal Node (x, y):");
        goalNode = getNode(sc.nextInt(), sc.nextInt());

        System.out.println("Enter the number of random obstacles:");
        int numObstacles = sc.nextInt();
        Random rand = new Random();

        // Randomly generate obstacles
        for (int i = 0; i < numObstacles; i++) {
            int x = rand.nextInt(GRID_SIZE);
            int y = rand.nextInt(GRID_SIZE);
            if ((x != startNode.x || y != startNode.y) && (x != goalNode.x || y != goalNode.y)) {
                grid[x][y] = 1; // Mark obstacle
            }
        }
        sc.close();
    }

    // Get or create a node based on coordinates
    private Node getNode(int x, int y) {
        String key = x + "," + y;
        return nodeMap.computeIfAbsent(key, k -> new Node(x, y));
    }

    // Calculate the heuristic (Manhattan Distance) between two nodes
    private double heuristic(Node a, Node b) {
        return Math.abs(a.x - b.x) + Math.abs(a.y - b.y);
    }

    // A* Algorithm implementation
    private void aStar(Node start, Node goal) {
        PriorityQueue<Node> openSet = new PriorityQueue<>(Comparator.comparingDouble(n -> n.f));
        Set<String> closedSet = new HashSet<>();

        start.g = 0;
        start.h = heuristic(start, goal);
        start.f = start.g + start.h;
        openSet.add(start);

        while (!openSet.isEmpty()) {
            Node current = openSet.poll();
            exploredNodes.add(current);

            if (current.x == goal.x && current.y == goal.y) {
                reconstructPath(current);
                return;
            }

            closedSet.add(current.x + "," + current.y);

            // Check all four directions (up, down, left, right)
            for (int[] dir : new int[][]{{0, 1}, {1, 0}, {0, -1}, {-1, 0}}) {
                int nx = current.x + dir[0];
                int ny = current.y + dir[1];

                if (nx < 0 || ny < 0 || nx >= GRID_SIZE || ny >= GRID_SIZE || grid[nx][ny] == 1)
                    continue;

                if (closedSet.contains(nx + "," + ny)) continue;

                Node neighbor = getNode(nx, ny);
                double tentativeG = current.g + 1;

                if (tentativeG < neighbor.g) {
                    neighbor.g = tentativeG;
                    neighbor.h = heuristic(neighbor, goal);
                    neighbor.f = neighbor.g + neighbor.h;
                    neighbor.parent = current;
                    openSet.add(neighbor);
                }
            }
        }
    }

    // Reconstruct the path from goal to start using parent nodes
    private void reconstructPath(Node current) {
        path.clear();
        while (current != null) {
            path.add(current);
            current = current.parent;
        }
        Collections.reverse(path);
    }

    // Draw the grid, obstacles, explored nodes, path, and start/goal nodes
    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;

        for (int i = 0; i < GRID_SIZE; i++) {
            for (int j = 0; j < GRID_SIZE; j++) {
                if (grid[i][j] == 1) {
                    g2d.setColor(Color.BLACK); // Obstacles
                } else {
                    g2d.setColor(Color.WHITE); // Free space
                }
                g2d.fillRect(i * CELL_SIZE, j * CELL_SIZE, CELL_SIZE, CELL_SIZE);
                g2d.setColor(Color.BLACK); // Darker grid lines
                g2d.drawRect(i * CELL_SIZE, j * CELL_SIZE, CELL_SIZE, CELL_SIZE);
            }
        }

        g2d.setColor(Color.RED); // Explored nodes
        for (Node node : exploredNodes) {
            g2d.fillRect(node.x * CELL_SIZE, node.y * CELL_SIZE, CELL_SIZE, CELL_SIZE);
        }

        if (!path.isEmpty()) {
            g2d.setColor(Color.BLUE);
            for (Node node : path) {
                g2d.fillRect(node.x * CELL_SIZE, node.y * CELL_SIZE, CELL_SIZE, CELL_SIZE);
            }
        }

        // Draw Start and Goal
        g2d.setColor(Color.GREEN);
        g2d.fillRect(startNode.x * CELL_SIZE, startNode.y * CELL_SIZE, CELL_SIZE, CELL_SIZE);
        g2d.setColor(Color.RED);
        g2d.fillRect(goalNode.x * CELL_SIZE, goalNode.y * CELL_SIZE, CELL_SIZE, CELL_SIZE);
    }

    // Main method to run the visualization
    public static void main(String[] args) {
        JFrame frame = new JFrame("A* Pathfinding Visualization");
        AStarVisualization panel = new AStarVisualization();
        JScrollPane scrollPane = new JScrollPane(panel);
        frame.add(scrollPane);
        frame.pack();
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setVisible(true);
    }
}
