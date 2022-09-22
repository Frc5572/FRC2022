package frc.robot.modules;

import java.util.ArrayList;
import java.util.List;
import java.util.PriorityQueue;
import org.opencv.core.Point;

public class PRM {

    Point[] nodes;
    int[][] edges;

    public PRM(Map map, int samples) {
        nodes = map.sample(samples);
        edges = new int[samples][];
        for (int i = 0; i < samples; i++) {
            edges[i] = new int[0];
        }
        for (int i = 0; i < samples; i++) {
            for (int j = i + 1; j < samples; j++) {
                Point p1 = nodes[i];
                Point p2 = nodes[j];
                if ((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) < 0.05 * 0.05) {
                    int[] temp = new int[edges[i].length + 1];
                    for (int k = 0; k < edges[i].length; k++) {
                        temp[k] = edges[i][k];
                    }
                    temp[edges[i].length] = j;
                    edges[i] = temp;
                    temp = new int[edges[j].length + 1];
                    for (int k = 0; k < edges[j].length; k++) {
                        temp[k] = edges[j][k];
                    }
                    temp[edges[j].length] = i;
                    edges[j] = temp;
                }
            }
        }
    }

    private static class Path {
        List<Point> path;
        double cost;
        double heuristic;

        Point goal;
        Point last;
        int lastId;

        Path(Point start, Point goal_, int start_id) {
            goal = goal_;
            path = new ArrayList<>();
            last = start;
            lastId = start_id;
            path.add(start);
            cost = 0;
            heuristic =
                (start.x - goal.x) * (start.x - goal.x) + (start.y - goal.y) * (start.y - goal.y);
        }

        void add(Point newPoint, int pid) {
            cost += (newPoint.x - last.x) * (newPoint.x - last.x)
                + (newPoint.y - last.y) * (newPoint.y - last.y);
            path.add(newPoint);
            last = newPoint;
            lastId = pid;
            heuristic = (newPoint.x - goal.x) * (newPoint.x - goal.x)
                + (newPoint.y - goal.y) * (newPoint.y - goal.y);
        }

        void addNoUpdate(Point newPoint) {
            path.add(newPoint);
        }

        Path copy() {
            Path p = new Path(path.get(0), goal, lastId);
            for (int i = 1; i < path.size(); i++) {
                p.addNoUpdate(path.get(i));
            }
            p.heuristic = heuristic;
            p.cost = cost;
            p.last = last;
            return p;
        }
    }

    public List<Point> plan(Point A, Point B) {
        int nodeA = 0;
        double currAlen =
            (A.x - nodes[0].x) * (A.x - nodes[0].x) + (A.y - nodes[0].y) * (A.y - nodes[0].y);
        int nodeB = 0;
        double currBlen =
            (B.x - nodes[0].x) * (B.x - nodes[0].x) + (B.y - nodes[0].y) * (B.y - nodes[0].y);
        for (int i = 1; i < nodes.length; i++) {
            double newAlen =
                (A.x - nodes[i].x) * (A.x - nodes[i].x) + (A.y - nodes[i].y) * (A.y - nodes[i].y);
            double newBlen =
                (B.x - nodes[i].x) * (B.x - nodes[i].x) + (B.y - nodes[i].y) * (B.y - nodes[i].y);
            if (newAlen < currAlen) {
                currAlen = newAlen;
                nodeA = i;
            }
            if (newBlen < currBlen) {
                currBlen = newBlen;
                nodeB = i;
            }
        }
        PriorityQueue<Path> openSet = new PriorityQueue<>((a, b) -> {

            double alen = a.heuristic + a.cost;
            double blen = b.heuristic + b.cost;

            return (int) Math.signum(alen - blen);

        });
        openSet.add(new Path(nodes[nodeA], nodes[nodeB], nodeA));

        while (!openSet.isEmpty()) {
            Path p = openSet.poll();
            if (p.heuristic < 0.001) {
                return p.path;
            }

            for (int i = 0; i < edges[p.lastId].length; i++) {
                int j = edges[p.lastId][i];

            }
        }

        return new ArrayList();
    }

    void debug() {
        System.out.println("Num Nodes: " + nodes.length);
        System.out.println("Adj List: ");
        for (int i = 0; i < edges.length; i++) {
            System.out.print("\t" + i + ": [");
            for (int j = 0; j < edges[i].length; j++) {
                if (j != 0) {
                    System.out.print(", ");
                }
                System.out.print(edges[i][j]);
            }
            System.out.println("]");
        }
    }

    public static void main(String[] argv) {
        Map m = new Map(5);
        PRM p = new PRM(m, 100);
        p.debug();
    }
}
