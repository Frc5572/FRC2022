package frc.robot.modules;

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
