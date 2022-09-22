package frc.robot.modules;

import org.opencv.core.Point;

public class Map {

    double radius;

    public Map(double radius_) {
        radius = radius_;
    }

    private Point sampleOne() {
        double x = radius * (Math.random() * 2 - 1);
        double y = radius * (Math.random() * 2 - 1);

        while (x * x + y * y > radius * radius) {
            x = radius * (Math.random() * 2 - 1);
            y = radius * (Math.random() * 2 - 1);
        }
        return new Point(x, y);
    }

    public Point[] sample(int num) {
        Point[] samples = new Point[num];

        for (int i = 0; i < num; i++) {
            samples[i] = sampleOne();
        }

        return samples;
    }

}
