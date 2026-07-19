package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Circle {
    // Class to represent a circle
    private double r;
    private Translation2d center;

    public Circle(Translation2d p1, Translation2d p2, Translation2d p3) {

        double x1 = p1.getX(), y1 = p1.getY();
        double x2 = p2.getX(), y2 = p2.getY();
        double x3 = p3.getX(), y3 = p3.getY();

        double d = 2 * (x1 * (y2 - y3) +
                x2 * (y3 - y1) +
                x3 * (y1 - y2));

        double h = ((x1 * x1 + y1 * y1) * (y2 - y3) +
                (x2 * x2 + y2 * y2) * (y3 - y1) +
                (x3 * x3 + y3 * y3) * (y1 - y2)) / d;

        double k = ((x1 * x1 + y1 * y1) * (x3 - x2) +
                (x2 * x2 + y2 * y2) * (x1 - x3) +
                (x3 * x3 + y3 * y3) * (x2 - x1)) / d;

        double r = Math.hypot(x1 - h, y1 - k);
        this.center = new Translation2d(h, k);
        this.r = r;
    }

    public String toString() {
        String centerPoint = "centerPoint = " + "(" + center + ")" + "," + "radius = " + r;
        ;
        return centerPoint;
    }

    public Translation2d getPoint(double angle) {
        double x_origin = r * Math.cos(angle);
        double y_origin = r * Math.sin(angle);
        double x_circ = x_origin + center.getX();
        double y_circ = y_origin + center.getY();

        return new Translation2d(x_circ, y_circ);
    }

    public Translation2d getPoint(Rotation2d angle) {
        return getPoint(angle.getRadians());
    }

    public Translation2d getCenter() {
        return center;
    }

    public double getRadius() {
        return r;
    }

}
