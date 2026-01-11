package frc.lib.math.graphing;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Line {
    private Point point1;
    private Point point2;

    public Line(Point point1, Point point2) {
        this.point1 = point1;
        this.point2 = point2;
        SmartDashboard.putString("Swerve Formula", this.toString());
        calculate(0);
    }

    public double getSlope() {
        
        if (point1.getX() == point2.getX()) {
            throw new ArithmeticException("Slope is undefined for vertical line");
        }
        return (point2.getY() - point1.getY()) / (point2.getX() - point1.getX());
    }

    public double calculate(double x) {
        SmartDashboard.putString("swerve speed", format(getSlope() * x + getYIntercept()));
        return getSlope() * x + getYIntercept();
    }

    public double getYIntercept() {
        return point1.getY() - getSlope() * point1.getX();
    }

    private String format(double number){
        return String.format("%.2f",number);
    }

    @Override
    public String toString() {
        return "y = " + format(getSlope()) + "x + " + format(getYIntercept());
    }
}
