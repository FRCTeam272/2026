package frc.lib.math.graphing;

import java.util.function.DoubleSupplier;

public class Point {
    private DoubleSupplier x;
    private DoubleSupplier y;

    public Point(DoubleSupplier x, DoubleSupplier y) {
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return x.getAsDouble();
    }

    public double getY() {
        return y.getAsDouble();
    }

    @Override
    public String toString() {
        return "(" + x + ", " + y + ")";
    }
}
