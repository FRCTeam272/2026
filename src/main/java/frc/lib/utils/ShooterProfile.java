package frc.lib.utils;

import java.util.function.DoubleSupplier;
import java.util.function.Function;

public class ShooterProfile {
    public PIDSettings settings;
    public Function<Double, Double> calculateHood;
    public int minDistance;
    public int maxDistance;
    /**
    * Constructor for Shooter Profile
    * @param settings
    * @param calculateHood
    * @param minDistance
    * @param maxDistance
    */
    public ShooterProfile(PIDSettings settings, Function<Double, Double> calculateHood, int minDistance, int maxDistance){
        this.settings = settings;
        this.calculateHood = calculateHood;
        this.minDistance = minDistance;
        this.maxDistance = maxDistance;
    }

    

    public boolean isInRange(int distance) {
        return distance >= minDistance && distance <= maxDistance;
    }
}
