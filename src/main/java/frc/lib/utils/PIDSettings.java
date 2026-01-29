package frc.lib.utils;

/* 
kS - Volts
kV - Volts per velocity, Volts per motor RPM by default
kA - Volts per velocity/s, Volts per motor RPM/s by default
kG - Volts, Elevator/linear mechanism gravity feedforward
kCos - Volts, Arm/rotary mechanism gravity feedforward. Feedback sensor must be configured to 0 = horizontal
kCosRatio - Ratio, Converts feedback sensor readings to mechanism rotations
*/
public class PIDSettings {
    // PID Values
    public double kP;
    public double kI;
    public double kD;
    
    // Feed Forward Values
    public double kS;
    public double kV;
    public double kA;
    public double kG;
    public double kCos;
    public double kCosRatio;

    /**
     * Constructor for PID Settings
     * @param p
     * @param i
     * @param d
     */
    public PIDSettings(double p, double i, double d){
        kP = p;
        kI = i;
        kD = d;
    }

    /**
     * Constructor for PID Settings with Feed Forward
     * @param p
     * @param i
     * @param d
     * @param s
     * @param v
     * @param a
     * @param g
     * @param cos
     * @param cosRatio
     */
    public PIDSettings(double p, double i, double d, double s, double v, double a, double g, double cos, double cosRatio){
        kP = p;
        kI = i;
        kD = d;
        kS = s;
        kV = v;
        kA = a;
        kG = g;
        kCos = cos;
        kCosRatio = cosRatio;
    }

    // For Flywheels
    public PIDSettings(double p, double i, double d, double v, double a){
        kP = p;
        kI = i;
        kD = d;
        kV = v;
        kA = a;
    }
}
