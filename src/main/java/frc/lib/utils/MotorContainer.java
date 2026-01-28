package frc.lib.utils;

/**
 * a genralized wrapper for motor controllers so we can treat SparkMaxes like
 * Talons like Thrivtys
 */
public interface MotorContainer {
    /**
     * Assigns the defualt PID values to the motor assumes P = 0.1, I = 0, D = 0
     */
    public void assignPIDValues();

    /**
     * Assigns the PID values to the motor
     * 
     * @param P the P value
     * @param I the I value
     * @param D the D value
     */
    public void assignPIDValues(double P, double I, double D);

    public void assignFF(double kS, double kV, double kA, double kG);

    /**
     * Assigns this motor to follow another Motor of the same type
     * 
     * @param leader the motorContainer this should follow
     * @param invert weither or not this motor should be inverted from the other
     *               motor
     */
    public void setupAsFollowerMotor(MotorContainer leader, boolean invert);

    /**
     * Sets the gear ratio of the motor for finner postion control
     * 
     * @param gearRatio can be represeneted via a fraction
     */
    public void setGearRatio(double gearRatio);

    /**
     * Sends the motor to a specific position, returns true if it is within the
     * deadband (4 encoder ticks)
     * 
     * @param pos desired postion
     * @return true when within deadband
     */
    public boolean goToPostion(double pos);

    /**
     * Sends the motor to a specific position, returns true if it is within the
     * deadband see also {@link #goToPostion(double)}
     * 
     * @param pos      desired postion
     * @param deadband the deadband to be within, deadband should not be 0 but can
     *                 be as small as 1
     * @return true when within deadband
     */
    public boolean goToPostion(double pos, int deadband);

    /**
     * Sets the current limit of the motor
     * @param limit
     */
    public void setCurrentLimit(double limit);

    /**
     * Sets the break mode of the motor
     * 
     * @param isBreakMode true for break mode, false for coast mode
     */
    public void setBreakMode(boolean isBreakMode);
    /**
     * Gets the temperature of the motor
     * @return temperature in C
     */
    public double getMotorTemperatureInC();
    /**
     * Gets the temperature of the motor
     * @return temperature in F
    */
    public double getMotorTemperatureInF();

    /**
     * Gets PID as an array
     */
    public void getPID(String key);
    /**
     * Sends information about the motor to SmartDashboard, these calls should be
     * contained in an if(!DriverStation.isFMSAttached()) block to avoid flooding
     * the system in competition
     * 
     * @param key the header key smart dashboard will use
     */
    public void reportMotor(String key);
}
