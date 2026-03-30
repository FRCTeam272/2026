// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.utils;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

/** A container to generalize motor controllers */
public class TalonFxContainer implements MotorContainer{
    public TalonFX motor;
    public TalonFXConfiguration configurator;
    public int currentSlot = 0;
    
    /**
     * Creates a new TalonFxContainer
     * @param id the can id of the motor
     * THIS ASSUMES THE MOTOR IS BRUSHLESS
     */
    public TalonFxContainer(int id) {
        this.motor = new TalonFX(id);
        this.configurator = new TalonFXConfiguration();
    }

    public TalonFxContainer(int id, boolean isAKraken){
        this(id);
        if(isAKraken) this.setupKraken();
    }

    public void setupKraken(){
        // Current Limits (Important for Krakens!)
        this.configurator.CurrentLimits.StatorCurrentLimit = 50.0;
        this.configurator.CurrentLimits.StatorCurrentLimitEnable = true;
        this.configurator.CurrentLimits.SupplyCurrentLimit = 50.0;
        this.configurator.CurrentLimits.SupplyCurrentLimitEnable = true;
        this.applyConfig();
    }

    public void applyConfig(){
        this.motor.getConfigurator().apply(this.configurator);
    }

    /**
     * Assigns the defualt PID values to the motor assumes P = 0.1, I = 0, D = 0
     * see also {@link #assignPIDValues(double, double, double)}
     */
    @Override
    public void assignPIDValues() {
        this.assignPIDValues(0.1, 0, 0);
    }

    /**
     * Assigns the PID values to the motor
     * @param P the P value
     * @param I the I value
     * @param D the D value
     * See also {@link #assignPIDValues()}
     */
    @Override
    public void assignPIDValues(double P, double I, double D) {
        var slot = configurator.Slot0;
        slot.kP = P;
        slot.kI = I;
        slot.kD = D;
        this.applyConfig();
    }
    
    public void SetPIDSlot(int slot){
        currentSlot = slot;   
    }

    public void assignPIDSettings(PIDSettings settings, int slot){
        assignPIDValues(settings.kP, settings.kI, settings.kD, slot);
        assignFF(settings.kS, settings.kV, settings.kA, settings.kG, slot);
    }

    public void assignPIDValues(double P, double I, double D, int slot){
        switch (slot) {
            case 0:
                assignPIDSlot0(P, I, D);
                break;
            case 1:
                assignPIDSlot1(P, I, D);
                break;
            case 2:
                assignPIDSlot2(P, I, D);
                break;
            default:
                DriverStation.reportWarning("Invalid PID slot: " + slot + ". Valid slots are 0, 1, or 2.", false);
        }
    }

    private void assignPIDSlot0(double P, double I, double D){
        var slot = configurator.Slot0;
        slot.kP = P;
        slot.kI = I;
        slot.kD = D;
        this.applyConfig();
    }

    private void assignPIDSlot1(double P, double I, double D){
        var slot = configurator.Slot1;
        slot.kP = P;
        slot.kI = I;
        slot.kD = D;
        this.applyConfig();
    }

    private void assignPIDSlot2(double P, double I, double D){
        var slot = configurator.Slot2;
        slot.kP = P;
        slot.kI = I;
        slot.kD = D;
        this.applyConfig();
    }

    public void assignFF(double S, double V, double A, double G, int slot){
        switch (slot) {
            case 0:
                assignFFSlot0(S, V, A, G);
                break;
            case 1:
                assignFFSlot1(S, V, A, G);
                break;
            case 2:
                assignFFSlot2(S, V, A, G);
                break;
            default:
                DriverStation.reportWarning("Invalid PID slot: " + slot + ". Valid slots are 0, 1, or 2.", false);
        }
    }

    private void assignFFSlot0(double S, double V, double A, double G){
        var slot = configurator.Slot0;
        slot.kS = S;
        slot.kV = V;
        slot.kA = A;
        slot.kG = G;
        this.applyConfig();
    }
    
    private void assignFFSlot1(double S, double V, double A, double G){
        var slot = configurator.Slot1;
        slot.kS = S;
        slot.kV = V;
        slot.kA = A;
        slot.kG = G;
        this.applyConfig();
    }

    private void assignFFSlot2(double S, double V, double A, double G){
        var slot = configurator.Slot2;
        slot.kS = S;
        slot.kV = V;
        slot.kA = A;
        slot.kG = G;
        this.applyConfig();
    }

    /**
    * Assigns the Feed Forward values to the motor
    * @param A Acceleration feedforward gain. The units for this gain is dependent on the control mode. Since this gain is multiplied by the requested acceleration, the units should be defined as units of output per unit of requested input acceleration. For example, when controlling velocity using a duty cycle closed loop, the units for the acceleration feedfoward gain will be duty cycle per requested rot per sec², or 1/(rot per sec²).
    * @param G Gravity feedforward/feedback gain. The type of gravity compensation is selected by GravityType. This is added to the closed loop output. The sign is determined by the gravity type. The unit for this constant is dependent on the control mode, typically fractional duty cycle, voltage, or torque current.
    * @param S Static feedforward gain. This is added to the closed loop output. The unit for this constant is dependent on the control mode, typically fractional duty cycle, voltage, or torque current. The sign is typically determined by reference velocity when using position, velocity, and Motion Magic® closed loop modes. However, when using position closed loop with zero velocity reference (no motion profiling), the application can instead use the position closed loop error by setting the Static Feedforward Sign configuration parameter. When doing so, we recommend the minimal amount of kS, otherwise the motor output may dither when closed loop error is near zero.
    * @param V Velocity feedforward gain. The units for this gain is dependent on the control mode. Since this gain is multiplied by the requested velocity, the units should be defined as units of output per unit of requested input velocity. For example, when controlling velocity using a duty cycle closed loop, the units for the velocity feedfoward gain will be duty cycle per requested rps, or 1/rps.
    */
    @Override
    public void assignFF(double kS, double kV, double kA, double kG){
        var slot = configurator.Slot0;
        slot.kS = kS;
        slot.kV = kV;
        slot.kA = kA;
        slot.kG = kG;
        this.applyConfig();
    }

    /**
     * Assigns this motor to follow another Motor of the same type
     * * @param leader the motorContainer this should follow (Must be a TalonFXContainer)
     * * @param invert weither or not this motor should be inverted from the other
     */
    @Override
    public void setupAsFollowerMotor(MotorContainer leader, boolean invert) {
        if(leader instanceof TalonFxContainer) {
            TalonFxContainer lead = (TalonFxContainer) leader;
            this.motor.setControl(
                new Follower(
                    lead.motor.getDeviceID(),
                    invert ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned
                )
            );
        }
        else {
            throw new IllegalArgumentException("Leader must be a TalonFX");
        }
    }
    /**
     * Unimpleted until needed
     */
    @Override
    public void setGearRatio(double gearRatio) {
        configurator.Feedback.SensorToMechanismRatio = gearRatio;
        this.applyConfig();
    }

    /**
     * Sends the motor to a specific position, returns true if it is within the deadband (4 encoder ticks)
     * @param pos desired postion
     */
    @Override
    public boolean goToPostion(double pos) {
        return goToPostion(pos, 4);
    }


    /**
     * Sends the motor to a specific position, returns true if it is within the deadband see also {@link #goToPostion(double)}
     * @param pos desired postion
     * @param deadband the deadband to be within, deadband should not be 0 but can be as small as 1
     */
    @Override
    public boolean goToPostion(double pos, int deadband) {
        var request = new PositionDutyCycle(pos).withSlot(currentSlot);
        motor.setControl(request);
        return motor.getPosition().getValue().isNear(Angle.ofBaseUnits(pos, Degree), deadband);
    }
    /**
     * sets the current limit of the motor
     */
    @Override
    public void setCurrentLimit(double limit) {
        CurrentLimitsConfigs currentLimitsConfigs = configurator.CurrentLimits;
        currentLimitsConfigs.SupplyCurrentLimit = limit;
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;
        this.applyConfig();
    }

    /**
     * Sets the break mode of the motor
     * @param isBreakMode true for break mode, false for coast mode
     */
    @Override
    public void setBreakMode(boolean isBreakMode) {
        motor.setNeutralMode(isBreakMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    /**
     * Gets the temperature of the motor in Celsius
     */
    @Override
    public double getMotorTemperatureInC() {
        return this.motor.getDeviceTemp().getValue().in(Celsius);        
    }

    /**
     * Gets the temperature of the motor in Fahrenheit
     */
    @Override
    public double getMotorTemperatureInF() {
        return this.motor.getDeviceTemp().getValue().in(Fahrenheit);
    }

    /**
     * Reports the motor data to the SmartDashboard
     * can be paired with a {@link SmartDashboard.isFMSConnected()} for optimization reasons
     * @param key the key to report the data under
     */
    @Override
    public void reportMotor(String key) {
        SmartDashboard.putNumber(key + "/Encoder Value", motor.getPosition().getValue().in(Degree));
        SmartDashboard.putNumber(key + "/Velocity", motor.getVelocity().getValue().in(RPM));
        SmartDashboard.putNumber(key + "/Current", motor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber(key + "/Applied Output", motor.getMotorOutputStatus().getValueAsDouble());
    }

    public double getVelocity(){
        return motor.getVelocity().getValue().in(RPM);
    }

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    public boolean setVelocity(double target_velocity){
        // this.motor.setControl(null)
        motor.setControl(velocityRequest.withVelocity(target_velocity).withSlot(currentSlot));
        return getVelocity() == target_velocity;
    }

    public boolean setVelocity(double target_velocity, double velocityThreshold){
        // this.motor.setControl(null)
        motor.setControl(velocityRequest.withVelocity(target_velocity).withSlot(currentSlot));
        return Math.abs(getVelocity()) - Math.abs(target_velocity) < velocityThreshold;
    }

    @Override
    public void getPID(String key) {
        motor.getConfigurator().refresh(this.configurator);
        if(currentSlot == 0){
            // PID
            SmartDashboard.putNumber(key + "P", this.configurator.Slot0.kP);
            SmartDashboard.putNumber(key + "I", this.configurator.Slot0.kI);
            SmartDashboard.putNumber(key + "D", this.configurator.Slot0.kD);
            // FF
            SmartDashboard.putNumber(key + "FF/A", this.configurator.Slot0.kA);
            SmartDashboard.putNumber(key + "FF/V", this.configurator.Slot0.kV);
            SmartDashboard.putNumber(key + "FF/G", this.configurator.Slot0.kG);
        } else if(currentSlot == 1){
            // PID
            SmartDashboard.putNumber(key + "P", this.configurator.Slot1.kP);
            SmartDashboard.putNumber(key + "I", this.configurator.Slot1.kI);
            SmartDashboard.putNumber(key + "D", this.configurator.Slot1.kD);
            // FF
            SmartDashboard.putNumber(key + "FF/A", this.configurator.Slot1.kA);
            SmartDashboard.putNumber(key + "FF/V", this.configurator.Slot1.kV);
            SmartDashboard.putNumber(key + "FF/G", this.configurator.Slot1.kG);
        } else if(currentSlot == 2){
            // PID
            SmartDashboard.putNumber(key + "P", this.configurator.Slot2.kP);
            SmartDashboard.putNumber(key + "I", this.configurator.Slot2.kI);
            SmartDashboard.putNumber(key + "D", this.configurator.Slot2.kD);
            // FF
            SmartDashboard.putNumber(key + "FF/A", this.configurator.Slot2.kA);
            SmartDashboard.putNumber(key + "FF/V", this.configurator.Slot2.kV);
            SmartDashboard.putNumber(key + "FF/G", this.configurator.Slot2.kG);
        }
        
    }

    @Override
    public void goToVoltage(double voltage) {
        motor.setControl(new VoltageOut(voltage));
    }
}
