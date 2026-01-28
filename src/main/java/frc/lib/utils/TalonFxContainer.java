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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;

/** A container to generalize motor controllers */
public class TalonFxContainer implements MotorContainer{
    public TalonFX motor;
    public TalonFXConfiguration configurator;
    
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
        if(isAKraken){
            // Current Limits (Important for Krakens!)
            this.configurator.CurrentLimits.StatorCurrentLimit = 60.0;
            this.configurator.CurrentLimits.StatorCurrentLimitEnable = true;
            this.configurator.CurrentLimits.SupplyCurrentLimit = 40.0;
            this.configurator.CurrentLimits.SupplyCurrentLimitEnable = true;
            this.applyConfig();
        }
    }

    public void setupKraken(){
        this.configurator.CurrentLimits.StatorCurrentLimit = 60.0;
        this.configurator.CurrentLimits.StatorCurrentLimitEnable = true;
        this.configurator.CurrentLimits.SupplyCurrentLimit = 40.0;
        this.configurator.CurrentLimits.SupplyCurrentLimitEnable = true;
        this.applyConfig();
    }

    private void applyConfig(){
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
        var request = new PositionDutyCycle(pos);
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

    public double getVelocity(){
        return this.motor.getVelocity().getValue().in(RPM);
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

    @Override
    public void getPID(String key) {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        
        // PID
        SmartDashboard.putNumber(key + "P", configs.Slot0.kP);
        SmartDashboard.putNumber(key + "I", configs.Slot0.kI);
        SmartDashboard.putNumber(key + "D", configs.Slot0.kD);
        // FF
        SmartDashboard.putNumber(key + "FF/A", configs.Slot0.kA);
        SmartDashboard.putNumber(key + "FF/V", configs.Slot0.kV);
        SmartDashboard.putNumber(key + "FF/G", configs.Slot0.kG);
    }
}
