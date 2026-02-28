// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utils.SparkMAXContainer;
import frc.lib.utils.TalonFxContainer;
import frc.lib.utils.TrimPot;
import frc.robot.Constants;
import frc.lib.utils.MotorContainer;
import frc.lib.utils.PIDSettings;


public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  TalonFxContainer flywheel;
  TalonFxContainer flywheelFollower;
  SparkMAXContainer hood;

  final int FLYWHEEL_LOCATION = 4;
  final int FLYWHEEL_FOLLOWER_LOCATION = 6;
  final int HOOD_LOCATION = 5;
  int speedThreshold = 50;
  int angleThreshold = 2;

  
  public boolean useAutoFlywheel = true;

  public TrimPot hoodTrim = new TrimPot("HoodTrim");
  public TrimPot flywheelTrim = new TrimPot("FlywheelTrim");

  public double targetVelocity = 3500;

  // Cache last PID values to prevent constant re-configuration
  private double lastP, lastI, lastD, lastV, lastA;

  public Shooter() {
    flywheel = new TalonFxContainer(FLYWHEEL_LOCATION, true);
    flywheelFollower = new TalonFxContainer(FLYWHEEL_FOLLOWER_LOCATION, true);


    for (var  motor : new TalonFxContainer[]{flywheel, flywheelFollower}) {
      
      motor.assignPIDSettings(Constants.SHOOTER_LOW_PID_SETTINGS, 0);
      motor.assignPIDSettings(Constants.SHOOTER_MID_PID_SETTINGS, 1);
      motor.assignPIDSettings(Constants.SHOOTER_HIGH_PID_SETTINGS, 2);

      motor.setBreakMode(false);
      motor.configurator.Audio.BeepOnConfig = false;
      motor.motor.getVelocity().setUpdateFrequency(20);
      motor.applyConfig();  
    }
    
    flywheelFollower.setupAsFollowerMotor(flywheel, false);

    hood = new SparkMAXContainer(HOOD_LOCATION);
    hood.assignPIDValues(.1, 0, 0);
    hood.setCurrentLimit(40);

    lastP = Constants.SHOOTER_LOW_PID_SETTINGS.kP;
    lastI = Constants.SHOOTER_LOW_PID_SETTINGS.kI;
    lastD = Constants.SHOOTER_LOW_PID_SETTINGS.kD;
    lastV = Constants.SHOOTER_LOW_PID_SETTINGS.kV;
    lastA = Constants.SHOOTER_LOW_PID_SETTINGS.kA;
    
    this.setupSmartDashboard();
  }

  private void setupSmartDashboard(){
    
    SmartDashboard.putNumber("FlyWheel/TargetVelocity", targetVelocity);
    PIDSettings settings;
    switch (flywheel.currentSlot) {
      case 0:
        settings = Constants.SHOOTER_LOW_PID_SETTINGS;
        break;
      case 1:
        settings = Constants.SHOOTER_MID_PID_SETTINGS;
        break;
      default:
        settings = Constants.SHOOTER_HIGH_PID_SETTINGS;
        break;
    }
    setupSmartDashboard(settings);
  }

  private void setupSmartDashboard(PIDSettings pidConstants){
    SmartDashboard.putNumber("Shooter/P", pidConstants.kP);
    SmartDashboard.putNumber("Shooter/I", pidConstants.kI);
    SmartDashboard.putNumber("Shooter/D", pidConstants.kD);
    SmartDashboard.putNumber("Shooter/kV", pidConstants.kV);
    SmartDashboard.putNumber("Shooter/kA", pidConstants.kA);
  }

  public boolean TrueStop(){
    flywheel.motor.set(0);
    return true;
  }

  public boolean SpinWheel(double target_velocity){
    if(target_velocity == 0) return TrueStop();
    target_velocity = -(Math.abs(target_velocity) + flywheelTrim.adjusterValue);
    return flywheel.setVelocity(target_velocity, 250);
  }

  public boolean AdjustHood(double target_angle){
    return hood.goToPostion(target_angle + hoodTrim.adjusterValue, angleThreshold);
    // hood.motor.set(.2 * Math.signum(target_angle));
    // return true;
  }

  public boolean AdjustHoodIncremental(double value){
    var target = hood.getPosition() + value;
    if(target > 0) target = 0;
    if(target < -6.6) target = -6.6;
    return hood.goToPostion(target, 0);
  }

  final TalonFxContainer[] flywheels = new TalonFxContainer[] {flywheel, flywheelFollower}; 
  public void UpdatePID(PIDSettings settings){
    for (TalonFxContainer motor : flywheels) {
      dynamicPID(settings.kP, settings.kI, settings.kD, motor);
      dynamicFeedForward(settings.kV, settings.kA, motor);
    }
  }

  private void dynamicPID(double kP, double kI, double kD, TalonFxContainer motor){
    motor.assignPIDValues(kP, kI, kD, motor.currentSlot);
  }
  
  private void dynamicFeedForward(double kV, double kA, TalonFxContainer motor){
    motor.assignFF(0, kV, kA, 0, motor.currentSlot);
  }

  double lastTargetVelocity = targetVelocity;
  @Override
  public void periodic() {
    if(this.useAutoFlywheel){
      if(targetVelocity == 0) {

      }
      else if(targetVelocity < 5000){
        flywheel.currentSlot = 0;
      } else if(targetVelocity < 5500){
        flywheel.currentSlot = 1;
      } else {
        flywheel.currentSlot = 2;
      }
    }
    

    // if(lastTargetVelocity != targetVelocity){
    //   this.SpinWheel(targetVelocity);
    //   lastTargetVelocity = targetVelocity;
    // }

    // This method will be called once per scheduler run
    targetVelocity = SmartDashboard.getNumber("FlyWheel/TargetVelocity", targetVelocity);
    SmartDashboard.putNumber("FlyWheel/CurrentVelocity/Main", flywheel.getVelocity());
    SmartDashboard.putNumber("FlyWheel/CurrentVelocity/Follower", flywheelFollower.getVelocity());
    hood.reportMotor("ShooterHood");
    // If we aren't connected to FMS, allow for dynamic PID tuning
    if(!DriverStation.isFMSAttached()){
      final double p = SmartDashboard.getNumber("Shooter/P", lastP);
      final double i = SmartDashboard.getNumber("Shooter/I", lastI);
      final double d = SmartDashboard.getNumber("Shooter/D", lastD);
      final double v = SmartDashboard.getNumber("Shooter/kV", lastV);
      final double a = SmartDashboard.getNumber("Shooter/kA", lastA);      
    
      // ONLY configure if values have changed
      if (p != lastP || i != lastI || d != lastD) {
        dynamicPID(p, i, d, flywheel);
        dynamicPID(p, i, d, flywheelFollower);
        lastP = p;
        lastI = i;
        lastD = d;
      }

      if (v != lastV || a != lastA) {
        dynamicFeedForward(v, a, flywheel);
        dynamicFeedForward(v, a, flywheelFollower);
        lastV = v;
        lastA = a;
      }
      
      flywheel.getPID("Shooter/PID_Actual/");
      flywheel.reportMotor("ShooterVals");
    }
  }
}
