// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;

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
  final PIDSettings shooterLowPID = Constants.SHOOTER_LOW_PID_SETTINGS;
  final PIDSettings shooterMidPID = Constants.SHOOTER_MID_PID_SETTINGS;
  int speedThreshold = 50;
  int angleThreshold = 2;

  public TrimPot hoodTrim = new TrimPot("HoodTrim");
  public TrimPot flywheelTrim = new TrimPot("FlywheelTrim");

  public double targetVelocity = 3500;

  public Shooter() {
    flywheel = new TalonFxContainer(FLYWHEEL_LOCATION, true);
    flywheelFollower = new TalonFxContainer(FLYWHEEL_FOLLOWER_LOCATION, true);

    for (var  motor : new TalonFxContainer[]{flywheel, flywheelFollower}) {
      motor.assignPIDValues(shooterLowPID.kP, shooterLowPID.kI, shooterLowPID.kD, 0);
      motor.assignFF(shooterLowPID.kS, shooterLowPID.kV, shooterLowPID.kA, 0, 0);

      // TODO: add addtional Slots as needed      
      motor.assignPIDValues(shooterMidPID.kP, shooterMidPID.kI, shooterMidPID.kD, 1);
      motor.assignFF(shooterMidPID.kS, shooterMidPID.kV, shooterMidPID.kA, 0, 1);
      
      motor.setBreakMode(false);
      motor.configurator.Audio.BeepOnConfig = false;
      motor.motor.getVelocity().setUpdateFrequency(20);
      motor.applyConfig();  
    }
    
    flywheelFollower.setupAsFollowerMotor(flywheel, false);

    hood = new SparkMAXContainer(HOOD_LOCATION);
    hood.assignPIDValues(.1, 0, 0);
    hood.setCurrentLimit(40);
    
    this.setupSmartDashboard();
  }

  private void setupSmartDashboard(){
    
    SmartDashboard.putNumber("FlyWheel/TargetVelocity", targetVelocity);

    SmartDashboard.putNumber("Shooter/P", shooterLowPID.kP);
    SmartDashboard.putNumber("Shooter/I", shooterLowPID.kI);
    SmartDashboard.putNumber("Shooter/D", shooterLowPID.kD);
    SmartDashboard.putNumber("Shooter/kV", shooterLowPID.kV);
    SmartDashboard.putNumber("Shooter/kA", shooterLowPID.kA);
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

  private void dynamicPID(double kP, double kI, double kD, TalonFxContainer motor){
    motor.assignPIDValues(kP, kI, kD, motor.currentSlot);
  }
  
  private void dynamicFeedForward(double kV, double kA, TalonFxContainer motor){
    motor.assignFF(0, kV, kA, 0, motor.currentSlot);
  }

  @Override
  public void periodic() {
    // if(targetVelocity < 3600){
    //   flywheel.SetPIDSlot(0);
    //   setupSmartDashboard(shooterLowPID);
    // } else if(targetVelocity > 3600){
    //   flywheel.SetPIDSlot(1);
    //   setupSmartDashboard(shooterMidPID);
    // }

    // This method will be called once per scheduler run
    targetVelocity = SmartDashboard.getNumber("FlyWheel/TargetVelocity", targetVelocity);
    SmartDashboard.putNumber("FlyWheel/CurrentVelocity", flywheel.getVelocity());
    hood.reportMotor("ShooterHood");
    // If we aren't connected to FMS, allow for dynamic PID tuning
    if(!DriverStation.isFMSAttached()){
      final double p = SmartDashboard.getNumber("Shooter/P", shooterLowPID.kP);
      final double i = SmartDashboard.getNumber("Shooter/I", shooterLowPID.kI);
      final double d = SmartDashboard.getNumber("Shooter/D", shooterLowPID.kD);
      final double v = SmartDashboard.getNumber("Shooter/kV", shooterLowPID.kV);
      final double a = SmartDashboard.getNumber("Shooter/kA", shooterLowPID.kA);  
      this.targetVelocity = SmartDashboard.getNumber("TargetVelocity", this.targetVelocity);      
      
      if(p != shooterLowPID.kP || i != shooterLowPID.kI || d != shooterLowPID.kD || v != shooterLowPID.kV || a != shooterLowPID.kA){
        for (TalonFxContainer motor : new TalonFxContainer[]{flywheel, flywheelFollower}) {
          dynamicPID(p, i, d, motor);
          dynamicFeedForward(v, a, motor);
        }
      }
      
      flywheel.getPID("Shooter/PID_Actual/");
      flywheel.reportMotor("ShooterVals");
    }
  }
}
