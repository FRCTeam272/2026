// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utils.SparkMAXContainer;
import frc.lib.utils.TalonFxContainer;
import frc.robot.Constants;
import frc.lib.utils.PIDSettings;


public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  
  TalonFxContainer flywheelLeader;
  TalonFxContainer flywheelFollower;
  SparkMAXContainer hood;

  final int FLYWHEEL_LOCATION = 4;
  final int FLYWHEEL_FOLLOWER_LOCATION = 6;
  final int HOOD_LOCATION = 5;
  final PIDSettings shooterPID = Constants.SHOOTER_PID_SETTINGS;
  int speedThreshold = 50;
  int angleThreshold = 2;

  public double targetVelocity = 3000;

  public Shooter() {
    flywheelLeader = new TalonFxContainer(FLYWHEEL_LOCATION, true);
    flywheelFollower = new TalonFxContainer(FLYWHEEL_FOLLOWER_LOCATION, true);
    flywheelFollower.setupAsFollowerMotor(flywheelLeader, true);

    flywheelLeader.assignPIDValues(shooterPID.kP, shooterPID.kI, shooterPID.kD);
    flywheelLeader.assignFF(shooterPID.kS, shooterPID.kV, shooterPID.kA, 0);
    flywheelLeader.setBreakMode(false);
    hood = new SparkMAXContainer(HOOD_LOCATION);  
    SmartDashboard.putNumber("FlyWheel/TargetVelocity", targetVelocity);

    SmartDashboard.putNumber("Shooter/P", shooterPID.kP);
    SmartDashboard.putNumber("Shooter/I", shooterPID.kI);
    SmartDashboard.putNumber("Shooter/D", shooterPID.kD);
    SmartDashboard.putNumber("Shooter/kV", shooterPID.kV);
    SmartDashboard.putNumber("Shooter/kA", shooterPID.kA);
  }

  public boolean SpinWheel(double target_velocity){
    if(target_velocity > 0) target_velocity = -target_velocity;
    flywheelLeader.motor.set(target_velocity);
    return true;
    
    // forces flywheel to be negative
    // if(target_velocity > 0) target_velocity = -target_velocity;
    
    // final double current_velocity = flywheel.setVelocity(target_velocity);
    // return Math.abs(current_velocity - target_velocity) < speedThreshold;
  }

  public boolean AdjustHood(double target_angle){
    return hood.goToPostion(target_angle, angleThreshold);
  }

  private void dynamicPID(double kP, double kI, double kD){
    flywheelLeader.assignPIDValues(kP, kI, kD);
    flywheelFollower.assignPIDValues(kP, kI, kD);
  }
  
  private void dynamicFeedForward(double kV, double kA){
    flywheelLeader.assignFF(0, kV, kA, 0);
    flywheelFollower.assignFF(0, kV, kA, 0);
  }

  @Override 
  public void periodic() {
    // This method will be called once per scheduler run
    targetVelocity = SmartDashboard.getNumber("FlyWheel/TargetVelocity", targetVelocity);
    SmartDashboard.putNumber("FlyWheel/CurrentVelocity", flywheelLeader.getVelocity());

    // If we aren't connected to FMS, allow for dynamic PID tuning
    if(!DriverStation.isFMSAttached()){
      final double p = SmartDashboard.getNumber("Shooter/P", shooterPID.kP);
      final double i = SmartDashboard.getNumber("Shooter/I", shooterPID.kI);
      final double d = SmartDashboard.getNumber("Shooter/D", shooterPID.kD);
      final double v = SmartDashboard.getNumber("Shooter/kV", shooterPID.kV);
      final double a = SmartDashboard.getNumber("Shooter/kA", shooterPID.kA);      
      dynamicPID(p, i, d);
      dynamicFeedForward(v, a);

      flywheelLeader.getPID("Shooter/PID_Actual/");
      flywheelLeader.reportMotor("ShooterVals");
    }
    
  }
}
