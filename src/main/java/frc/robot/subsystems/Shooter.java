// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utils.SparkMAXContainer;
import frc.robot.Constants;
import frc.lib.utils.PIDSettings;


public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  
  SparkMAXContainer flywheel;
  SparkMAXContainer hood;

  final int FLYWHEEL_LOCATION = 4;
  final int HOOD_LOCATION = 5;
  final PIDSettings shooterPID = Constants.SHOOTER_PID_SETTINGS;
  
  int speedThreshold = 50;
  int angleThreshold = 2;

  boolean usePID = false;
  
  public double targetVelocity = 3000;

  public Shooter() {
    flywheel = new SparkMAXContainer(FLYWHEEL_LOCATION);

    flywheel.assignPIDValues(shooterPID.kP, shooterPID.kI, shooterPID.kD);
    flywheel.assignFF(shooterPID.kS, shooterPID.kV, shooterPID.kA, 0);
    flywheel.setBreakMode(false);
    hood = new SparkMAXContainer(HOOD_LOCATION);  
    SmartDashboard.putNumber("FlyWheel/TargetVelocity", targetVelocity);

    SmartDashboard.putNumber("Shooter/P", shooterPID.kP);
    SmartDashboard.putNumber("Shooter/I", shooterPID.kI);
    SmartDashboard.putNumber("Shooter/D", shooterPID.kD);
    SmartDashboard.putNumber("Shooter/kV", shooterPID.kV);
    SmartDashboard.putNumber("Shooter/kA", shooterPID.kA);

    SmartDashboard.putBoolean("Shooter/UsePID", usePID);
  }

  public boolean SpinWheel(double target_velocity){
    if(usePID){
      // forces flywheel to be negative
      if(target_velocity > 0) target_velocity = -target_velocity;
      final double current_velocity = flywheel.setVelocity(target_velocity);
      return Math.abs(current_velocity - target_velocity) < speedThreshold;
    }else{
      if(target_velocity > 0) target_velocity = -target_velocity;
      flywheel.motor.set(target_velocity);
      return true;  
    }
  }

  public boolean AdjustHood(double target_angle){
    return hood.goToPostion(target_angle, angleThreshold);
  }

  private void dynamicPID(double kP, double kI, double kD){
    flywheel.assignPIDValues(kP, kI, kD);
  }
  
  private void dynamicFeedForward(double kV, double kA){
    flywheel.assignFF(0, kV, kA, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    targetVelocity = SmartDashboard.getNumber("FlyWheel/TargetVelocity", targetVelocity);
    SmartDashboard.putNumber("FlyWheel/CurrentVelocity", flywheel.getVelocity());

    // If we aren't connected to FMS, allow for dynamic PID tuning
    if(!DriverStation.isFMSAttached()){
      final double p = SmartDashboard.getNumber("Shooter/P", shooterPID.kP);
      final double i = SmartDashboard.getNumber("Shooter/I", shooterPID.kI);
      final double d = SmartDashboard.getNumber("Shooter/D", shooterPID.kD);
      final double v = SmartDashboard.getNumber("Shooter/kV", shooterPID.kV);
      final double a = SmartDashboard.getNumber("Shooter/kA", shooterPID.kA);      
      dynamicPID(p, i, d);
      dynamicFeedForward(v, a);

      usePID = SmartDashboard.getBoolean("Shooter/UsePID", usePID);

      flywheel.getPID("Shooter/PID_Actual/");
      flywheel.reportMotor("ShooterVals");
    }
    
  }
}
