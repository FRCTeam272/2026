// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.HAL.SimPeriodicAfterCallback;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utils.SparkMAXContainer;


public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  
  SparkMAXContainer flywheel;
  SparkMAXContainer hood;

  final int FLYWHEEL_LOCATION = 4;
  final int HOOD_LOCATION = 5;

  int speedThreshold = 50;
  int angleThreshold = 2;

  public double targetVelocity = 3000;

  public Shooter() {
    flywheel = new SparkMAXContainer(FLYWHEEL_LOCATION);
    flywheel.assignPIDValues(0.03, 0, 0);
    flywheel.assignFeedForward(.01, .01);
    hood = new SparkMAXContainer(HOOD_LOCATION);  
    SmartDashboard.putNumber("FlyWheel/TargetVelocity", targetVelocity);
  }

  public boolean SpinWheel(double target_velocity){
    if(target_velocity > 0) target_velocity = -target_velocity;
    flywheel.motor.set(target_velocity);
    return true;
    // if(target_velocity > 0) target_velocity = -target_velocity;
    // final double current_velocity = flywheel.setVelocity(target_velocity);
    // return Math.abs(current_velocity - target_velocity) < speedThreshold;
  }

  public boolean AdjustHood(double target_angle){
    return hood.goToPostion(target_angle, angleThreshold);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    targetVelocity = SmartDashboard.getNumber("FlyWheel/TargetVelocity", targetVelocity);
    SmartDashboard.putNumber("FlyWheel/CurrentVelocity", flywheel.getVelocity());
  }
}
