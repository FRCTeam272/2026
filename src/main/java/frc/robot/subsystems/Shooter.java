// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utils.SparkMAXContainer;


public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  
  SparkMAXContainer flywheel;
  SparkMAXContainer hood;

  final int FLYWHEEL_LOCATION = 2;
  final int HOOD_LOCATION = 3;

  int speedThreshold = 50;
  int angleThreshold = 2;

  public Shooter() {
    flywheel = new SparkMAXContainer(FLYWHEEL_LOCATION);
    hood = new SparkMAXContainer(HOOD_LOCATION);  
  }

  public boolean SpinWheel(double target_velocity){
    const current_velocity = flywheel.setVelocity(target_velocity);
    return Math.abs(current_velocity - target_velocity) < speedThreshold;
  }

  public boolean AdjustHood(double target_angle){
    return hood.goToPostion(target_angle, angleThreshold);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
