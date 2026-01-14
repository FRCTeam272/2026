// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  
  SparkMAXContainer flywheel;
  SparkMAXContainer hood;

  final int FLYWHEEL_LOCATION = 2;
  final int HOOD_LOCATION = 3;

  public Shooter() {
    flywheel = new SparkMAXContainer(FLYWHEEL_LOCATION);
    hood = new SparkMAXContainer(HOOD_LOCATION);  
  }

  public boolean SpinWheel(double speed){
    flywheel.motor.set(speed);
    return true;
  }

  public void AdjustHood(double angle){
    hood.goToPostion(angle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
