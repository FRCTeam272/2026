// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utils.SparkMAXContainer;

public class Climber extends SubsystemBase {
  SparkMAXContainer motor;
  int port = 3;
  double speedup = 0.5;
  double speeddown = -0.5;
  /** Creates a new Climber. */
  public Climber() {
    motor = new SparkMAXContainer(port);
  }
  @Override
  public void periodic() {
    motor.reportMotor("Climber");
    // This method will be called once per scheduler run
  }

  public void climbup() {
    motor.motor.set(speedup);
  }

  public void climbdown() {
    motor.motor.set(speeddown);
  }

  
}
