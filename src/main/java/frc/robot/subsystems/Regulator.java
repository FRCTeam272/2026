// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utils.SparkMAXContainer;

public class Regulator extends SubsystemBase {
  /** Creates a new Regulator. */
  SparkMAXContainer motor;
  public final int regulator_id=1;
  public final double default_speed= .5;

  public Regulator() {
    motor= new SparkMAXContainer(regulator_id);
  }

  public void Open() {
    motor.motor.set(default_speed);
  }

  @Override
  public void periodic() {
    motor.reportMotor("Regulator");
  }
}
