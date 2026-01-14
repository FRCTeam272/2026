// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utils.SparkMAXContainer;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  SparkMAXContainer motor;
  public final int intake_id = 1;
  public final double defult_speed = .5;

  public Intake() {
    motor= new SparkMAXContainer(intake_id);
  }
  
  public void intake() {
    motor.motor.set(defult_speed);
  }

  public void release() {
    motor.motor.set(-defult_speed);
  }
  
  public void stop() {
    motor.motor.set(0);
  }
  @Override
  public void periodic() {
    motor.reportMotor("Intake");
  }
}
