// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utils.SparkMAXContainer;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  SparkMAXContainer rollerMotor;
  SparkMAXContainer deployMotor;
  public final int intake_id = 1;
  public final int deploy_id = 2;
  public final double defult_speed = .5;

  public final double deploy_position = 10.0;
  public final double retract_postion = 0; 

  public Intake() {
    rollerMotor= new SparkMAXContainer(intake_id);
    deployMotor = new SparkMAXContainer(deploy_id);
  }
  
  public void intake() {
    rollerMotor.motor.set(defult_speed);
  }

  public void release() {
    rollerMotor.motor.set(-defult_speed);
  }
  
  public void stop() {
    rollerMotor.motor.set(0);
  }

  public void deploy() {
    deployMotor.goToPostion(deploy_position);
  }

  public void retract() {
    deployMotor.goToPostion(retract_postion);
  }
  
  @Override
  public void periodic() {
    rollerMotor.reportMotor("Intake");
  }
}
