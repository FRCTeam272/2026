// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utils.SparkMAXContainer;

public class ConveyorAndRegulator extends SubsystemBase {
  /** Creates a new Conveyor. */
  public SparkMAXContainer conveyorMotor;
  public SparkMAXContainer regulatorMotor;

  private double converyorSpeed = -.75;
  private double regulatorSpeed = .99;
  
  public ConveyorAndRegulator() {
    conveyorMotor = new SparkMAXContainer(7);
    conveyorMotor.assignPIDValues(0.001, 0, 0);

    regulatorMotor = new SparkMAXContainer(8);
    regulatorMotor.assignPIDValues(0.001, 0, 0);
  }

  public void conveyorLoad() {
    conveyorMotor.motor.set(converyorSpeed);
  }

  public void conveyorUnload() {
    conveyorMotor.motor.set(-converyorSpeed);
  }

  public void stopConveyor() {
    conveyorMotor.motor.set(0);
  }
  
  public void regulatorLoad() {
    regulatorMotor.motor.set(regulatorSpeed);
  }

  public void regulatorUnload() {
    regulatorMotor.motor.set(-regulatorSpeed);
  }

  public void stopRegulator() {
    regulatorMotor.motor.set(0);
  }

  public void stopAll() {
    stopConveyor();
    stopRegulator();
  }

  public void startAll() {
    conveyorLoad();
    regulatorLoad();
  }

  public void reverseAll() {
    conveyorUnload();
    regulatorUnload();
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
