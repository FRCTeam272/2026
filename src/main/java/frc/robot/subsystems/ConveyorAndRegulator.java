// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utils.SparkMAXContainer;
import frc.robot.Constants;
import frc.lib.utils.PIDSettings;

public class ConveyorAndRegulator extends SubsystemBase {
  /** Creates a new Conveyor. */
  public SparkMAXContainer conveyorMotor;
  public SparkMAXContainer regulatorMotor;

  public final int CONVEYOR_LOCATION = 7;
  public final int REGULATOR_LOCATION = 8;

  private static PIDSettings conveyorPID = Constants.CONVEYOR_PID_SETTINGS;
  private static PIDSettings regulatorPID = Constants.REGULATOR_PID_SETTINGS;

  // private double converyorSpeed = -.75;
  // private double regulatorSpeed = .99;
  
  private int converyorSpeed = -1500;
  private int regulatorSpeed = 1500;
  

  public ConveyorAndRegulator() {
    conveyorMotor = new SparkMAXContainer(CONVEYOR_LOCATION);
    conveyorMotor.assignPIDValues(conveyorPID.kP, conveyorPID.kI, conveyorPID.kD);

    regulatorMotor = new SparkMAXContainer(REGULATOR_LOCATION);
    regulatorMotor.assignPIDValues(regulatorPID.kP, regulatorPID.kI, regulatorPID.kD);  
  }

  public void conveyorLoad() {
    conveyorMotor.setVelocity(converyorSpeed);
    // conveyorMotor.motor.set(converyorSpeed);
  }

  public void conveyorUnload() {
    conveyorMotor.setVelocity(-converyorSpeed);
    // conveyorMotor.motor.set(-converyorSpeed);
  }

  public void stopConveyor() {
    conveyorMotor.motor.set(0);
  }
  
  public void regulatorLoad() {
    regulatorMotor.setVelocity(regulatorSpeed);
    // regulatorMotor.motor.set(regulatorSpeed);
  }

  public void regulatorUnload() {
    regulatorMotor.setVelocity(-regulatorSpeed);
    // regulatorMotor.motor.set(-regulatorSpeed);
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
