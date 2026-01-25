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
  
  private double converyorSpeed = -1500;
  private double regulatorSpeed = 1500;
  

  public ConveyorAndRegulator() {
    conveyorMotor = new SparkMAXContainer(CONVEYOR_LOCATION);
    conveyorMotor.assignPIDValues(conveyorPID.kP, conveyorPID.kI, conveyorPID.kD);

    regulatorMotor = new SparkMAXContainer(REGULATOR_LOCATION);
    regulatorMotor.assignPIDValues(regulatorPID.kP, regulatorPID.kI, regulatorPID.kD);  


    SmartDashboard.putNumber("Conveyor/Speed", converyorSpeed);
    SmartDashboard.putNumber("Regulator/Speed", regulatorSpeed);
    SmartDashboard.putNumber("Conveyor/P", conveyorPID.kP);
    SmartDashboard.putNumber("Conveyor/I", conveyorPID.kI);
    SmartDashboard.putNumber("Conveyor/D", conveyorPID.kD);
    SmartDashboard.putNumber("Regulator/P", regulatorPID.kP);
    SmartDashboard.putNumber("Regulator/I", regulatorPID.kI);
    SmartDashboard.putNumber("Regulator/D", regulatorPID.kD);
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

  private void dynamicConveyorPID(double kP, double kI, double kD){
    conveyorMotor.assignPIDValues(kP, kI, kD);
  }

  private void dynamicRegulatorPID(double kP, double kI, double kD){
    regulatorMotor.assignPIDValues(kP, kI, kD);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // if we don't see FMS, allow for dynamic PID tuning
    if(!DriverStation.isFMSAttached()){
      final double p = SmartDashboard.getNumber("Conveyor/P", conveyorPID.kP);
      final double i = SmartDashboard.getNumber("Conveyor/I", conveyorPID.kI);
      final double d = SmartDashboard.getNumber("Conveyor/D", conveyorPID.kD);
      final double rp = SmartDashboard.getNumber("Regulator/P", regulatorPID.kP);
      final double ri = SmartDashboard.getNumber("Regulator/I", regulatorPID.kI);
      final double rd = SmartDashboard.getNumber("Regulator/D", regulatorPID.kD);

      dynamicConveyorPID(p, i, d);
      dynamicRegulatorPID(rp, ri, rd);

      this.converyorSpeed = SmartDashboard.getNumber("Conveyor/Speed", converyorSpeed);
      this.regulatorSpeed = SmartDashboard.getNumber("Regulator/Speed", regulatorSpeed);
    }
    
  }
}