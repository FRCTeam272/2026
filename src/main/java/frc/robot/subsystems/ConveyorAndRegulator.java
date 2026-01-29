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
  public boolean ConveryorUsePID = false;
  public boolean RegulatorUsePID = false;
  public SparkMAXContainer conveyorMotor;
  public SparkMAXContainer regulatorMotor;

  public final int CONVEYOR_LOCATION = 7;
  public final int REGULATOR_LOCATION = 8;

  private static PIDSettings conveyorPID = Constants.CONVEYOR_PID_SETTINGS;
  private static PIDSettings regulatorPID = Constants.REGULATOR_PID_SETTINGS;
  private double converyorVoltage = -.99;
  private double regulatorVoltage = .99;
  
  private double converyorVelocity = -1500;
  private double regulatorVelocity = 1500;
  

  public ConveyorAndRegulator() {
    conveyorMotor = new SparkMAXContainer(CONVEYOR_LOCATION);
    conveyorMotor.assignPIDValues(conveyorPID.kP, conveyorPID.kI, conveyorPID.kD);

    regulatorMotor = new SparkMAXContainer(REGULATOR_LOCATION);
    regulatorMotor.assignPIDValues(regulatorPID.kP, regulatorPID.kI, regulatorPID.kD);  


    SmartDashboard.putNumber("Conveyor/Velocity", converyorVelocity);
    SmartDashboard.putNumber("Conveyor/Voltage", converyorVoltage);
    SmartDashboard.putNumber("Regulator/Velocity", regulatorVelocity);
    SmartDashboard.putNumber("Regulator/Voltage", regulatorVoltage);

    SmartDashboard.putNumber("Conveyor/P", conveyorPID.kP);
    SmartDashboard.putNumber("Conveyor/I", conveyorPID.kI);
    SmartDashboard.putNumber("Conveyor/D", conveyorPID.kD);
    SmartDashboard.putNumber("Regulator/P", regulatorPID.kP);
    SmartDashboard.putNumber("Regulator/I", regulatorPID.kI);
    SmartDashboard.putNumber("Regulator/D", regulatorPID.kD);

    SmartDashboard.putBoolean("Conveyor/UsePID", ConveryorUsePID);
    SmartDashboard.putBoolean("Regulator/UsePID", RegulatorUsePID);

  }

  public void conveyorLoad() {
    if (ConveryorUsePID) {
      conveyorMotor.setVelocity(converyorVelocity);
    }
     else {
      conveyorMotor.motor.set(converyorVoltage);
    }
  }

  public void conveyorUnload() {
    if (ConveryorUsePID) {
      conveyorMotor.setVelocity(-converyorVelocity);
    } else {
      conveyorMotor.motor.set(-converyorVoltage);
    }
  }

  public void stopConveyor() {
    conveyorMotor.motor.set(0);
  }
  
  public void regulatorLoad() {
    if (RegulatorUsePID) {
      regulatorMotor.setVelocity(regulatorVelocity);
    } else {
      regulatorMotor.motor.set(regulatorVoltage);
    }
  }

  public void regulatorUnload() {
    if (RegulatorUsePID) {
      regulatorMotor.setVelocity(-regulatorVelocity);
    } else {
      regulatorMotor.motor.set(-regulatorVoltage);
    }
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

      this.converyorVelocity = SmartDashboard.getNumber("Conveyor/Velocity", converyorVelocity);
      this.regulatorVelocity = SmartDashboard.getNumber("Regulator/Velocity", regulatorVelocity);
      this.converyorVoltage = SmartDashboard.getNumber("Conveyor/Voltage", converyorVoltage);
      this.regulatorVoltage = SmartDashboard.getNumber("Regulator/Voltage", regulatorVoltage);

      this.ConveryorUsePID = SmartDashboard.getBoolean("Conveyor/UsePID", ConveryorUsePID);
      this.RegulatorUsePID = SmartDashboard.getBoolean("Regulator/UsePID", RegulatorUsePID);

      this.conveyorMotor.getPID("Converyor/PID_Actual/");
      this.regulatorMotor.getPID("Regulator/PID_Actual/");
    }
    
  }
}