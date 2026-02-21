// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utils.SparkMAXContainer;
import frc.lib.utils.TalonFxContainer;
import frc.lib.utils.TrimPot;


public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  TalonFxContainer rollerMotor;
  SparkMAXContainer deployMotor;
  public final int intake_id = 2;
  public final int deploy_id = 3;
  public double defult_speed = .75;

  public double deploy_position = 19;
  public double retract_position = 0;

  public Intake() {
    rollerMotor= new TalonFxContainer(intake_id, true);
    rollerMotor.motor.getVelocity().setUpdateFrequency(20);
    deployMotor = new SparkMAXContainer(deploy_id);
    deployMotor.assignPIDValues(0.1, 0, 0);
    deployMotor.setCurrentLimit(40);
    
    this.setupSmartDashboard();
  }
  
  private void setupSmartDashboard(){
    SmartDashboard.putNumber("ConfigIntake/DeployPosition", deploy_position);
    SmartDashboard.putNumber("ConfigIntake/RetractPosition", retract_position);
    SmartDashboard.putNumber("ConfigIntake/DefaultSpeed", defult_speed);
    SmartDashboard.putNumber("ConfigIntake/DeployMotorP", 0.01);
    SmartDashboard.putNumber("ConfigIntake/DeployMotorI", 0);
    SmartDashboard.putNumber("ConfigIntake/DeployMotorD", 0);
  }

  public void intake() {
    rollerMotor.motor.set(defult_speed);
  }

  public void intake(double speed) {
    rollerMotor.motor.set(speed);
  }

  public void release() {
    rollerMotor.motor.set(-defult_speed);
  }
  
  public void stop() {
    rollerMotor.motor.set(0);
  }

  public boolean deploy() {
    return deployMotor.goToPostion(deploy_position, 0);
  }

  public boolean retract() {
    return deployMotor.goToPostion(retract_position, 0);
  }

  public void setCurrentLimitOfDeployMotor(int limit){
    deployMotor.setCurrentLimit(limit);
  }
  
  @Override
  public void periodic() {
    rollerMotor.reportMotor("Intake");
    deployMotor.reportMotor("IntakeDeploy");

    if(!DriverStation.isDSAttached()){
      final double deployP = SmartDashboard.getNumber("ConfigIntake/DeployMotorP", 0.01);
      final double deployI = SmartDashboard.getNumber("ConfigIntake/DeployMotorI", 0);
      final double deployD = SmartDashboard.getNumber("ConfigIntake/DeployMotorD", 0);
      // deployMotor.assignPIDValues(deployP, deployI, deployD);
      deploy_position = SmartDashboard.getNumber("ConfigIntake/DeployPosition", deploy_position);
      retract_position = SmartDashboard.getNumber("ConfigIntake/RetractPosition", retract_position);
      defult_speed = SmartDashboard.getNumber("ConfigIntake/DefaultSpeed", defult_speed);
    }
  }
}
