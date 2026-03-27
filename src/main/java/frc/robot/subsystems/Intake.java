// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utils.SparkMAXContainer;
import frc.lib.utils.TalonFxContainer;
import frc.lib.utils.TrimPot;


public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  TalonFxContainer rollerMotor;
  TalonFxContainer rollerFollowerMotor;
  SparkMAXContainer deployMotor;
  public final int intake_id = 2;
  public final int intake_follower_id = 60;
  public final int deploy_id = 3;
  public double defult_speed = .85;

  public double deploy_position = 16.5;
  public double retract_position = 0.6;

  // Force Detection Constants
  private final double kImpactCurrentThreshold = 8.0; // Amps (slightly below the 10A hold limit)
  private final double kPositionTolerance = .2;      // Ticks/Degrees
  private final double kDebounceTime = 0.1;           // Seconds (100ms)
  private final Debouncer m_impactDebouncer = new Debouncer(kDebounceTime, DebounceType.kRising);
  // private final Debouncer m_impactDebouncer = new Debouncer(kDebounceTime, Debouncer.Type.kRising);
  private boolean m_impactDetected = false;

  public Intake() {
    rollerMotor= new TalonFxContainer(intake_id, true);
    rollerFollowerMotor = new TalonFxContainer(intake_follower_id, true);

    for (var i : new TalonFxContainer[] {
      rollerMotor, 
      rollerFollowerMotor
    }) {
      i.motor.getVelocity().setUpdateFrequency(20);
      i.assignPIDValues(0.01, 0, 0);
      i.setBreakMode(false);
    }

    rollerFollowerMotor.setupAsFollowerMotor(rollerMotor, true);

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
    // rollerMotor.setVelocity(1000);
    rollerMotor.setVelocity(5100);
    // rollerMotor.motor.set(defult_speed);
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

  public boolean jostle(){
    // return true;
    return deployMotor.goToPostion(8.4);
  }

  public boolean deploy() {
    // deployMotor.motor.set(.1);
    // return true;
    return deployMotor.goToPostion(deploy_position, 0);
  }

  public boolean retract() {
    // return true;
    return deployMotor.goToPostion(retract_position, 0);
  }

  public void setCurrentLimitOfDeployMotor(int limit){
    // deployMotor.setCurrentLimit(limit);
  }

  /**
   * Checks if the intake is currently at the deployed position and
    * experiencing high resistance.
  */
  public boolean isImpactDetected() {
    double current = deployMotor.motor.getOutputCurrent();
    double position = deployMotor.getPosition();
    
    // Logic: If at target (19) AND current is high, we are stalling against something
    boolean isAtTarget = Math.abs(position - deploy_position) < kPositionTolerance;
    boolean isStalling = current >= kImpactCurrentThreshold;

    return m_impactDebouncer.calculate(isAtTarget && isStalling);
  }

  public void stopDeploy(){
    this.deployMotor.motor.set(0);
  }
  
  @Override
  public void periodic() {
    rollerMotor.reportMotor("Intake");
    deployMotor.reportMotor("IntakeDeploy");
    SmartDashboard.putNumber("Intake/Deploy Postion", this.deployMotor.getPosition());

    if(!DriverStation.isFMSAttached()){
      // final double deployP = SmartDashboard.getNumber("ConfigIntake/DeployMotorP", 0.01);
      // final double deployI = SmartDashboard.getNumber("ConfigIntake/DeployMotorI", 0);
      // final double deployD = SmartDashboard.getNumber("ConfigIntake/DeployMotorD", 0);
      // // deployMotor.assignPIDValues(deployP, deployI, deployD);
      // deploy_position = SmartDashboard.getNumber("ConfigIntake/DeployPosition", deploy_position);
      // retract_position = SmartDashboard.getNumber("ConfigIntake/RetractPosition", retract_position);
      // defult_speed = SmartDashboard.getNumber("ConfigIntake/DefaultSpeed", defult_speed);
    }
  }
}
