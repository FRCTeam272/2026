// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.tuner;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utils.TalonFxContainer;

public class FlyWheelAutoTuner extends SubsystemBase {
  double tartgetRPM = 3000;

  double kP = 0.0;
  double KI = 0.0;
  double kD = 0.0;
  double kV = 0.0;

  TalonFxContainer flywheel;

  /** Creates a new FlyWheelAutoTuner. */
  public FlyWheelAutoTuner() {
    flywheel = new TalonFxContainer(4, true);
    flywheel.setBreakMode(false);
    flywheel.configurator.Audio.BeepOnConfig = false;
    flywheel.applyConfig();
    
    SmartDashboard.putNumber("Test/CurrentRPM", flywheel.getVelocity());
    SmartDashboard.putNumber("Test/TargetRPM", tartgetRPM);
    // get Real PID and kV values
    flywheel.getPID("Test/");
  }


  long timer = 0;
  @Override
  public void periodic() {
    if(flywheel.getVelocity() >= tartgetRPM - 50){
      new Alert("Reached Target RPM", AlertType.kInfo).close();;
      // flywheel.setVelocity(0);
    }

    // delay 2 seconds between adjustments
    if (System.currentTimeMillis() - timer <= 2000) {
      return;
    }

    SmartDashboard.putNumber("Test/CurrentRPM", flywheel.getVelocity());
    SmartDashboard.putNumber("Test/TargetRPM", tartgetRPM);
    // get Real PID and kV values
    flywheel.getPID("Test/");
    
    if (DriverStation.isDisabled()) {
      flywheel.setVelocity(0);
      timer = System.currentTimeMillis();
    }

    if(flywheel.getVelocity() < tartgetRPM - 50){
      kV += 0.00001;
      flywheel.assignFF(kP, kV, kD, KI);
      flywheel.setVelocity(tartgetRPM);
      timer = System.currentTimeMillis();
    }
    
    if(flywheel.getVelocity() < tartgetRPM - 50){
      kV -= 0.00001;
      flywheel.assignFF(kP, kV, kD, KI);
      flywheel.setVelocity(tartgetRPM);
      timer = System.currentTimeMillis();
    }
  }
}
