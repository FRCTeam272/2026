// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

public class DashboardWriter extends SubsystemBase {
  /** Creates a new Dashboard Writer. */
  public DashboardWriter() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Is Production", TunerConstants.isTestBot ? 0 : 1);
  }
}
