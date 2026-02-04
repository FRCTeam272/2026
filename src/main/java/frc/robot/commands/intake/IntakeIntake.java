// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class IntakeIntake extends InstantCommand {
  Intake intake;
  public IntakeIntake(Intake intake) {
    addRequirements(intake);
    this.intake = intake;
  }

  @Override
  public void execute() {
    intake.intake();
  }
}
