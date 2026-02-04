// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeRelease extends InstantCommand {
  Intake intake;
  public IntakeRelease(Intake intake) {
    addRequirements(intake);
    this.intake = intake;
  }

  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.release();
  }
}
