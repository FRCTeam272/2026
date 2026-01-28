// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeIntake extends Command {
  /** Creates a new IntakeIntake. */
  Intake intake;
  boolean isDone = false;
  public IntakeIntake(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    this.intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.intake();
    System.out.println("Hey Jake");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    isDone = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
