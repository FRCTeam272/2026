// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimberAlign extends Command {
  /** Creates a new ClimberAlign. */
  private final CommandSwerveDrivetrain drivetrain;
  private final Climber climber;
  private boolean isFinished = false;
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();

  public ClimberAlign(CommandSwerveDrivetrain drivetrain, Climber climber) {
    this.drivetrain = drivetrain;
    this.climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var sensorValue = climber.isSensorTripped();
    if(sensorValue == 404){
      isFinished = true;
      return;
    }
    final double yVelocity = sensorValue == 1 ? 0.1 : (sensorValue == -1 ? -0.1 : 0.0);

    drivetrain.applyRequest(() -> drive
        .withVelocityX(0)
        .withVelocityY(yVelocity)
        .withRotationalRate(0)
        .withDeadband(0).withRotationalDeadband(0)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
    );

    isFinished = sensorValue == 0;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
