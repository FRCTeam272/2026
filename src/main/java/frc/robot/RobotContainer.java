// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drivetrain.AlignToHub;
import frc.robot.commands.intake.IntakeIntake;
import frc.robot.commands.intake.IntakeStop;
import frc.robot.sub_containers.AutoContainer;
import frc.robot.sub_containers.DriveBaseContainer;
import frc.robot.subsystems.DashboardWriter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ConveyorAndRegulator;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
    // Sub-Containers
    public final DriveBaseContainer driveBaseContainer; // HINT: looking for DriveBase Controls look in here
    public final AutoContainer autoContainer;
    // Subsystems
    public final DashboardWriter dashboardWriter = new DashboardWriter();
    public final Intake intake = new Intake();

    public final Shooter shooter = new Shooter();
    public final ConveyorAndRegulator regulator = new ConveyorAndRegulator();

    // Controllers
    private final CommandXboxController driverController = new CommandXboxController(0);

    public RobotContainer() {
        driveBaseContainer = new DriveBaseContainer(driverController);
        autoContainer = new AutoContainer(); // this will need to add subsystems (all of them pretty much)
        driveBaseContainer.drivetrain.resetPose(
            new Pose2d(
                new Translation2d(2.299, 3.913),
                new Rotation2d()
            )
        );
        configureBindings();
    }

    private void configureBindings() {
        driverController.leftTrigger().onTrue(
            new AlignToHub(driveBaseContainer.drivetrain, driverController)
        ).onFalse(
            Commands.runOnce(() -> driveBaseContainer.driveHider())
        );

        driverController.a().whileTrue(new IntakeIntake(intake)).onFalse(new IntakeStop(intake));
    }
    
    public Command getAutonomousCommand() {
        return Commands.none();
    }
}
