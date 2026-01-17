// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drivetrain.AlignToHub;
import frc.robot.sub_containers.DriveBaseContainer;
import frc.robot.subsystems.DashboardWriter;

public class RobotContainer {
    // Sub-Containers
    public final DriveBaseContainer driveBaseContainer; // HINT: looking for DriveBase Controls look in here
    // Subsystems
    public final DashboardWriter dashboardWriter = new DashboardWriter();


    // Controllers
    private final CommandXboxController driverController = new CommandXboxController(0);

    public RobotContainer() {
        driveBaseContainer = new DriveBaseContainer(driverController);
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
            driveBaseContainer.driveHider()
        );
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }
}
