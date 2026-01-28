// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.intake.IntakeDeploy;
import frc.robot.commands.intake.IntakeIntake;
import frc.robot.commands.intake.IntakeRetract;
import frc.robot.commands.intake.IntakeStop;
import frc.robot.sub_containers.DriveBaseContainer;
import frc.robot.subsystems.DashboardWriter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ConveyorAndRegulator;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
    // Sub-Containers
    public final DriveBaseContainer driveBaseContainer; // HINT: looking for DriveBase Controls look in here
    // Subsystems
    public final DashboardWriter dashboardWriter = new DashboardWriter();
    public final Intake intake = new Intake();

    public final Shooter shooter = new Shooter();
    public final ConveyorAndRegulator regulator = new ConveyorAndRegulator();

    // Controllers
    private final CommandXboxController driverController = new CommandXboxController(0);

    public RobotContainer() {
        driveBaseContainer = new DriveBaseContainer(driverController);
        configureBindings();
    }

    private void configureBindings() {
        driverController.a().whileTrue((new IntakeIntake(intake))).onFalse(new IntakeStop(intake));
        // driverController.a().onTrue(new IntakeDeploy(intake));
        // driverController.b().onTrue(new IntakeRetract(intake));
        driverController.rightTrigger().onTrue(new InstantCommand(() -> shooter.SpinWheel(shooter.targetVelocity)));
                // .onFalse(new InstantCommand(() -> shooter.SpinWheel(0)));
        
        driverController.leftTrigger().onTrue(new InstantCommand(() -> {
            shooter.SpinWheel(0);
            regulator.stopConveyor();
            regulator.stopRegulator();
        })).onFalse(new InstantCommand());

        driverController.a().onTrue(new InstantCommand(() -> {
            regulator.conveyorLoad();
            regulator.regulatorLoad();
        }));
        //  driverController.b().onTrue(new InstantCommand(() -> regulator.regulatorLoad()));
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }
}
