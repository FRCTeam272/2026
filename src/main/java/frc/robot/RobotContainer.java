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
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.drivetrain.AlignToHub;
import frc.robot.commands.intake.IntakeCommands;
import frc.robot.commands.intake.IntakeIntake;
import frc.robot.commands.intake.IntakeStop;
import frc.robot.commands.shooter.ShooterShoot;
import frc.robot.commands.shooter.ShooterStop;
import frc.robot.sub_containers.AutoContainer;
import frc.robot.sub_containers.DriveBaseContainer;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.DashboardWriter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Regulator;
import frc.robot.subsystems.Shooter;
import frc.tuner.FlyWheelAutoTuner;
import frc.lib.controllers.LC_2026Custom;
import frc.lib.utils.TrimPot;
import frc.robot.commands.ComplexCommands;
import frc.robot.commands.TrimPotCommands;

public class RobotContainer {
    // Sub-Containers
    public final DriveBaseContainer driveBaseContainer; // HINT: looking for DriveBase Controls look in here
    // // Subsystems
    public final DashboardWriter dashboardWriter = new DashboardWriter();
    public final Intake intake = new Intake();

    public final Shooter shooter = new Shooter();
    public final Regulator regulator = new Regulator();
    public final Conveyor conveyor = new Conveyor();

    // Controllers
    private final CommandXboxController DC = new CommandXboxController(0);
    private final LC_2026Custom OC = new LC_2026Custom(1);
    public RobotContainer() {
        driveBaseContainer = new DriveBaseContainer(DC);
        // configureBindings();
        configurTestBindings();

        RobotModeTriggers.disabled().onTrue(new InstantCommand(() -> {
            intake.stop();
            shooter.SpinWheel(0);
            regulator.Stop();
            conveyor.Stop();

        }).andThen(TrimPotCommands.SaveTrimPotValues(shooter, intake)));
    }

    private void configurTestBindings() {
        DC.rightTrigger()
                .whileTrue(new InstantCommand(() -> {
                    conveyor.Load();
                    regulator.Load();
                }, conveyor))
                .onFalse(new InstantCommand(() -> {
                    conveyor.Stop();
                    regulator.Stop();
                }));
        DC.leftTrigger()
                .whileTrue(new InstantCommand(() -> {
                    intake.deploy();
                    // intake.intake();
                })).onFalse(new InstantCommand(() -> {
                    // intake.stop();
                    intake.retract();
                }));
        DC.y().onTrue(new InstantCommand(() -> shooter.SpinWheel(shooter.targetVelocity)));
        DC.b().onTrue(new InstantCommand(() -> shooter.SpinWheel(0)));
        DC.a().onTrue(new InstantCommand(() -> {
            shooter.AdjustHood(10);
        }));
        DC.x().onTrue(new InstantCommand(() -> {
            shooter.AdjustHood(0);
        }));
    }

    private void configureBindings() {
        DC.leftTrigger()
                .whileTrue(IntakeCommands.deployIntake(intake)).onFalse(new InstantCommand(() -> {
                    intake.stop();
                }));
        DC.y().onTrue(IntakeCommands.retractIntake(intake));
        DC.rightBumper().onTrue(
            // @TODO: add auto update to shooter velocity targets
            new InstantCommand(() -> {shooter.SpinWheel(shooter.targetVelocity);})
        );

        OC.AgitateButton.whileTrue(IntakeCommands.hopperAgitation(intake));
        OC.ShooterTrimPotUpButton.onTrue(new InstantCommand(() -> shooter.flywheelTrim.adjusterValue += 1));
        OC.ShooterTrimPotDownButton.onTrue(new InstantCommand(() -> shooter.flywheelTrim.adjusterValue -= 1));
        OC.IntakeTrimPotUpButton.onTrue(new InstantCommand(() -> intake.deployTrim.adjusterValue += 1));
        OC.IntakeTrimPotDownButton.onTrue(new InstantCommand(() -> intake.deployTrim.adjusterValue -= 1));
        OC.hoodTrimPotUpButton.onTrue(new InstantCommand(() -> shooter.hoodTrim.adjusterValue += 1));
        OC.hoodTrimPotDownButton.onTrue(new InstantCommand(() -> shooter.hoodTrim.adjusterValue -= 1));
        
    }

    public Command getAutonomousCommand() {
        return Commands.none();
        // return this.driveBaseContainer.GetAutonCommand();
    }
}
