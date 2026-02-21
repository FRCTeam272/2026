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
import frc.robot.commands.ClimberCommands;
import frc.robot.sub_containers.AutoContainer;
import frc.robot.sub_containers.DriveBaseContainer;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.DashboardWriter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Regulator;
import frc.robot.subsystems.Shooter;


import frc.lib.controllers.LC_2026Custom;
import frc.robot.commands.TrimPotCommands;

public class RobotContainer {
    // // Subsystems
    public final DashboardWriter dashboardWriter = new DashboardWriter();
    
    // public final Intake intake = new Intake();
    // public final Shooter shooter = new Shooter();
    // public final Regulator regulator = new Regulator();
    // public final Conveyor conveyor = new Conveyor();
    
    public final Climber climber = new Climber();

    // Controllers
    private final CommandXboxController DC = new CommandXboxController(0);
    private final LC_2026Custom OC = new LC_2026Custom(1);

    // Sub-Containers
    // public final DriveBaseContainer driveBaseContainer = new DriveBaseContainer(DC); // HINT: looking for DriveBase
                                                                                     // Controls look in here

    public RobotContainer() {
        // configureBindings();
        // configurTestBindings();
        configureClimberTest();
        /* 
        RobotModeTriggers.disabled().whileTrue(new InstantCommand(() -> {
            TrimPotCommands.SaveTrimPotValues(shooter, intake);
            intake.stop();
            shooter.SpinWheel(0);
            regulator.Stop();
            conveyor.Stop();
            // climber.Stop();
        }, intake, shooter, regulator, conveyor));
        */
    }

    public void configureClimberTest(){
        DC.a().whileTrue(ClimberCommands.Stage1(climber));
        DC.b().whileTrue(ClimberCommands.Stage2(climber));
        DC.x().whileTrue(ClimberCommands.Stage3(climber));
        DC.y().onTrue(ClimberCommands.Dismount(climber));
        
    }   

    /*
    private void configurTestBindings() {
        DC.rightTrigger()
                .whileTrue(new InstantCommand(() -> {
                    conveyor.Load();
                    regulator.Load();
                    intake.setCurrentLimitOfDeployMotor(10);
                    intake.retract();
                    intake.intake(.5);
                }, conveyor))
                .onFalse(new InstantCommand(() -> {
                    conveyor.Stop();
                    regulator.Stop();
                    intake.stop();
                    // []\intake.deploy();
                    intake.setCurrentLimitOfDeployMotor(40);
                }));
        DC.leftTrigger()
                .whileTrue(new InstantCommand(() -> {
                    intake.setCurrentLimitOfDeployMotor(40);
                    intake.deploy();
                    intake.intake();
                    // conveyor.Load(-.2);
                })).onFalse(new InstantCommand(() -> {
                    intake.stop();
                    // intake.retract();
                }));
        DC.y().onTrue(new InstantCommand(() -> shooter.SpinWheel(shooter.targetVelocity)));
        DC.b().onTrue(new InstantCommand(() -> {
            shooter.SpinWheel(0);
            intake.retract();
        }));
        DC.a().onTrue(new InstantCommand(() -> {
            shooter.AdjustHoodIncremental(0.5);
        }));
        DC.x().onTrue(new InstantCommand(() -> {
            shooter.AdjustHoodIncremental(-0.5);
        }));
        DC.rightBumper().whileTrue(new InstantCommand(() -> {
            intake.deploy();
            intake.release();
            conveyor.Load(.3);
            regulator.Load(0.3);
        })).onFalse(new InstantCommand(() -> {
            intake.stop();
            conveyor.Stop();
            regulator.Stop();
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
                new InstantCommand(() -> {
                    shooter.SpinWheel(shooter.targetVelocity);
                }));
        // DC.a().onTrue(new AlignToHub(driveBaseContainer.drivetrain, DC));
        DC.b().onTrue(new InstantCommand(() -> shooter.SpinWheel(0)));

        OC.AgitateButton.whileTrue(IntakeCommands.hopperAgitation(intake));
        OC.ShooterTrimPotUpButton.onTrue(new InstantCommand(() -> shooter.flywheelTrim.adjusterValue += 1));
        OC.ShooterTrimPotDownButton.onTrue(new InstantCommand(() -> shooter.flywheelTrim.adjusterValue -= 1));
        OC.hoodTrimPotUpButton.onTrue(new InstantCommand(() -> shooter.hoodTrim.adjusterValue += 1));
        OC.hoodTrimPotDownButton.onTrue(new InstantCommand(() -> shooter.hoodTrim.adjusterValue -= 1));

        // OC.ClimbTrimPotDownButton.onTrue(new InstantCommand(() ->
        // climber.trim.adjusterValue -= 1));
        // OC.ClimbTrimPotUpButton.onTrue(new InstantCommand(() ->
        // climber.trim.adjusterValue += 1));
        // OC.ClimberRaise.onTrue(new InstantCommand(() -> climber.Raise()));
        // OC.ClimberLower.onTrue(new InstantCommand(() -> climber.Lower()));
        // OC.ClimberZero.onTrue(new InstantCommand(() -> climber.Zero()));
    }
    */
    
    public Command getAutonomousCommand() {
        return Commands.none();
        // return this.driveBaseContainer.GetAutonCommand();
    }
}
