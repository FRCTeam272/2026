// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.drivetrain.AlignToHub;
import frc.robot.commands.drivetrain.AlignToHubPP;
import frc.robot.commands.drivetrain.SwerveX;
import frc.robot.commands.intake.IntakeCommands;
import frc.robot.commands.intake.IntakeIntake;
import frc.robot.commands.intake.IntakeStop;
import frc.robot.commands.shooter.ShooterShoot;
import frc.robot.commands.shooter.ShooterStop;
import frc.robot.commands.ClimberAlign;
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
import frc.lib.utils.PIDSettings;
import frc.robot.commands.TrimPotCommands;

public class RobotContainer {
    // Controllers
    private final CommandXboxController DC = new CommandXboxController(0);
    private final LC_2026Custom OC = new LC_2026Custom(1);

    // // Subsystems
    public final DashboardWriter dashboardWriter = new DashboardWriter();

    public final Intake intake = new Intake();
    public final Shooter shooter = new Shooter();
    public final Regulator regulator = new Regulator();
    public final Conveyor conveyor = new Conveyor();

    // public final Climber climber = new Climber();
    public PIDSettings lastPidSettings = Constants.SHOOTER_LOW_PID_SETTINGS;

    // Sub-Containers
    public DriveBaseContainer driveBaseContainer = new DriveBaseContainer(DC, this); // HINT: looking for DriveBase
                                                                                     // Controls look in here

    public RobotContainer() {
        configureBindings();
        // configurTestBindings();
        configureOperatorPanel();
        configureAutonmousBindings();

        if (!DriverStation.isFMSAttached()) {
            OC.ClimberStage1.whileTrue(
                    createTestFunction("Deploy Intake", () -> intake.deploy())
                            .andThen(createTestFunction("Intake", () -> intake.intake()))
                            .andThen(createTestFunction("Stop Intake", () -> intake.stop()))
                            .andThen(createTestFunction("Retract Intake", () -> intake.retract()))
                            .andThen(createTestFunction("Hood: 2.5", () -> shooter.AdjustHood(2.5)))
                            .andThen(createTestFunction("Hood: 5", () -> shooter.AdjustHood(5)))
                            .andThen(createTestFunction("Hood: 0", () -> shooter.AdjustHood(0)))
                            .andThen(CreateShooterOverride(Constants.SHOOTER_LOW_PID_SETTINGS))
                            .andThen(createTestFunction("Shooter Override Close",
                                    () -> shooter.SpinWheel(Constants.SHOOTER_LOW_PID_SETTINGS.target_velocity)))
                            .andThen(CreateShooterOverride(Constants.SHOOTER_MID_PID_SETTINGS))
                            .andThen(createTestFunction("Shooter Override Mid",
                                    () -> shooter.SpinWheel(Constants.SHOOTER_MID_PID_SETTINGS.target_velocity)))
                            .andThen(CreateShooterOverride(Constants.SHOOTER_HIGH_PID_SETTINGS))
                            .andThen(createTestFunction("Shooter Override Far",
                                    () -> shooter.SpinWheel(Constants.SHOOTER_HIGH_PID_SETTINGS.target_velocity)))
                            .andThen(createTestFunction("Shooter Stop", () -> shooter.TrueStop()))
                            .andThen(createTestFunction("Regulator Load", () -> regulator.Load()))
                            .andThen(createTestFunction("Conveyor Load", () -> conveyor.Load()))
                            .andThen(createTestFunction("Conveyor Stop", () -> conveyor.Stop())));
        }

        shooter.setDriveBase(driveBaseContainer);
    }

    public Command createTestFunction(String message, Runnable action) {
        return new InstantCommand(() -> {
            SmartDashboard.putString("Test Message", message);
            action.run();
        }).andThen(new WaitCommand(1));
    }

    public void configureAutonmousBindings() {
        new Trigger(intake::isImpactDetected)
                .onTrue(
                        new InstantCommand(() -> intake.setCurrentLimitOfDeployMotor(20))
                                .andThen(IntakeCommands.retractIntake(intake))
                                .andThen(new InstantCommand(() -> intake.setCurrentLimitOfDeployMotor(40))))
                .onTrue(edu.wpi.first.wpilibj2.command.Commands.print("Intake Impact Detected! Retracting..."));

        // new Trigger(driveBaseContainer.drivetrain.m_visionSubsystem::didReseedRecently)
        //         .onTrue(
        //                 new InstantCommand(() -> {
        //                     DC.getHID().setRumble(RumbleType.kBothRumble, 0.3);
        //                 }))
        //         .onFalse(
        //                 new InstantCommand(() -> {
        //                     DC.getHID().setRumble(RumbleType.kBothRumble, 0);
        //                 }));
    }

    public void configureOperatorPanel() {
        // OC.ClimberStage1.onTrue(ClimberCommands.Stage1(climber));
        // OC.ClimberStage2.onTrue(ClimberCommands.Stage2(climber));
        // OC.ClimberStage3.onTrue(ClimberCommands.Stage3(climber));
        // OC.ClimberRelease.onTrue(ClimberCommands.Dismount(climber));
        // toggels photon vision on and off
        OC.ClimberRelease.onTrue(new InstantCommand(() -> {
            this.driveBaseContainer.drivetrain.usePhotonVision = !this.driveBaseContainer.drivetrain.usePhotonVision;
        }));

        OC.Stir.whileTrue(IntakeCommands.hopperAgitation(intake)).onFalse(new InstantCommand(() -> {
            this.intake.retract();
            this.intake.stop();
        }));
        OC.CloseUpFlywheel.onTrue(CreateShooterOverride(Constants.SHOOTER_LOW_PID_SETTINGS));
        OC.MeduimFlywheel.onTrue(CreateShooterOverride(Constants.SHOOTER_MID_PID_SETTINGS));
        OC.FarFlywheel.onTrue(CreateShooterOverride(Constants.SHOOTER_HIGH_PID_SETTINGS));
        OC.UseAutoFlywheel.onTrue(new InstantCommand(() -> shooter.useAutoFlywheel = true));
        OC.HoodTrimPotUp.onTrue(TrimPotCommands.AdjustShooterHoodTrim(shooter, .5));
        OC.HoodTrimPotDown.onTrue(TrimPotCommands.AdjustShooterHoodTrim(shooter, -.50));
        OC.ShooterTrimPotUp.onTrue(TrimPotCommands.AdjustShooterVelocityTrim(shooter, 50));
        OC.ShooterTrimPotDown.onTrue(TrimPotCommands.AdjustShooterVelocityTrim(shooter, -50));
        OC.ForceCloseIntake.onTrue(new InstantCommand(() -> {
            intake.setCurrentLimitOfDeployMotor(20);
            intake.retract();
            intake.intake();
        }));
        OC.BonusButton3.onTrue(new InstantCommand(() -> {
            var pose = DriverStation.isDSAttached() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red
                    ? new Translation2d(13, 4.018)
                    : new Translation2d(3.566, 4.018);
            driveBaseContainer.drivetrain.resetPose(
                    new Pose2d(
                            pose,
                            driveBaseContainer.drivetrain.getPose().getRotation()));
        }));
        // OC.BonusButton2.whileTrue(new InstantCommand(() -> {
        // SmartDashboard.putString("Climb Align", "Starting");
        // }).andThen(new ClimberAlign(driveBaseContainer.drivetrain, climber)));
    }

    // private void configurTestBindings() {
    // DC.rightTrigger()
    // .whileTrue(new InstantCommand(() -> {
    // conveyor.Load();
    // regulator.Load();
    // intake.setCurrentLimitOfDeployMotor(10);
    // intake.retract();
    // intake.intake(.5);
    // }, conveyor))
    // .onFalse(new InstantCommand(() -> {
    // conveyor.Stop();
    // regulator.Stop();
    // intake.stop();
    // // []\intake.deploy();
    // intake.setCurrentLimitOfDeployMotor(40);
    // }));
    // DC.leftTrigger()
    // .whileTrue(new InstantCommand(() -> {
    // intake.setCurrentLimitOfDeployMotor(40);
    // intake.jostle();
    // intake.deploy();
    // intake.intake();
    // // conveyor.Load(-.2);
    // })).onFalse(new InstantCommand(() -> {
    // intake.stop();
    // // intake.retract();
    // }));
    // DC.y().onTrue(new InstantCommand(() ->
    // shooter.SpinWheel(shooter.targetVelocity)));
    // DC.b().onTrue(new InstantCommand(() -> {
    // shooter.SpinWheel(0);
    // intake.retract();
    // }));
    // DC.a().onTrue(new InstantCommand(() -> {
    // shooter.AdjustHoodIncremental(0.5);
    // }));
    // DC.x().onTrue(new InstantCommand(() -> {
    // shooter.AdjustHoodIncremental(-0.5);
    // }));
    // DC.rightBumper().whileTrue(new InstantCommand(() -> {
    // intake.deploy();
    // intake.release();
    // conveyor.Load(.3);
    // regulator.Load(0.3);
    // })).onFalse(new InstantCommand(() -> {
    // intake.stop();
    // conveyor.Stop();
    // regulator.Stop();
    // }));
    // }

    private void configureBindings() {
        DC.leftTrigger()
                .whileTrue(new InstantCommand(() -> {
                    DriveBaseContainer.speedFactor = DriveBaseContainer.intakeSpeedFactor;
                    intake.setCurrentLimitOfDeployMotor(40);
                    intake.intake();
                    conveyor.Load(-3);
                }).alongWith(
                        new InstantCommand(() -> intake.deploy())
                                .andThen(new WaitCommand(2))
                                .andThen(new InstantCommand(() -> intake.stopDeploy()))))
                .onFalse(new InstantCommand(() -> {
                    DriveBaseContainer.speedFactor = DriveBaseContainer.maxSpeedFactor;
                    intake.stop();
                    conveyor.Stop();
                    // intake.retract();
                }));
        DC.leftBumper().onTrue(new InstantCommand(() -> {
            DriveBaseContainer.speedFactor = DriveBaseContainer.maxSpeedFactor;
        }));
        DC.y().onTrue(IntakeCommands.retractIntake(intake)).onFalse(new InstantCommand(() -> intake.stop()));
        DC.rightBumper().onTrue(
                // CreateShooterOverride(lastPidSettings).andThen(
                new InstantCommand(() -> {

                    shooter.autoFlywheel();
                    shooter.forceSync();
                    shooter.SpinWheel(shooter.targetVelocity);
                }));
        DC.a().whileTrue(new AlignToHubPP(driveBaseContainer.drivetrain, () -> DC.getLeftX(), () -> DC.getLeftY()));
        // DC.a().whileTrue(new AlignToHubPP(driveBaseContainer.drivetrain));
        DC.b().onTrue(new InstantCommand(() -> {
            shooter.SpinWheel(0);
            shooter.AdjustHood(0);
            SmartDashboard.putNumber("Hood/Target", 0);

        }));
        DC.rightTrigger().onTrue(
                new InstantCommand(() -> {
                    conveyor.Load();
                    regulator.Load();
                    intake.setCurrentLimitOfDeployMotor(20);
                    intake.retract();
                    intake.intake(.85);
                })).onFalse(
                        new InstantCommand(() -> {
                            conveyor.Stop();
                            regulator.Stop();
                            intake.stop();
                            intake.setCurrentLimitOfDeployMotor(40);
                        }));
        DC.x().whileTrue(new SwerveX(driveBaseContainer.drivetrain));
    }

    public InstantCommand CreateShooterOverride(PIDSettings settings) {
        return new InstantCommand(() -> {
            shooter.useAutoFlywheel = false;
            // shooter.UpdatePID(settings);
            shooter.targetVelocity = settings.target_velocity;
            shooter.targetHood = settings.target_hood;

            SmartDashboard.putNumber("Shooter/P", settings.kP);
            SmartDashboard.putNumber("Shooter/I", settings.kI);
            SmartDashboard.putNumber("Shooter/D", settings.kD);
            SmartDashboard.putNumber("Shooter/kV", settings.kV);
            SmartDashboard.putNumber("Shooter/kA", settings.kA);

            SmartDashboard.putNumber("FlyWheel/TargetVelocity", settings.target_velocity);
            SmartDashboard.putNumber("Hood/Target", settings.target_hood);
            shooter.forceSync();

        });
    }

    public Command getAutonomousCommand() {
        return this.driveBaseContainer.GetAutonCommand();
    }

    public Command getKillAll() {
        return new InstantCommand(() -> {
            TrimPotCommands.SaveTrimPotValues(shooter, intake);
            intake.stop();
            shooter.SpinWheel(0);
            regulator.Stop();
            conveyor.Stop();
            // climber.Stop();
        }, intake, shooter, regulator, conveyor);
    }

    public Subsystem[] getAllSubsystems() {
        return new Subsystem[] { intake, shooter, regulator, conveyor };
    }
}
