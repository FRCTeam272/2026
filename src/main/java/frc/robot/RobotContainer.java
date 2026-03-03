// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
import frc.lib.utils.PIDSettings;
import frc.robot.commands.TrimPotCommands;

public class RobotContainer {
    // // Subsystems
    public final DashboardWriter dashboardWriter = new DashboardWriter();
    
    public final Intake intake = new Intake();
    public final Shooter shooter = new Shooter();
    public final Regulator regulator = new Regulator();
    public final Conveyor conveyor = new Conveyor();
    
    public final Climber climber = new Climber();

    // Controllers
    private final CommandXboxController DC = new CommandXboxController(0);
    private final LC_2026Custom OC = new LC_2026Custom(1);

    // Sub-Containers
    public final DriveBaseContainer driveBaseContainer = new DriveBaseContainer(DC, this); // HINT: looking for DriveBase Controls look in here

    public RobotContainer() {
        configureBindings();
        // configurTestBindings();
        configureOperatorPanel();
        configureAutonmousBindings(); 
        RobotModeTriggers.disabled().whileTrue(new InstantCommand(() -> {
            TrimPotCommands.SaveTrimPotValues(shooter, intake);
            intake.stop();
            shooter.SpinWheel(0);
            regulator.Stop();
            conveyor.Stop();
            climber.Stop();
        }, intake, shooter, regulator, conveyor));
        
    }

    public void configureAutonmousBindings(){
        new Trigger(intake::isImpactDetected)
           .onTrue(
                new InstantCommand(() -> intake.setCurrentLimitOfDeployMotor(20))
                .andThen(IntakeCommands.retractIntake(intake))
                .andThen(new InstantCommand(() -> intake.setCurrentLimitOfDeployMotor(40)))
            )
           .onTrue(edu.wpi.first.wpilibj2.command.Commands.print("Intake Impact Detected! Retracting..."));
    }

    public void configureOperatorPanel(){
        OC.ClimberStage1.onTrue(ClimberCommands.Stage1(climber));
        OC.ClimberStage2.onTrue(ClimberCommands.Stage2(climber));
        OC.ClimberStage3.onTrue(ClimberCommands.Stage3(climber));
        OC.ClimberRelease.onTrue(ClimberCommands.Dismount(climber));
        
        OC.Stir.whileTrue(IntakeCommands.hopperAgitation(intake));
        OC.CloseUpFlywheel.onTrue(CreateShooterOverride(Constants.SHOOTER_LOW_PID_SETTINGS));
        OC.MeduimFlywheel.onTrue(CreateShooterOverride(Constants.SHOOTER_MID_PID_SETTINGS));
        OC.FarFlywheel.onTrue(CreateShooterOverride(Constants.SHOOTER_HIGH_PID_SETTINGS));
        OC.UseAutoFlywheel.onTrue(new InstantCommand(() -> shooter.useAutoFlywheel = true));
        OC.HoodTrimPotUp.onTrue(TrimPotCommands.AdjustShooterHoodTrim(shooter, .5));
        OC.HoodTrimPotDown.onTrue(TrimPotCommands.AdjustShooterHoodTrim(shooter, -.50));
        OC.ShooterTrimPotUp.onTrue(TrimPotCommands.AdjustShooterVelocityTrim(shooter, 50));
        OC.ShooterTrimPotUp.onTrue(TrimPotCommands.AdjustShooterVelocityTrim(shooter, -50));
        OC.ForceCloseIntake.onTrue(new InstantCommand(() -> {
            intake.setCurrentLimitOfDeployMotor(20);
            intake.retract();
            intake.intake();
        }));
    }


    
    // private void configurTestBindings() {
    //     DC.rightTrigger()
    //             .whileTrue(new InstantCommand(() -> {
    //                 conveyor.Load();
    //                 regulator.Load();
    //                 intake.setCurrentLimitOfDeployMotor(10);
    //                 intake.retract();
    //                 intake.intake(.5);
    //             }, conveyor))
    //             .onFalse(new InstantCommand(() -> {
    //                 conveyor.Stop();
    //                 regulator.Stop();
    //                 intake.stop();
    //                 // []\intake.deploy();
    //                 intake.setCurrentLimitOfDeployMotor(40);
    //             }));
    //     DC.leftTrigger()
    //             .whileTrue(new InstantCommand(() -> {
    //                 intake.setCurrentLimitOfDeployMotor(40);
    //                 intake.jostle();
    //                 intake.deploy();
    //                 intake.intake();
    //                 // conveyor.Load(-.2);
    //             })).onFalse(new InstantCommand(() -> {
    //                 intake.stop();
    //                 // intake.retract();
    //             }));
    //     DC.y().onTrue(new InstantCommand(() -> shooter.SpinWheel(shooter.targetVelocity)));
    //     DC.b().onTrue(new InstantCommand(() -> {
    //         shooter.SpinWheel(0);
    //         intake.retract();
    //     }));
    //     DC.a().onTrue(new InstantCommand(() -> {
    //         shooter.AdjustHoodIncremental(0.5);
    //     }));
    //     DC.x().onTrue(new InstantCommand(() -> {
    //         shooter.AdjustHoodIncremental(-0.5);
    //     }));
    //     DC.rightBumper().whileTrue(new InstantCommand(() -> {
    //         intake.deploy();
    //         intake.release();
    //         conveyor.Load(.3);
    //         regulator.Load(0.3);
    //     })).onFalse(new InstantCommand(() -> {
    //         intake.stop();
    //         conveyor.Stop();
    //         regulator.Stop();
    //     }));
    // }
    
    private void configureBindings() {
        DC.leftTrigger()
                .whileTrue(new InstantCommand(() -> {
                    intake.setCurrentLimitOfDeployMotor(40);
                    intake.jostle();
                    intake.deploy();
                    intake.intake();
                    conveyor.Load(-.2);
                })).onFalse(new InstantCommand(() -> {
                    intake.stop();
                    conveyor.Stop();
                    // intake.retract();
                }));
        DC.y().onTrue(IntakeCommands.retractIntake(intake)).onFalse(new InstantCommand(() -> intake.stop()));
        DC.rightBumper().onTrue(
                new InstantCommand(() -> {
                    shooter.SpinWheel(shooter.targetVelocity);
                }));
        DC.a().onTrue(new AlignToHub(driveBaseContainer.drivetrain, DC));
        DC.b().onTrue(new InstantCommand(() -> shooter.SpinWheel(0)));
        DC.rightTrigger().onTrue(
            new InstantCommand(() -> {
                conveyor.Load();
                regulator.Load();
                intake.setCurrentLimitOfDeployMotor(20);
                intake.retract();
                intake.intake(.85);
            })
        ).onFalse(
            new InstantCommand(() -> {
                conveyor.Stop();
                regulator.Stop();
                intake.stop();
                intake.setCurrentLimitOfDeployMotor(40);
            })
        );
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
            
        });
    }

    public Command getAutonomousCommand() {
        return this.driveBaseContainer.GetAutonCommand();
    }
}
