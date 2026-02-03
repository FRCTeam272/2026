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
import frc.robot.commands.shooter.ShooterShoot;
import frc.robot.commands.shooter.ShooterStop;
import frc.robot.sub_containers.AutoContainer;
import frc.robot.sub_containers.DriveBaseContainer;
import frc.robot.subsystems.DashboardWriter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ConveyorAndRegulator;
import frc.robot.subsystems.Shooter;
import frc.tuner.FlyWheelAutoTuner;
import frc.robot.commands.ComplexCommands;
public class RobotContainer {
    // Sub-Containers
    public final DriveBaseContainer driveBaseContainer; // HINT: looking for DriveBase Controls look in here
    // // Subsystems
    public final DashboardWriter dashboardWriter = new DashboardWriter();
    public final Intake intake = new Intake();

    public final Shooter shooter = new Shooter();
    public final ConveyorAndRegulator regulator = new ConveyorAndRegulator();
    public final FlyWheelAutoTuner flyWheelAutoTuner = new FlyWheelAutoTuner();

    // Controllers
    private final CommandXboxController driverController = new CommandXboxController(0);

    public RobotContainer() {
        driveBaseContainer = new DriveBaseContainer(driverController);
        configureBindings();
    }

    private void configureBindings() {
        // driverController.leftTrigger().onTrue(
        //     new AlignToHub(driveBaseContainer.drivetrain, driverController)
        // ).onFalse(
        //     Commands.runOnce(() -> driveBaseContainer.driveHider())
        // );
        driverController.leftTrigger().onTrue(new AlignToHub(driveBaseContainer.drivetrain, driverController)).onFalse(Commands.runOnce(() -> driveBaseContainer.driveHider()));
        // Intake - Driver Controler Left Bumper
        driverController.leftBumper().whileTrue(ComplexCommands.Intake(intake)).onFalse(ComplexCommands.StopIntake(intake));

        // Stop Shooter - Left Bumper
        driverController.rightBumper().onTrue(new ShooterStop(shooter));
        // Shoot - Right Trigger
        driverController.rightTrigger().onTrue(new ShooterShoot(shooter, () -> shooter.targetVelocity));
        // Conveyor and Regulator - Driver A Button
        driverController.a().whileTrue(Commands.run(() -> regulator.startAll(), regulator)).onFalse(Commands.run(() -> regulator.stopAll(), regulator));
        
        // driverController.rightTrigger().onTrue(new InstantCommand(() -> shooter.SpinWheel(shooter.targetVelocity)));
        // driverController.y().onTrue(new InstantCommand(() -> shooter.SpinWheel(0)));
        // driverController.a().whileTrue(new InstantCommand(() -> regulator.startAll())).onFalse(new InstantCommand(() -> regulator.stopAll()));
        // driverController.b().onTrue(new Command() {
        //     @Override
        //     public void initialize() {
        //         super.initialize();
        //         shooter.SpinWheel(0);
        //         regulator.stopAll();
        //         intake.stop();
        //     }
        // });
        // driverController.leftTrigger().whileTrue(new Command() {
        //    @Override
        //    public void initialize() {
        //        super.initialize();
        //        intake.intake();
        //    }
        // }).onFalse(new InstantCommand(() -> intake.stop()));

        // driverController.leftBumper().whileTrue(new InstantCommand(() -> intake.release())).onFalse(new InstantCommand(() -> intake.stop()));
        
    }
    
    public Command getAutonomousCommand() {
        return Commands.none();
    }
}
