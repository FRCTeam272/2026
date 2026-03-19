package frc.robot.sub_containers;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.ClimberAlign;
import frc.robot.commands.ClimberCommands;
import frc.robot.commands.intake.IntakeCommands;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Regulator;
import frc.robot.subsystems.Shooter;

public class AutoContainer {
    private SendableChooser<Command> autoChooser;
    Intake intake;
    Shooter shooter;
    Regulator regulator;
    Conveyor conveyor;
    Climber climber;
    private CommandSwerveDrivetrain drivetrain;

    public AutoContainer(RobotContainer rc, CommandSwerveDrivetrain drivetrain) {
        // this.drivetrain.configureAutoBuilder();
        this.intake = rc.intake;
        this.shooter = rc.shooter;
        this.regulator = rc.regulator;
        this.conveyor = rc.conveyor;
        this.climber = rc.climber;
        this.drivetrain = drivetrain;

        this.configureAutoBindings();
        SmartDashboard.putNumber("Auto Delay (Seconds)", 0);
    }

    private void configureAutoBindings() {
        NamedCommands.registerCommand("DeployIntake",
                new WaitCommand(.2).andThen(
                        new InstantCommand(() -> {
                            intake.setCurrentLimitOfDeployMotor(40);
                            intake.jostle();
                            intake.deploy();
                        })));
        NamedCommands.registerCommand("StartIntake", new InstantCommand(() -> {
            intake.intake();
        }));
        NamedCommands.registerCommand("SpinFlywheel",
                new InstantCommand(() -> {
                    shooter.SpinWheel(shooter.targetVelocity);
                    intake.stop();
                }));
        NamedCommands.registerCommand("Shoot", new InstantCommand(() -> {
            conveyor.Load();
            regulator.Load();
            intake.setCurrentLimitOfDeployMotor(20);
        }).alongWith(IntakeCommands.hopperAgitation(intake).withTimeout(3.5)));
        NamedCommands.registerCommand("StopIntake", new InstantCommand(() -> {
            intake.stop();
        }));
        NamedCommands.registerCommand("DeployAndStartIntake",
                new WaitCommand(.2).andThen(
                        new InstantCommand(() -> {
                            intake.setCurrentLimitOfDeployMotor(40);
                            intake.jostle();
                            intake.deploy();
                            intake.intake();
                        })));
        NamedCommands.registerCommand("DeployAndStartIntakeDelayed",
                new WaitCommand(1.5).andThen(
                        new InstantCommand(() -> {
                            intake.setCurrentLimitOfDeployMotor(40);
                            intake.jostle();
                            intake.deploy();
                            intake.intake();
                        })));
        NamedCommands.registerCommand("RetractIntake", new InstantCommand(() -> {
            intake.stop();
            intake.retract();
        }));
        NamedCommands.registerCommand("Kill", new InstantCommand(() -> {
            intake.stop();
            shooter.TrueStop();
            regulator.Stop();
            conveyor.Stop();
        }));
        NamedCommands.registerCommand("Climb", new InstantCommand(() -> {
            this.intake.deploy();
        })
                .andThen(ClimberCommands.Stage1(climber))
                .andThen(ClimberCommands.Stage2(climber))
                .andThen(new WaitCommand(3))
                .andThen(ClimberCommands.Stage3(climber)));

        NamedCommands.registerCommand("ClimbPrep",
                ClimberCommands.Stage1(climber).andThen(ClimberCommands.Stage2(climber)));
        NamedCommands.registerCommand("ClimberExecute", ClimberCommands.Stage3(climber));
        NamedCommands.registerCommand("ClimbExecute", ClimberCommands.Stage3(climber));

        NamedCommands.registerCommand("AutoFlywheel", new InstantCommand(() -> {
            shooter.useAutoFlywheel = true;
            shooter.autoFlywheel();
        }));

        // NamedCommands.registerCommand("ClimberAlign", new ClimberAlign(drivetrain, climber));

        autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getAutonomousCommand() {
        var delay = SmartDashboard.getNumber("Auto Delay (Seconds)", 0);
        return new WaitCommand(delay).andThen(autoChooser.getSelected());
    }
}
