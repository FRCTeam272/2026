package frc.robot.sub_containers;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
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
    
    public AutoContainer(RobotContainer rc) {
        // this.drivetrain.configureAutoBuilder();
        this.configureAutoBindings();
        this.intake = rc.intake;
        this.shooter = rc.shooter;
        this.regulator = rc.regulator;
        this.conveyor = rc.conveyor;
        
    }

    private void configureAutoBindings() {
        NamedCommands.registerCommand("DeployIntake", new InstantCommand(() -> {
            intake.setCurrentLimitOfDeployMotor(40);
            intake.jostle();
            intake.deploy();
        }));
        NamedCommands.registerCommand("StartIntake", new InstantCommand(() -> {
            intake.intake();
        }));
        NamedCommands.registerCommand("SpinFlywheel", new InstantCommand(() -> shooter.SpinWheel(shooter.targetVelocity)));
        NamedCommands.registerCommand("Shoot", new InstantCommand(() -> {
            conveyor.Load();    
            regulator.Load();
            intake.setCurrentLimitOfDeployMotor(20);
            intake.retract();
            intake.intake(.85);
        }));
        NamedCommands.registerCommand("StopIntake", new InstantCommand(() -> {
            intake.stop();
        }));
        NamedCommands.registerCommand("DeployAndStartIntake", new InstantCommand(() -> {
            intake.setCurrentLimitOfDeployMotor(40);
            intake.jostle();
            intake.deploy();
            intake.intake();
        }));
        NamedCommands.registerCommand("Kill", new InstantCommand(() -> {
            intake.stop();
            shooter.TrueStop();
            regulator.Stop();
            conveyor.Stop();
        }));

        autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
