package frc.robot.sub_containers;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoContainer {
    private SendableChooser<Command> autoChooser;
    private CommandSwerveDrivetrain drivetrain; 
        
    public AutoContainer(CommandSwerveDrivetrain drivetrain){
        this.drivetrain = drivetrain;
        // this.drivetrain.configureAutoBuilder();
        this.configureAutoBindings();
    }

    private void configureAutoBindings() {
        NamedCommands.registerCommand("DeployIntake", Commands.none());
        NamedCommands.registerCommand("StartIntake", Commands.none());
        NamedCommands.registerCommand("SpinFlywheel", Commands.none());
        NamedCommands.registerCommand("Shoot", Commands.none());
        NamedCommands.registerCommand("StopIntake", Commands.none());
        
        
        autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
