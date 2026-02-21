package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Climber;
import frc.robot.commands.climber.*;

public class ClimberCommands {
    //* Raise Up to the point */ 
    public static Command Stage1(Climber climber){  
        return new InstantCommand(() -> {
            var height = SmartDashboard.getNumber("Climber/Stage1", 1);
            SmartDashboard.putNumber("Climber/Executing", 1);
            climber.SetHeight(height);
        });
    }
    //* an additional pop up to make sure the hook is locked in */
    public static Command Stage2(Climber climber){
        return new InstantCommand(() -> {
            var height = SmartDashboard.getNumber("Climber/Stage2", 1);
            SmartDashboard.putNumber("Climber/Executing", 2);
            climber.SetHeight(height);
        }).andThen(new WaitCommand(2));
    }
    //* Bring the Robot up */ 
    public static Command Dismount(Climber climber){
        return new DriveToHeight(climber, 70)
        // .andThen(new WaitCommand(2))
        .andThen(new DriveToHeight(climber, 1));
    }

    public static Command Stage3(Climber climber){
        return new InstantCommand(() -> {
            var height = SmartDashboard.getNumber("Climber/Stage3", 1);
            SmartDashboard.putNumber("Climber/Executing", 3);
            climber.SetHeight(height);
        });
    }

    //* Lower the Robot */ 
    public Command Lower(Climber climber){
        return new InstantCommand(() -> {
            climber.setValue(.3);
        }).andThen(new WaitCommand(1));
    }

    //* Bring the climber all the way down */ 
    public Command ToFloor(Climber climber){
        return new InstantCommand(() -> {
            climber.Zero();
        });
    }
}

