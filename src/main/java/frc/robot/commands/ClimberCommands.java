package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Climber;

public class ClimberCommands {
    //* Raise Up to the point */ 
    public Command Stage1(Climber climber){
        return new InstantCommand(() -> {
            climber.RaiseToPoint();
        });
    }
    //* an additional pop up to make sure the hook is locked in */
    public Command Stage2(Climber climber){
        return new InstantCommand(() -> {
            climber.setValue(-.2);
        }).andThen(new WaitCommand(2));
    }
    //* Bring the Robot up */ 
    public Command Climb(Climber climber){
        return new InstantCommand(() -> {
            climber.LowerToPoint();
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

