package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.intake.IntakeCommands;
import frc.robot.commands.intake.IntakeIntake;
import frc.robot.commands.intake.IntakeStop;
import frc.robot.subsystems.*;

public class ComplexCommands {
    public static Command Intake(Intake intake) {
        return IntakeCommands.deployIntake(intake).andThen(new IntakeIntake(intake));
    }
    
    public static Command StopIntake(Intake intake) {
        return new IntakeStop(intake).andThen(IntakeCommands.retractIntake(intake));
    }
}
