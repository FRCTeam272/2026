package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;

public class IntakeCommands {
    public static Command deployIntake(Intake intake) {
        return new InstantCommand(() -> intake.setCurrentLimitOfDeployMotor(40)) // set current limit to 40
        .andThen(new IntakeDeploy(intake)).withTimeout(2) // deploy intake, max 2 seconds
        .andThen(new InstantCommand(() -> intake.setCurrentLimitOfDeployMotor(10))); // set current limit to 10
    }

    public static Command retractIntake(Intake intake) {
        return new InstantCommand(() -> intake.setCurrentLimitOfDeployMotor(40)) // set current limit to 40
        .andThen(new InstantCommand(() -> intake.retract())); // retract intake
    }

    public static Command hopperAgitation(Intake intake) {
        return // Cycle between retract and deploy every 0.5 seconds
            new InstantCommand(() -> intake.retract()) // retract intake
                .andThen(new WaitCommand(0.5)) // wait 0.5 seconds
                .andThen(new InstantCommand(() -> intake.deploy())) // deploy intake
                .andThen(new WaitCommand(0.5)) // wait 0.5 seconds
                .repeatedly();
    }
}
