package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class IntakeCommands {
    public static Command deployIntake(Intake intake) {
        return new InstantCommand() {
            @Override
            public void initialize() {
                intake.setCurrentLimitOfDeployMotor(40);
            }
        }
        .andThen(new InstantCommand(){
            @Override
            public void initialize() {
                intake.deploy();
            }
        }).withTimeout(2)
        .andThen(new InstantCommand() {
            @Override
            public void initialize() {
                intake.setCurrentLimitOfDeployMotor(10);
            }
        });
    }

    public static Command retractIntake(Intake intake) {
        return new InstantCommand() {
            @Override
            public void initialize() {
                intake.setCurrentLimitOfDeployMotor(40);
            }
        }
        .andThen(new InstantCommand(){
            @Override
            public void initialize() {
                intake.retract();
            }
        });
    }
}
