package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ConveyorAndRegulator;
import frc.robot.subsystems.Shooter;

public class ComplexCommandsContainer {
    public static Command Shoot(Shooter shooter, ConveyorAndRegulator regulator){
        return new InstantCommand(() -> shooter.SpinWheel(shooter.targetVelocity))
            .alongWith(new InstantCommand(() -> regulator.conveyorLoad()));
    }
}
