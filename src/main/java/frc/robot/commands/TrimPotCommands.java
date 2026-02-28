package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class TrimPotCommands {
    /**
     * Writes out the current trimpot values so they can be loaded on next boot
     * 
     * @param shooter
     * @param intake
     * @return
     */
    public static Command SaveTrimPotValues(Shooter shooter, Intake intake) {
        return new InstantCommand(() -> {
            shooter.hoodTrim.saveAdjusterValue();
            shooter.flywheelTrim.saveAdjusterValue();
            
        });
    }

    public static Command SaveTrimPotValues(Shooter shooter, Intake intake, Climber climber) {
        return new InstantCommand(() -> {
            shooter.hoodTrim.saveAdjusterValue();
            shooter.flywheelTrim.saveAdjusterValue();
            climber.trim.saveAdjusterValue();
        });
    }

    /**
     * Increases the hood trim by the given value. This will adjust the target angle of the hood by the given value. This is used to fine tune the hood angle without having to change the target angle in the code.
     * @param shooter
     * @param value
     * @return
     */
    public static Command AdjustShooterHoodTrim(Shooter shooter, double value) {
        return new InstantCommand(() -> shooter.hoodTrim.adjusterValue += value);
    }
    /**
     * Increases the flywheel trim by the given value. This will adjust the target velocity of the flywheel by the given value. This is used to fine tune the flywheel velocity without having to change the target velocity in the code.
     * @param shooter
     * @param value
     * @return
     */
    public static Command AdjustShooterVelocityTrim(Shooter shooter, double value) {
        return new InstantCommand(() -> shooter.flywheelTrim.adjusterValue += value);
    }
}
