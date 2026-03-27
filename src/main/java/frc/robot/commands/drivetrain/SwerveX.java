package frc.robot.commands.drivetrain;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Command to lock the swerve modules into an X-pattern.
 * This makes the robot extremely difficult to push in any direction.
 */
public class SwerveX extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final SwerveRequest.SwerveDriveBrake m_brakeRequest = new SwerveRequest.SwerveDriveBrake();

    /**
     * Creates a new SwerveX command.
     *
     * @param drivetrain The drivetrain subsystem to be locked.
     */
    public SwerveX(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
        addRequirements(m_drivetrain);
    }

    @Override
    public void execute() {
        // Apply the brake request which points all modules toward the center of the robot
        m_drivetrain.setControl(m_brakeRequest);
    }

    @Override
    public void end(boolean interrupted) {
        // No specific exit logic needed as the default command will take over
    }

    @Override
    public boolean isFinished() {
        // This command should run as long as the button is held
        return false;
    }
}