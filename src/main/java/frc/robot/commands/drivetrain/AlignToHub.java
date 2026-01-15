package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import java.util.function.DoubleSupplier;

public class AlignToHub extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    
    // Coordinates of the Hub (Update these based on the specific game year/field)
    // Example: 2022 Hub was roughly at (8.22, 4.11)
    private static final Translation2d kHubCenter = new Translation2d(8.22, 4.11); 

    // Rotation PID Controller
    // Tune these values: kP, kI, kD
    private final ProfiledPIDController m_rotationController = 
        new ProfiledPIDController(4.0, 0.0, 0.0, 
        new TrapezoidProfile.Constraints(
            Math.PI * 4, // Max velocity (rad/s)
            Math.PI * 8  // Max acceleration (rad/s^2)
        ));

    // The request to apply to the swerve
    private final SwerveRequest.FieldCentricFacingAngle m_driveRequest = 
        new SwerveRequest.FieldCentricFacingAngle();

    /**
     * Creates a new AlignToHub command.
     *
     * @param drivetrain The drivetrain subsystem
     * @param translationXSupplier Supplier for x translation (joystick input)
     * @param translationYSupplier Supplier for y translation (joystick input)
     */
    public AlignToHub(
        CommandSwerveDrivetrain drivetrain, 
        DoubleSupplier translationXSupplier, 
        DoubleSupplier translationYSupplier
    ) {
        m_drivetrain = drivetrain;
        m_translationXSupplier = translationXSupplier;
        m_translationYSupplier = translationYSupplier;

        // Ensure the controller handles continuous rotation (e.g., -PI to PI)
        m_rotationController.enableContinuousInput(-Math.PI, Math.PI);
        
        // Tolerance for when we consider ourselves "aligned"
        m_rotationController.setTolerance(Math.toRadians(1.0));

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        // 1. Get current robot pose
        Pose2d currentPose = m_drivetrain.getState().Pose;

        // 2. Calculate the angle to the hub
        // Math.atan2(dy, dx) gives the angle from the robot to the target
        double dx = kHubCenter.getX() - currentPose.getX();
        double dy = kHubCenter.getY() - currentPose.getY();
        Rotation2d targetAngle = new Rotation2d(Math.atan2(dy, dx));

        m_drivetrain.setControl(
            m_driveRequest
                .withVelocityX(m_translationXSupplier.getAsDouble())
                .withVelocityY(m_translationYSupplier.getAsDouble())
                .withTargetDirection(targetAngle)
        );
    }

    @Override
    public boolean isFinished() {
        // command will exit on either driver releasing button or 
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake());
    }
}