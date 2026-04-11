package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * A command that rotates the robot to face a specific coordinate on the field
 * without moving the robot's translation.
 */
public class AlignToHubPP extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final Translation2d m_targetLocation;

    private DoubleSupplier leftXSupplier = () -> 0.0;
    private DoubleSupplier leftYSupplier = () -> 0.0;

    // Profiled PID Controller for smooth rotation
    private final ProfiledPIDController m_rotationController = new ProfiledPIDController(
        5.0, 0.0, 0.0, // Tune these gains as needed
        new TrapezoidProfile.Constraints(
            Math.PI * 2, // Max angular velocity (rad/s)
            Math.PI * 4  // Max angular acceleration (rad/s^2)
        )
    );

    /**
     * Rotates to face a coordinate.
     * @param drivetrain The drivetrain subsystem.
     * @param targetLocation The X,Y coordinate to look at.
     */
    public AlignToHubPP(CommandSwerveDrivetrain drivetrain, Translation2d targetLocation) {
        m_drivetrain = drivetrain;
        m_targetLocation = targetLocation;

        m_rotationController.enableContinuousInput(-Math.PI, Math.PI);
        m_rotationController.setTolerance(Math.toRadians(1.0));

        addRequirements(m_drivetrain);
    }

    public AlignToHubPP(CommandSwerveDrivetrain drivetrain, Translation2d targetLocation, DoubleSupplier leftXSupplier, DoubleSupplier leftYSupplier) {
        m_drivetrain = drivetrain;
        m_targetLocation = targetLocation;
        this.leftXSupplier = leftXSupplier;
        this.leftYSupplier = leftYSupplier;

        m_rotationController.enableContinuousInput(-Math.PI, Math.PI);
        m_rotationController.setTolerance(Math.toRadians(1.0));

        addRequirements(m_drivetrain);
    }

    public AlignToHubPP(CommandSwerveDrivetrain drivetrain, DoubleSupplier leftXSupplier, DoubleSupplier leftYSupplier) {
        m_drivetrain = drivetrain;
        m_targetLocation = getTargetLocation();
        this.leftXSupplier = leftXSupplier;
        this.leftYSupplier = leftYSupplier;

        m_rotationController.enableContinuousInput(-Math.PI, Math.PI);
        m_rotationController.setTolerance(Math.toRadians(1.0));

        addRequirements(m_drivetrain);
    }

    public AlignToHubPP(CommandSwerveDrivetrain drivetrain) {
        this(drivetrain, getTargetLocation());
    }

    private static Translation2d getTargetLocation() {
        return DriverStation.isDSAttached() && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)
              ? new Translation2d(11.914, 4.032) // if red allaince
              : new Translation2d(4.620, 4.032); // if blue alliance
    }

    @Override
    public void initialize() {
        m_rotationController.reset(m_drivetrain.getState().Pose.getRotation().getRadians());
    }

    @Override
    public void execute() {
        Pose2d currentPose = m_drivetrain.getState().Pose;
        
        // Calculate the angle from robot to target
        double dx = m_targetLocation.getX() - currentPose.getX();
        double dy = m_targetLocation.getY() - currentPose.getY();
        
        // Atan2 gives us the angle to face the point
        Rotation2d targetRotation = new Rotation2d(Math.atan2(dy, dx));

        double rotationSpeed = m_rotationController.calculate(
            currentPose.getRotation().getRadians(),
            targetRotation.getRadians()
        );

        if(m_rotationController.atGoal()){
            rotationSpeed = 0;
        }

        // JAKE I KNOW THIS LOOKS WRONG DON"T TOUCH IT DUMMY
        double leftX = leftXSupplier.getAsDouble();
        double leftY = leftYSupplier.getAsDouble();

        // Drive with supplied X and Y velocities, and calculated rotation
        m_drivetrain.drive(-leftY, -leftX, rotationSpeed, true);
    }

    @Override
    public boolean isFinished() {
        // Ends when the robot is facing the target within tolerance
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain when finished
        m_drivetrain.drive(0, 0, 0, true);
    }
}