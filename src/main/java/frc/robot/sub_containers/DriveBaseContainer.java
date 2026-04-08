package frc.robot.sub_containers;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.RobotContainer;
import frc.robot.Telemetry;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Regulator;

public class DriveBaseContainer {
    public AutoContainer autoContainer;
    public static final double maxSpeedFactor = .25; // 85
    public static final double intakeSpeedFactor = .25; // 32, same as
    public static double speedFactor = maxSpeedFactor;
    public static double rotationFactor = .25; // 85
    
    static {
        // edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Speed Factor", speedFactor);
        // edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Rotation Factor", rotationFactor);
    }

    public static DoubleSupplier MaxSpeed = () -> speedFactor * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static DoubleSupplier MaxAngularRate = () -> RotationsPerSecond.of(rotationFactor).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    // private final Telemetry logger = new Telemetry(MaxSpeed.getAsDouble());
    CommandXboxController joystick;
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public DriveBaseContainer(CommandXboxController driverController, RobotContainer rc) {
        joystick = driverController;
        configureBindings();
        SmartDashboard.putBoolean("DriveBase Running",true);
    
        if(!TunerConstants.isTestBot){
            SmartDashboard.putString("MESSAGE", "we are at autoSetup");
            autoContainer = new AutoContainer(rc, this.drivetrain); 
        }
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive
                    .withVelocityX(-joystick.getLeftY() * MaxSpeed.getAsDouble()) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed.getAsDouble()) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate.getAsDouble()) // Drive counterclockwise with negative X (left)
                    .withDeadband(MaxSpeed.getAsDouble() * 0.1).withRotationalDeadband(MaxAngularRate.getAsDouble() * 0.1) // Add a 10% deadband
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // if(!TunerConstants.isTestBot){
        //     drivetrain.registerTelemetry(logger::telemeterize);
        // }
    }

    public Command GetAutonCommand(){
        if(TunerConstants.isTestBot){
            return Commands.none();
        }
        return this.autoContainer.getAutonomousCommand();
    }

    public Pose2d getPose() {
        return drivetrain.getPose();
    }

    public void resetPose(Pose2d newPose) {
        drivetrain.resetPose(newPose);
    }
}