package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inch;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.VisionSubsystem.VisionMeasurement;

public class VisionSubsystem extends SubsystemBase {

    /**
     * A simple record to pass vision data back to the drivetrain.
     */
    public record VisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
    }

    // =========================================================================
    // CONSTANTS & CONFIG
    // =========================================================================
    // TODO: Update these names to match the camera names in the PhotonVision UI
    private static final String kLeftCameraName = "FFF";
    private static final String kRightCameraName = "overridename";

    // TODO: Update these transforms! 0, 0, 0 is center of robot
    // X = Forward, Y = Left, Z = Up. Rotation is in Radians.

    // Example: Left camera is 0.2m forward, 0.2m LEFT (positive Y), facing 90 deg
    // to left
    private static final Transform3d kRobotToLeftCamera = new Transform3d(
            new Translation3d(inchToMeter(11), inchToMeter(-11), inchToMeter(16)),
            new Rotation3d(0, Math.toRadians(15), Math.toRadians(90)));

    // Example: Right camera is 0.2m forward, 0.2m RIGHT (negative Y), facing 90 deg
    // to right
    private static final Transform3d kRobotToRightCamera = new Transform3d(
            new Translation3d(
                inchToMeter(-11), inchToMeter(-11), inchToMeter(16)
            ),
            new Rotation3d(0, Math.toRadians(-90), Math.toRadians(-15)));
    // =========================================================================

    private PhotonCamera m_leftCamera;
    private PhotonCamera m_rightCamera;
    private PhotonPoseEstimator m_leftEstimator;
    private PhotonPoseEstimator m_rightEstimator;

    private static double inchToMeter(double inches) {
        return inches * 0.0254;
    }

    public VisionSubsystem() {
        try {
            // Attempt to load the AprilTagFieldLayout (automatically loads current season)
            // Use the specific game field (e.g., k2025Reefscape or k2026...).
            AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(
                    AprilTagFields.k2026RebuiltWelded);

            // --- Left Camera Setup ---
                // m_leftCamera = new PhotonCamera(kLeftCameraName);
                // m_leftEstimator = new PhotonPoseEstimator(
                //         fieldLayout,
                //         PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                //         kRobotToLeftCamera);

            // --- Right Camera Setup ---
            m_rightCamera = new PhotonCamera(kRightCameraName);
            m_rightEstimator = new PhotonPoseEstimator(
                    fieldLayout,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    kRobotToRightCamera);

        } catch (Exception e) {
            DriverStation.reportError("Failed to load AprilTagFieldLayout: " + e.getMessage(), true);
            m_leftEstimator = null;
            m_rightEstimator = null;
        }
    }

    /**
     * Calculates the standard deviations of the vision measurement.
     * Trust vision less (higher std dev) when tags are far away or few are visible.
     */
    private Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose, PhotonPipelineResult result) {
        double avgDist = 0;
        var targets = result.getTargets();

        if (targets.isEmpty()) {
            return VecBuilder.fill(1.0, 1.0, 1.0); // High uncertainty
        }

        // Calculate average distance to all visible tags
        for (var target : targets) {
            avgDist += target.getBestCameraToTarget().getTranslation().getNorm();
        }
        avgDist /= targets.size();

        // Baseline noise
        double xyStdDev = 0.05;
        double thetaStdDev = 0.05;

        // Scale noise by distance (trust closer tags more)
        xyStdDev += avgDist * avgDist * 0.02;
        thetaStdDev += avgDist * avgDist * 0.02;

        // If we see multiple tags, we can trust the estimate much more
        if (targets.size() > 1) {
            xyStdDev /= 2.0;
            thetaStdDev /= 2.0;
        }

        return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
    }

    /**
     * Processes the cameras and returns a list of valid vision measurements.
     * This should be called periodically by the drivetrain.
     * * @return A list of new vision measurements (can be empty).
     */
    public List<VisionMeasurement> getEstimatedGlobalPoses() {
        List<VisionMeasurement> measurements = new ArrayList<>();

        processCamera(m_leftEstimator, m_leftCamera).ifPresent(measurements::add);
        processCamera(m_rightEstimator, m_rightCamera).ifPresent(measurements::add);

        return measurements;
    }

    private Optional<VisionMeasurement> processCamera(PhotonPoseEstimator estimator, PhotonCamera camera) {
        if (estimator == null || camera == null)
            return Optional.empty();

        PhotonPipelineResult result = camera.getLatestResult();
        if (!result.hasTargets())
            return Optional.empty();

        // Use multi-tag if available, fall back to single-tag
        Optional<EstimatedRobotPose> poseEstimate = result.getMultiTagResult().isPresent()
                ? estimator.estimateCoprocMultiTagPose(result)
                : Optional.empty(); // single tag is significantly less accurate, so we ignore it in this example. You can enable it if you want, but be aware of the potential for large errors.
                // estimator.update(result); // falls back to single-tag strategy

        return poseEstimate.map(estimatedRobotPose -> {
            Pose2d estPose = estimatedRobotPose.estimatedPose.toPose2d();
            double timestamp = estimatedRobotPose.timestampSeconds;
            Matrix<N3, N1> deviation = getEstimationStdDevs(estPose, result);
            return new VisionMeasurement(estPose, timestamp, deviation);
        });
    }
}