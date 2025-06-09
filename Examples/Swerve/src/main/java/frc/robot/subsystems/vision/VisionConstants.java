package frc.robot.subsystems.vision;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import org.photonvision.PhotonPoseEstimator.ConstrainedSolvepnpParams;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.vision.CameraIO.VisionStdDevs;

public class VisionConstants {

    public static final VisionStdDevs kDefaultSDseed = new CameraIO.VisionStdDevs(0.5,1.0,4.0,8.0);

    public static final AprilTagFieldLayout kFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    public static enum PhotonCamConfig {
        PLACEHOLDER_CAM(
            "PlaceholderCam",
            PoseStrategy.CONSTRAINED_SOLVEPNP,
            PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
            kDefaultSDseed,
            0.0, 0.0, 0.0, // x, y, z in inches
            0.0, 0.0, 0.0, // roll, pitch, yaw in degrees
            true, 1.0 // headingFree, headingScaleFactor (Constrained PnP parameters)
        );

        public final String cameraNickname; // This is the nickname used by photonvision to identify the camera
        public final Transform3d robotToCamera;
        public final PoseStrategy primaryPoseStrategy; // The strategy to use when multiple tags are detected
        public final PoseStrategy secondaryPoseStrategy; // The strategy to use when only one tag is detected or if the primary strategy fails
        public final VisionStdDevs SDseed;
        public final ConstrainedSolvepnpParams cpnpParams;
        private PhotonCamConfig(
            String cameraNickname,
            PoseStrategy primaryPoseStrategy, 
            PoseStrategy secondaryPoseStrategy, 
            VisionStdDevs SDseed,
            double xInches, double yInches, double zInches,
            double rollDegrees, double pitchDegrees, double yawDegrees,
            boolean headingFree, double headingScaleFactor
        ) {
            this.cameraNickname = cameraNickname;
            this.primaryPoseStrategy = primaryPoseStrategy;
            this.secondaryPoseStrategy = secondaryPoseStrategy;
            this.SDseed = SDseed;
            this.robotToCamera = new Transform3d(
                Units.inchesToMeters(xInches),
                Units.inchesToMeters(yInches),
                Units.inchesToMeters(zInches),
                new Rotation3d(
                    Units.degreesToRadians(rollDegrees),
                    Units.degreesToRadians(pitchDegrees),
                    Units.degreesToRadians(yawDegrees)
                )
            );
            this.cpnpParams = new ConstrainedSolvepnpParams(headingFree,headingScaleFactor);
        }

        private PhotonCamConfig(
            String cameraNickname, 
            PoseStrategy primaryPoseStrategy, 
            PoseStrategy secondaryPoseStrategy, 
            VisionStdDevs SDseed,
            double xInches, double yInches, double zInches,
            double rollDegrees, double pitchDegrees, double yawDegrees
        ) {
            this(cameraNickname, primaryPoseStrategy, secondaryPoseStrategy, SDseed, xInches, yInches, zInches, rollDegrees, pitchDegrees, yawDegrees, false, 1.0);
        }
    }
}
