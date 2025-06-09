package frc.robot.subsystems.vision;

import java.util.List;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;

// Generic interface for AprilTag Localization Source IO layers
public interface CameraIO {

    @AutoLog
    public static class CameraIOInputs {
        public CameraIOData data = new CameraIOData(
            false, // connected
            0,
            0.0,
            0.0
        );
    }

    public static record CameraIOData(
        boolean connected,
        int numTags,
        double fps,
        double latencyMs
    ) {}

    public static record AprilTagPoseObservation(
        double timestampSeconds,
        Pose2d pose,
        Vector<N3> stddevs
    ) {}

    public static record VisionStdDevs(
        double transMultiTagStdDev,
        double rotMultiTagStdDev,
        double transSingleTagStdDev,
        double rotSingleTagStdDev
    ) {}

    @SuppressWarnings("unchecked")
    public static Vector<N3>[] getSDVectors(VisionStdDevs StdDevs) {
        var multitagStdDevs = VecBuilder.fill(
            StdDevs.transMultiTagStdDev(),
            StdDevs.transMultiTagStdDev(),
            StdDevs.rotMultiTagStdDev()
        );
        var singletagStdDevs = VecBuilder.fill(
            StdDevs.transSingleTagStdDev(),
            StdDevs.transSingleTagStdDev(),
            StdDevs.rotSingleTagStdDev()
        );
        return (Vector<N3>[]) new Object[]{multitagStdDevs,singletagStdDevs};
    }

    public abstract void updateInputs(CameraIOInputs inputs);

    public abstract String getName();

    public abstract List<AprilTagPoseObservation> getAllUnreadPoseObservations();
}