package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import java.util.Queue;
import java.util.ArrayDeque;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import frc.robot.subsystems.vision.VisionConstants.PhotonCamConfig;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.wpilibj.Timer;

public class CameraIOPhoton implements CameraIO {

    private static final Vector<N3> maxStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);

    private final String sourceName;
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    private final Supplier<Rotation2d> headingSupplier;
    private final Vector<N3> defaultSingleTagStdDevs;
    private final Vector<N3> defaultMultiTagStdDevs;
    private final Optional<Matrix<N3, N3>> cameraMatrix;
    private final Optional<Vector<N8>> distCoeffs;
    private PhotonPipelineResult latestResult = new PhotonPipelineResult();
    private final Queue<Double> captureTimestamps = new ArrayDeque<>(20);

    public CameraIOPhoton(PhotonCamConfig config, Supplier<Rotation2d> headingSupplier) {
        sourceName = config.sourceName;
        camera = new PhotonCamera(config.photonNickname);
        poseEstimator = new PhotonPoseEstimator(
            VisionConstants.kFieldLayout,
            config.primaryPoseStrategy,
            config.robotToCamera
        );
        poseEstimator.setMultiTagFallbackStrategy(config.secondaryPoseStrategy);
        var SDVectorArray = CameraIO.getSDVectors(config.SDseed);
        defaultMultiTagStdDevs = SDVectorArray[0];
        defaultSingleTagStdDevs = SDVectorArray[1];
        cameraMatrix = camera.getCameraMatrix();
        distCoeffs = camera.getDistCoeffs().map(Vector<N8>::new);
        this.headingSupplier = headingSupplier;
    }

    private Vector<N3> calculateStdDevs(EstimatedRobotPose est) {
        var stdDevs = defaultSingleTagStdDevs;
        int numTargets = 0;
        double avgDist = 0.0;
        var targets = est.targetsUsed;
        for (var tgt : targets) {
            var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) {continue;}
            numTargets++;
            avgDist +=
                tagPose
                    .get()
                    .toPose2d()
                    .getTranslation()
                    .getDistance(est.estimatedPose.toPose2d().getTranslation());
        }
        if (numTargets == 0) {
            return maxStdDevs; //No targets detected, resort to maximum std devs
        }

        // One or more tags visible, run the full heuristic.

        // Decrease std devs if multiple targets are visible
        avgDist /= numTargets;
        if (numTargets > 1) {
            stdDevs = defaultMultiTagStdDevs;
        }

        // Increase std devs based on (average) distance
        if (numTargets == 1 && avgDist > 4){
            //Distance greater than 4 meters, and only one tag detected, resort to maximum std devs
            stdDevs = maxStdDevs;
        } else {
            stdDevs = stdDevs.times(1 + (avgDist * avgDist / 30));
        }
        return stdDevs;
    }

    private AprilTagPoseObservation buildPoseObservation(EstimatedRobotPose est) {
        var stdDevs = calculateStdDevs(est);
        return new AprilTagPoseObservation(
            est.timestampSeconds, // Convert to milliseconds
            est.estimatedPose.toPose2d(),
            stdDevs
        );
    }


    @Override
    public void updateInputs(CameraIOInputs inputs) {
        var it = captureTimestamps.iterator();
        int tDeltaCount = 0;
        double cumulativetDelta = 0.0;
        double current;
        double next;
        while (true) {
            current = it.next();
            if (!it.hasNext()) {
                break;
            }
            next = it.next();
            cumulativetDelta += (next - current);
            tDeltaCount++;
        }
        double avgFPS = tDeltaCount > 0 ? (cumulativetDelta / tDeltaCount) : 0.0;
        inputs.data = new CameraIOData(
            camera.isConnected(),
            latestResult.getTargets().size(),
            avgFPS,
            latestResult.metadata.getLatencyMillis()
        );
    }

    @Override
    public List<AprilTagPoseObservation> getAllUnreadPoseObservations() {
        poseEstimator.addHeadingData(Timer.getFPGATimestamp(), headingSupplier.get());
        List<PhotonPipelineResult> pipelineResults = camera.getAllUnreadResults();
        List<AprilTagPoseObservation> observations = new ArrayList<>();
        
        for (var res : pipelineResults) {
            captureTimestamps.offer(res.getTimestampSeconds());
            latestResult = res; // Update the latest result
            if (res.hasTargets()) {
                var observation = poseEstimator
                    .update(res)
                    .map((est) -> {
                        var stdDevs = calculateStdDevs(est);
                        return new AprilTagPoseObservation(
                            est.timestampSeconds,
                            est.estimatedPose.toPose2d(),
                            stdDevs
                        );
                    })
                    .orElse(null);
                if (observation != null) {
                    observations.add(observation);
                }
            }
        }

        return observations;
        
    }

    @Override
    public String getName(){
        return camera.getName();
    }

    


}
