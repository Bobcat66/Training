package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.vision.CameraIO.AprilTagPoseObservation;
import frc.robot.subsystems.vision.VisionConstants.PhotonCamConfig;
import frc.robot.utils.VirtualSubsystem;

public class Vision extends VirtualSubsystem {

    private final List<Camera> cameras = new ArrayList<>();
    private final List<EstimateConsumer> consumers = new ArrayList<>();
    private final Supplier<Rotation2d> headingSupplier;

    public Vision(Supplier<Rotation2d> headingSupplier) {
        this.headingSupplier = headingSupplier;
    }

    public void addConsumer(EstimateConsumer consumer) {
        consumers.add(consumer);
    }

    public void addPhotonSource(PhotonCamConfig config) {
        CameraIO io = buildPhotonCameraIO(config);
        addCamera(io);
    }

    @Override
    public void periodic() {
        List<AprilTagPoseObservation> poseObservations = new ArrayList<>();
        for (int i = 0; i < cameras.size(); i++) {
            var camera = cameras.get(i);
            camera.periodic();
            Logger.processInputs("Vision/Camera" + i, camera.getInputs());
            poseObservations.addAll(camera.getPoseObservations());
        }
        for (EstimateConsumer consumer : consumers){
            for (var observation : poseObservations){
                consumer.accept(observation.pose(),observation.timestampSeconds(),observation.stddevs());
            }
        }
    }

    public Camera getCamera(int id) {
        return cameras.get(id);
    }

    private CameraIOPhoton buildPhotonCameraIO(PhotonCamConfig config) {
        return new CameraIOPhoton(config, headingSupplier);
    }

    private void addCamera(CameraIO io) {
        cameras.add(new Camera(io));
    }

    private List<AprilTagPoseObservation> getAllPoseObservations() {
        List<AprilTagPoseObservation> combinedObservations = new ArrayList<>();
        for (Camera camera : cameras) {
            combinedObservations.addAll(camera.getPoseObservations());
        }
        return combinedObservations;
    }

    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestampSeconds, Vector<N3> estimationStdDevs);
    }
}
