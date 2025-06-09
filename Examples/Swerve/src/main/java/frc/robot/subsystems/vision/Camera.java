package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class Camera {
    private final CameraIO io;
    private final CameraIOInputsAutoLogged inputs = new CameraIOInputsAutoLogged();
    private List<CameraIO.AprilTagPoseObservation> poseObservationCache = new ArrayList<>();

    public Camera(CameraIO io) {
        this.io = io;
    }

    public void periodic() {
        poseObservationCache = io.getAllUnreadPoseObservations();
        io.updateInputs(inputs);
    }

    CameraIOInputsAutoLogged getInputs() {
        return inputs;
    }

    public List<CameraIO.AprilTagPoseObservation> getPoseObservations() {
        return Collections.unmodifiableList(poseObservationCache);
    }
}
