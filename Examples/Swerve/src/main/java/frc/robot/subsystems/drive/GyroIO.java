package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
    
    @AutoLog
    public static class GyroIOInputs {
        public GyroIOData data = new GyroIOData(
            false, // connected
            new Rotation2d(0.0), // yaw
            0.0, // yawVelocityRadPerSec
            new Rotation2d(0.0), // pitch
            0.0, // pitchVelocityRadPerSec
            new Rotation2d(0.0), // roll
            0.0 // rollVelocityRadPerSec
        );

        public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    }

    public static record GyroIOData(
        boolean connected,
        Rotation2d yaw,
        double yawVelocityRadPerSec,
        Rotation2d pitch,
        double pitchVelocityRadPerSec,
        Rotation2d roll,
        double rollVelocityRadPerSec
    ) {}
    
    public abstract void updateInputs(GyroIOInputs inputs);

    public abstract Rotation2d getYaw(); // Returns the current yaw immediately

    public abstract void rezero();
}
