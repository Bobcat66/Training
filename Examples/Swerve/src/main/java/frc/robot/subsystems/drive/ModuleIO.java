package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ModuleIO {

    @AutoLog
    public static class ModuleIOInputs{
        public double drivePositionMeters = 0.0;
        public double driveVelocityMetersPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;

        public Rotation2d steerAbsolutePosition = new Rotation2d();
        public Rotation2d steerPosition = new Rotation2d();
        public double steerVelocityRPM = 0.0;
        public double steerAppliedVolts = 0.0;
        public double steerCurrentAmps = 0.0;

        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionsMeters = new double[] {};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    }

    public abstract void updateInputs(ModuleIOInputs inputs);

    public default void setDriveVolts(double volts) {}

    public default void setSteerVolts(double volts) {}

    public default void setDriveVelocity(double velocityMetersPerSec, double FFVolts) {}

    public default void setSteerPosition(double positionRots, double FFVolts) {}
}