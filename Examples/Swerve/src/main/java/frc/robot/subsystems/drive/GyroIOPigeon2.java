package frc.robot.subsystems.drive;

import java.util.concurrent.ConcurrentLinkedQueue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import static frc.robot.subsystems.drive.DriveConstants.kGyroPort;

public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 m_pigeon = new Pigeon2(kGyroPort);
    private final StatusSignal<Angle> yaw = m_pigeon.getYaw();
    private final StatusSignal<Angle> pitch = m_pigeon.getPitch();
    private final StatusSignal<Angle> roll = m_pigeon.getRoll();
    private final StatusSignal<AngularVelocity> yawVelocity = m_pigeon.getAngularVelocityZWorld();
    private final StatusSignal<AngularVelocity> pitchVelocity = m_pigeon.getAngularVelocityXWorld();
    private final StatusSignal<AngularVelocity> rollVelocity = m_pigeon.getAngularVelocityYWorld();
    private final ConcurrentLinkedQueue<Double> odometryYawPositionsRadians;

    public GyroIOPigeon2() {
        m_pigeon.getConfigurator().apply(new Pigeon2Configuration());
        m_pigeon.getConfigurator().setYaw(0.0);
        yaw.setUpdateFrequency(DriveConstants.kOdometryFrequencyHz);
        BaseStatusSignal.setUpdateFrequencyForAll(50, pitch, roll, yawVelocity, pitchVelocity, rollVelocity);
        m_pigeon.optimizeBusUtilization();
        odometryYawPositionsRadians = OdometryThread.getInstance().registerSignal(() -> Units.degreesToRadians(yaw.getValueAsDouble()));
    }

    @Override
    public void rezero() {
        // Reset the Pigeon2's yaw to 0 degrees
        m_pigeon.getConfigurator().setYaw(0.0);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.data = new GyroIOData(
            m_pigeon.isConnected(),
            new Rotation2d(yaw.getValue()),
            yawVelocity.getValue().in(RadiansPerSecond),
            new Rotation2d(pitch.getValue()),
            pitchVelocity.getValue().in(RadiansPerSecond),
            new Rotation2d(roll.getValue()),
            rollVelocity.getValue().in(RadiansPerSecond)
        );

        inputs.odometryYawPositions = odometryYawPositionsRadians.stream()
            .map(Rotation2d::fromRadians)
            .toArray(Rotation2d[]::new);
    }

    @Override
    public Rotation2d getYaw() {
        return new Rotation2d(yaw.getValue());
    }

}
