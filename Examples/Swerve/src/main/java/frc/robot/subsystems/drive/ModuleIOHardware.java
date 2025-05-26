package frc.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.concurrent.ConcurrentLinkedQueue;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Angle;

import frc.robot.subsystems.drive.DriveConstants.ModuleK.ModuleConfig;
import frc.robot.subsystems.drive.DriveConstants.ModuleK.Common.DriveMotorK;
import frc.robot.subsystems.drive.DriveConstants.ModuleK.Common.SteerMotorK;

import static frc.robot.subsystems.drive.DriveConstants.kOdometryFrequencyHz;

public class ModuleIOHardware implements ModuleIO {
    
    private final SparkMax m_steerMotor;
    private final SparkClosedLoopController m_steerController;
    private final RelativeEncoder m_steerEncoder;

    private final SparkMax m_driveMotor;
    private final SparkClosedLoopController m_driveController;
    private final RelativeEncoder m_driveEncoder;

    private final ConcurrentLinkedQueue<Long> timestampQueue;
    private final ConcurrentLinkedQueue<Double> drivePositionQueue;
    private final ConcurrentLinkedQueue<Double> turnPositionQueue;

    private final ArrayList<Long> timestampBuffer = new ArrayList<>();
    private final ArrayList<Double> drivePositionBuffer = new ArrayList<>();
    private final ArrayList<Double> turnPositionBuffer = new ArrayList<>();

    private final CANcoder m_steerCANcoder;
    private final StatusSignal<Angle> steerAbsolutePosition;

    public ModuleIOHardware(ModuleConfig config) {

        // Drive motor acquisition
        m_driveMotor = new SparkMax(config.DrivePort,MotorType.kBrushless);
        m_driveController = m_driveMotor.getClosedLoopController();
        m_driveEncoder = m_driveMotor.getEncoder();

        // Steer motor acquisition
        m_steerMotor = new SparkMax(config.SteerPort,MotorType.kBrushless);
        m_steerController = m_steerMotor.getClosedLoopController();
        m_steerEncoder = m_steerMotor.getEncoder();

        // CANcoder acquisition
        m_steerCANcoder = new CANcoder(config.EncoderPort);

        // CANcoder configuration
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.FutureProofConfigs = false;
        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
        encoderConfig.MagnetSensor.MagnetOffset = config.EncoderOffsetRots;
        steerAbsolutePosition = m_steerCANcoder.getAbsolutePosition();

        // Drive motor configuration
        SparkMaxConfig driveConfig = new SparkMaxConfig();
        driveConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(DriveMotorK.CurrentLimit)
            .voltageCompensation(DriveMotorK.VoltageCompensation);
        driveConfig
            .encoder
            .positionConversionFactor(DriveMotorK.PositionConversionFactor)
            .velocityConversionFactor(DriveMotorK.VelocityConversionFactor);
        driveConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(DriveMotorK.kP)
            .i(DriveMotorK.kI)
            .d(DriveMotorK.kD);
        driveConfig
            .signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs((int) (1000/kOdometryFrequencyHz))
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);
        m_driveMotor.configure(driveConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        m_driveEncoder.setPosition(0.0);

        // Steer motor configuration
        SparkMaxConfig steerConfig = new SparkMaxConfig();
        steerConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(SteerMotorK.CurrentLimit)
            .voltageCompensation(SteerMotorK.VoltageCompensation);
        steerConfig
            .encoder
            .positionConversionFactor(SteerMotorK.PositionConversionFactor)
            .velocityConversionFactor(SteerMotorK.VelocityConversionFactor);
        steerConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(SteerMotorK.kP)
            .i(SteerMotorK.kI)
            .d(SteerMotorK.kD)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0,1);
        steerConfig
            .signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs((int) (1000/kOdometryFrequencyHz))
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);

        
        
    }
}
