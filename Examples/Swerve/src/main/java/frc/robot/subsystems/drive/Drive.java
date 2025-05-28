package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
    
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules;
    private final GyroIO gyroIO;
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(DriveConstants.kModuleTranslations);
    private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
    }; //For delta tracking
    private final SwerveDrivePoseEstimator poseEstimator;

    public Drive(GyroIO gyroIO, Module... modules) {
        this.modules = modules;
        this.gyroIO = gyroIO;
        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics, 
            gyroIO.getYaw(), 
            lastModulePositions,
            new Pose2d() // In a real competition bot, the initial pose would likely come from the driver station
        );
        
        OdometryThread.getInstance().start(); // Start the odometry thread
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        double[] timestamps = OdometryThread.getInstance().poll();

        // Rest of periodic code goes here
        // Logs data from the gyroscope
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);

        for (var module: modules) {
            module.periodic();
        }
    }
}
