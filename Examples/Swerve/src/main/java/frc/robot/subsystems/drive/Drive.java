package frc.robot.subsystems.drive;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
 * This is the main drive subsystem. This class handles the overall swerve drive logic,
 * including keeping track of the robot's pose, converting robot speeds into swerve module commands,
 * and exposing an interface to allow other parts of the code to interact with the drive system.
 */
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

    // The pose estimator uses odometry data from the modules to estimate the robot's pose. Over time,
    // the pose estimate will drift using odometry alone, so the pose estimator fuses vision pose estimates with the odometry
    private final SwerveDrivePoseEstimator poseEstimator;
    private Rotation2d rawGyroHeading = new Rotation2d();

    public Drive(GyroIO gyroIO, Module... modules) {
        this.modules = modules;
        this.gyroIO = gyroIO;
        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics, 
            rawGyroHeading, 
            lastModulePositions,
            new Pose2d() // In a real competition bot, the initial pose would likely come from the driver station
        );
        
        /*
         * The odometry thread reads data from the sensors at a higher rate than the main loop, so that the odometry estimate is
         * more accurate, as it relies on numerical integration of sensor data over time to approximate the robot's true pose.
         * Specifically, it keeps track of the previous module position given to it, and when given a new module position, it calculates a
         * "twist" (the derivative of a pose, i.e. the robot's instantaneous velocity) and numerically integrates those twists over time to get the new pose.
         */
        OdometryThread.getInstance().start();
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // This method signals to the odometry thread that a main loop cycle has completed. It is imperative that
        // this metho is called BEFORE any data is accessed from the Odometry thread
        double[] timestamps = OdometryThread.getInstance().poll();

        // Logs data from the gyroscope
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);

        for (var module: modules) {
            module.periodic();
        }

        // Update the pose estimator with the latest module positions and gyro data
        int sampleCount = OdometryThread.getInstance().getSampleCount();
        for (int i = 0; i < sampleCount; i++) {
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int modIndex = 0; modIndex < 4; modIndex++){
                modulePositions[modIndex] = modules[modIndex].getOdometryPositions()[i];
                moduleDeltas[modIndex] = new SwerveModulePosition(
                    modulePositions[modIndex].distanceMeters - lastModulePositions[modIndex].distanceMeters,
                    modulePositions[modIndex].angle);
                lastModulePositions[modIndex] = modulePositions[modIndex];
            }
            if (gyroInputs.data.connected()) {
                // Use the real gyro angle
                rawGyroHeading = gyroInputs.odometryYawPositions[i];
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroHeading = rawGyroHeading.plus(new Rotation2d(twist.dtheta));
            }
            poseEstimator.updateWithTime(timestamps[i],rawGyroHeading,modulePositions);
        }
    }

    // This method resets the pose estimator's rotation to a specific angle.
    public void resetRotation(Rotation2d rotation) {
        poseEstimator.resetRotation(rotation);
    }

    
    // This method resets the pose estimator to a specific pose
    public void resetPose(Pose2d pose) {
        poseEstimator.resetPose(pose);
    }

    // This method seeds the pose estimate with a new initial pose. Because the pose estimator works by integrating
    // module positions over time, it is crucial that this initial pose is accurate.
    public void seedPoseEstimate(Pose2d pose) {
        poseEstimator.resetPosition(rawGyroHeading,lastModulePositions,pose);
    }

    public void recalculateGyroOffset() {
        poseEstimator.resetPosition(rawGyroHeading, lastModulePositions, poseEstimator.getEstimatedPosition());
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Rotation2d getHeading() {
        return poseEstimator.getEstimatedPosition().getRotation();
    }

    public Rotation2d getAllianceHeading() {
        // Returns the heading adjusted for the alliance color
        if (DriverStation.getAlliance().isPresent()) {
            return getHeading().plus(
                DriverStation.getAlliance().get() == DriverStation.Alliance.Red
                    ? Rotation2d.k180deg // Red alliance is 180 degrees offset
                    : Rotation2d.kZero // Blue alliance is no offset
            );
        } else {
            return getHeading();
        }

    }

    public void addVisionMeasurement(Pose2d pose, double timestamp) {
        poseEstimator.addVisionMeasurement(pose, timestamp);
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp, Vector<N3> stddevs) {
        poseEstimator.addVisionMeasurement(pose, timestamp, stddevs);
    }

    public void applyRobotSpeeds(ChassisSpeeds robotSpeeds){
        ChassisSpeeds.discretize(robotSpeeds, 0.02); // Discretize the speeds to reduce translational skew (Note, desaturation may reintroduce skew).
        SwerveModuleState[] discreteStates = kinematics.toSwerveModuleStates(robotSpeeds);
        for (int i = 0; i < modules.length; i++) {
            modules[i].setTargetState(discreteStates[i]);
        }
    }

    public void applyRobotSpeeds(ChassisSpeeds robotSpeeds, Force[] wheelForces){
        ChassisSpeeds.discretize(robotSpeeds, 0.02); // Discretize the speeds to reduce translational skew (Note, desaturation may reintroduce skew)
        SwerveModuleState[] discreteStates = kinematics.toSwerveModuleStates(robotSpeeds);
        for (int i = 0; i < modules.length; i++) {
            modules[i].setTargetState(discreteStates[i],wheelForces[i]);
        }
    }

    public void applyFieldSpeeds(ChassisSpeeds fieldSpeeds){
        ChassisSpeeds robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, getAllianceHeading());
        applyRobotSpeeds(robotSpeeds);
    }

    public void applyFieldSpeeds(ChassisSpeeds fieldSpeeds, Force[] wheelForces){
        ChassisSpeeds robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, getAllianceHeading());
        applyRobotSpeeds(robotSpeeds,wheelForces);
    }
}
