package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
    
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules;
    private final GyroIO gyroIO;

    public Drive(GyroIO gyroIO, Module... modules) {
        this.modules = modules;
        this.gyroIO = gyroIO;
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        double[] timestamps = OdometryThread.getInstance().poll();

        // Rest of periodic code goes here
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);

        for (var module: modules) {
            module.periodic();
        }
    }
}
