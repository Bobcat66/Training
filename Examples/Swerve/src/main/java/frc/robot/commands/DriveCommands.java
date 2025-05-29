package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public final class DriveCommands {

    private DriveCommands () {}

    public static Command driveFieldRelative(
        DoubleSupplier xSupplier, 
        DoubleSupplier ySupplier, 
        DoubleSupplier omegaSupplier,
        Drive drive
    ) {
        return drive.run(() -> {
            var speeds = new ChassisSpeeds(
                xSupplier.getAsDouble(), 
                ySupplier.getAsDouble(), 
                omegaSupplier.getAsDouble()
            );
            drive.applyFieldSpeeds(speeds);
        });
    }

    public static Command driveRobotRelative(
        DoubleSupplier xSupplier, 
        DoubleSupplier ySupplier, 
        DoubleSupplier omegaSupplier,
        Drive drive
    ) {
        return drive.run(() -> {
            var speeds = new ChassisSpeeds(
                xSupplier.getAsDouble(), 
                ySupplier.getAsDouble(), 
                omegaSupplier.getAsDouble()
            );
            drive.applyRobotSpeeds(speeds);
        });
    }
}
