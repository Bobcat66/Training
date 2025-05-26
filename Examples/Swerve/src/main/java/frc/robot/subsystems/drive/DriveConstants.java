package frc.robot.subsystems.drive;


import edu.wpi.first.math.util.Units;

public final class DriveConstants {
    public static final int kOdometryFrequencyHz = 250;
    
    public static class ModuleK {
        public static class Common {
            public static class DriveMotorK {
                public static final int CurrentLimit = 60;
                public static final double gearRatio = 6.75;
                public static final double VoltageCompensation = 12;
                public static final double MaxModuleSpeed = 14.0; //Maximum attainable module speed
                public static final double WheelRadius = Units.inchesToMeters(4); //Meters
                public static final double WheelCOF = 1.0; //Coefficient of friction
                public static final double PositionConversionFactor = 2 * WheelRadius*Math.PI/gearRatio; //Units: Meters
                public static final double VelocityConversionFactor = PositionConversionFactor/60; //Units: Meters per second

                //PID constants
                public static final double kP = 0.035;
                public static final double kI = 0.000;
                public static final double kD = 0.0012;

                //Feedforward constants
                public static final double kV = 2.78;
                public static final double kS = 0.0;
                public static final double kA = 0.0;
            }

            public static class SteerMotorK {
                public static final int CurrentLimit = 60;
                public static final double VoltageCompensation = 12;
                public static final double gearRatio = 12.8;
                public static final double PositionConversionFactor = 1/gearRatio; //Units: Rotations
                public static final double VelocityConversionFactor = PositionConversionFactor; //Units: RPM

                //PID constants
                public static double kP = 0.75;
                public static final double kI = 0.0;
                public static final double kD = 0.0001;
            }
        }
        public static enum ModuleConfig {
    
            FrontLeft(1,11,21,-0.441162109375 +0.5),
            FrontRight(2,12,22,-0.3984375 +0.5),
            RearLeft(3,13,23,-0.525146484375 ),
            RearRight(4,14,24,-0.931396484375);
    
            public final int DrivePort;
            public final int SteerPort;
            public final int EncoderPort;
            public final double EncoderOffsetRots;
    
            private ModuleConfig(int DrivePort, int SteerPort,int EncoderPort,double EncoderOffsetRots) {
                this.DrivePort = DrivePort;
                this.SteerPort = SteerPort;
                this.EncoderPort = EncoderPort;
                this.EncoderOffsetRots = EncoderOffsetRots;
            }
        }
    }
}
