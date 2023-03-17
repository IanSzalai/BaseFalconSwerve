package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.OldSwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(15);
        public static final double wheelBase = Units.inchesToMeters(15);
        public static final double wheelCircumference = Units.inchesToMeters(4) * Math.PI;

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Angle Motor PID Values */
        public static final double angleKP = 0.01;
        public static final double angleKI = 0;
        public static final double angleKD = 0;
        public static final double angleKF = 0;

        public static final double angleMaxPercentOutput = 0.5;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; // TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        public static final boolean driveMotorInvert = false;
        public static final boolean angleMotorInvert = true;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; // TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; // TODO: This must be tuned to specific robot

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { // TODO: This must be tuned to specific robot
            public static final int driveMotorPWM = 0;
            public static final int angleMotorPWM = 1;
            public static final int absoluteAngleEncoderAnalog = 0;
            public static final boolean absoluteAngleEncoderInvert = false;
            public static final int relativeAngleEncoderDIOA = 0;
            public static final int relativeAngleEncoderDIOB = 1;
            public static final boolean relativeAngleEncoderInvert = false;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(139.416867);
            public static final OldSwerveModuleConstants constants = new OldSwerveModuleConstants(
                    driveMotorPWM,
                    angleMotorPWM,
                    absoluteAngleEncoderAnalog,
                    absoluteAngleEncoderInvert,
                    relativeAngleEncoderDIOA,
                    relativeAngleEncoderDIOB,
                    relativeAngleEncoderInvert,
                    angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { // TODO: This must be tuned to specific robot
            public static final int driveMotorPWM = 4;
            public static final int angleMotorPWM = 5;
            public static final int absoluteAngleEncoderAnalog = 2;
            public static final boolean absoluteAngleEncoderInvert = false;
            public static final int relativeAngleEncoderDIOA = 4;
            public static final int relativeAngleEncoderDIOB = 5;
            public static final boolean relativeAngleEncoderInvert = false;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(170.079038);
            public static final OldSwerveModuleConstants constants = new OldSwerveModuleConstants(
                    driveMotorPWM,
                    angleMotorPWM,
                    absoluteAngleEncoderAnalog,
                    absoluteAngleEncoderInvert,
                    relativeAngleEncoderDIOA,
                    relativeAngleEncoderDIOB,
                    relativeAngleEncoderInvert,
                    angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 { // TODO: This must be tuned to specific robot
            public static final int driveMotorPWM = 3;
            public static final int angleMotorPWM = 2;
            public static final int absoluteAngleEncoderAnalog = 1;
            public static final boolean absoluteAngleEncoderInvert = false;
            public static final int relativeAngleEncoderDIOA = 2;
            public static final int relativeAngleEncoderDIOB = 3;
            public static final boolean relativeAngleEncoderInvert = false;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(9.825341);
            public static final OldSwerveModuleConstants constants = new OldSwerveModuleConstants(
                    driveMotorPWM,
                    angleMotorPWM,
                    absoluteAngleEncoderAnalog,
                    absoluteAngleEncoderInvert,
                    relativeAngleEncoderDIOA,
                    relativeAngleEncoderDIOB,
                    relativeAngleEncoderInvert,
                    angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { // TODO: This must be tuned to specific robot
            public static final int driveMotorPWM = 7;
            public static final int angleMotorPWM = 6;
            public static final int absoluteAngleEncoderAnalog = 3;
            public static final boolean absoluteAngleEncoderInvert = false;
            public static final int relativeAngleEncoderDIOA = 6;
            public static final int relativeAngleEncoderDIOB = 7;
            public static final boolean relativeAngleEncoderInvert = false;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(178.321845);
            public static final OldSwerveModuleConstants constants = new OldSwerveModuleConstants(
                    driveMotorPWM,
                    angleMotorPWM,
                    absoluteAngleEncoderAnalog,
                    absoluteAngleEncoderInvert,
                    relativeAngleEncoderDIOA,
                    relativeAngleEncoderDIOB,
                    relativeAngleEncoderInvert,
                    angleOffset);
        }
    }

    public static final class AutoConstants { // TODO: The below constants are used in the example auto, and must be
                                              // tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
