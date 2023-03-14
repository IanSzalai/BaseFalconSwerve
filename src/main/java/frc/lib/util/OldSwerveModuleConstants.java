package frc.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class OldSwerveModuleConstants {
    public final int driveMotorPWM;
    public final int angleMotorPWM;
    public final int absoluteAngleEncoderAnalog;
    public final boolean absoluteAngleEncoderInvert;
    public final int driveEncoderDIOA;
    public final int driveEncoderDIOB;
    public final boolean driveEncoderInvert;
    public final Rotation2d angleOffset;

    /**
     * Old Swerve Module Constants to be used when creating Old Swerve Modules.
     * 
     * @param driveMotorPWM              PWM port on the roboRIO the drive motor is
     *                                   plugged in to
     * @param angleMotorPWM              PWM port on the roboRIO the angle motor is
     *                                   plugged in to
     * @param absoluteAngleEncoderAnalog Analog port on the roboRIO the absolute
     *                                   analog angle encoder is plugged in to
     * @param absoluteAngleEncoderInvert Is the absolute encoder inverted
     * @param driveEncoderDIOA           DIO port that the first channel of the
     *                                   drive encoder is plugged in to
     * @param driveEncoderDIOB           DIO port that the second channel of the
     *                                   drive encoder is plugged in to
     * @param driveEncoderInvert         Is the drive encoder inverted
     * @param angleOffset                Angle that the absolute encoder reads when
     *                                   the physical wheel has an angle of zero
     */
    public OldSwerveModuleConstants(
            int driveMotorPWM,
            int angleMotorPWM,
            int absoluteAngleEncoderAnalog,
            boolean absoluteAngleEncoderInvert,
            int driveEncoderDIOA,
            int driveEncoderDIOB,
            boolean driveEncoderInvert,
            Rotation2d angleOffset) {
        this.driveMotorPWM = driveMotorPWM;
        this.angleMotorPWM = angleMotorPWM;
        this.absoluteAngleEncoderAnalog = absoluteAngleEncoderAnalog;
        this.absoluteAngleEncoderInvert = absoluteAngleEncoderInvert;
        this.driveEncoderDIOA = driveEncoderDIOA;
        this.driveEncoderDIOB = driveEncoderDIOB;
        this.driveEncoderInvert = driveEncoderInvert;
        this.angleOffset = angleOffset;
    }

}
