package frc.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class OldSwerveModuleConstants {
    public final int driveMotorPWM;
    public final int angleMotorPWM;
    public final int absoluteAngleEncoderAnalog;
    public final boolean absoluteAngleEncoderInvert;
    public final int relativeAngleEncoderDIOA;
    public final int relativeAngleEncoderDIOB;
    public final boolean relativeAngleEncoderInvert;
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
     * @param relativeAngleEncoderDIOA   DIO port that the first channel of the
     *                                   relative angle encoder is plugged in to
     * @param relativeAngleEncoderDIOB   DIO port that the second channel of the
     *                                   relative angle encoder is plugged in to
     * @param relativeAngleEncoderInvert Is the relative encoder inverted
     * @param angleOffset                Angle that the absolute encoder reads when
     *                                   the physical wheel has an angle of zero
     */
    public OldSwerveModuleConstants(
            int driveMotorPWM,
            int angleMotorPWM,
            int absoluteAngleEncoderAnalog,
            boolean absoluteAngleEncoderInvert,
            int relativeAngleEncoderDIOA,
            int relativeAngleEncoderDIOB,
            boolean relativeAngleEncoderInvert,
            Rotation2d angleOffset) {
        this.driveMotorPWM = driveMotorPWM;
        this.angleMotorPWM = angleMotorPWM;
        this.absoluteAngleEncoderAnalog = absoluteAngleEncoderAnalog;
        this.absoluteAngleEncoderInvert = absoluteAngleEncoderInvert;
        this.relativeAngleEncoderDIOA = relativeAngleEncoderDIOA;
        this.relativeAngleEncoderDIOB = relativeAngleEncoderDIOB;
        this.relativeAngleEncoderInvert = relativeAngleEncoderInvert;
        this.angleOffset = angleOffset;
    }

}
