// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import frc.lib.util.OldSwerveModuleConstants;

/**
 * Swerve module using a Victor SP motor controller for drive and angle motors,
 * a continous potentiometer for the absolute steering encoder, and a quadrature
 * encoder for the relative steering encoder.
 */
public class OldSwerveModule {
    public int moduleNumber;
    private Rotation2d absoluteAngleOffset;
    private Rotation2d relativeAngleOffset;
    private Rotation2d lastAngle;

    private VictorSP mAngleMotor;
    private VictorSP mDriveMotor;
    private AnalogPotentiometer absoluteAngleEncoder;
    private Encoder relativeAngleEncoder;

    private double relativeAngleEncoderCountsPerWheelRotation;

    private PIDController anglePID;
    private double anglePIDTolerance;

    public OldSwerveModule(int moduleNumber, OldSwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.absoluteAngleOffset = moduleConstants.angleOffset;

        /* Angle Encoders Config */
        absoluteAngleEncoder = new AnalogPotentiometer(
                moduleConstants.absoluteAngleEncoderAnalog, 360, 0);
        relativeAngleEncoder = new Encoder(
                moduleConstants.relativeAngleEncoderDIOA,
                moduleConstants.relativeAngleEncoderDIOB,
                moduleConstants.absoluteAngleEncoderInvert);
        configAngleEncoders();

        /* Angle Motor Config */
        mAngleMotor = new VictorSP(moduleConstants.angleMotorPWM);
        relativeAngleEncoderCountsPerWheelRotation = 1000;
        configAngleMotor();

        /* Angle PID Config */
        anglePID = new PIDController(
                Constants.Swerve.angleKP,
                Constants.Swerve.angleKI,
                Constants.Swerve.angleKD);
        anglePIDTolerance = 0.5;
        configAnglePID();

        /* Drive Motor Config */
        mDriveMotor = new VictorSP(moduleConstants.driveMotorPWM);
        configDriveMotor();

    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, getAngle());
        setAngle(state);
        setSpeed(state);
    }

    private void setSpeed(SwerveModuleState desiredState) {
        double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
        mDriveMotor.set(percentOutput);
    }

    private void setAngle(SwerveModuleState desiredState) {
        // Prevent rotating module if speed is less than 1%. Prevents jittering.
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
                ? lastAngle
                : desiredState.angle;

        double percentOutput = anglePID.calculate(getAngle().getDegrees(), angle.getDegrees());

        mAngleMotor.set(percentOutput);
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(relativeAngleEncoder.getDistance() - relativeAngleOffset.getDegrees());
    }

    private Rotation2d getAbsoluteEncoder() {
        return Rotation2d.fromDegrees(absoluteAngleEncoder.get());
    }

    public void resetToAbsolute() {
        double absolutePosition = getAbsoluteEncoder().getDegrees() - absoluteAngleOffset.getDegrees();
        relativeAngleEncoder.reset();
        relativeAngleOffset = Rotation2d.fromDegrees(absolutePosition);
    }

    private void configAngleEncoders() {
        relativeAngleEncoder.setDistancePerPulse(relativeAngleEncoderCountsPerWheelRotation / 360);
    }

    private void configAngleMotor() {
        mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        mAngleMotor.setSafetyEnabled(false);
    }

    private void configAnglePID() {
        anglePID.enableContinuousInput(-180, 180);
        anglePID.setTolerance(anglePIDTolerance);
    }

    private void configDriveMotor() {
        mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        mDriveMotor.setSafetyEnabled(false);
    }
}
