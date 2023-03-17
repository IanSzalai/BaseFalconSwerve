// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
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
    private Rotation2d lastAngle;

    private VictorSP mAngleMotor;
    private VictorSP mDriveMotor;
    private AnalogPotentiometer absoluteAngleEncoder;
    private Encoder driveEncoder;

    private PIDController anglePID;

    public OldSwerveModule(int moduleNumber, OldSwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.absoluteAngleOffset = moduleConstants.angleOffset;

        /* Angle Encoders Config */
        absoluteAngleEncoder = new AnalogPotentiometer(moduleConstants.absoluteAngleEncoderAnalog, 360);
        configAngleEncoders();

        /* Angle Motor Config */
        mAngleMotor = new VictorSP(moduleConstants.angleMotorPWM);
        configAngleMotor();

        /* Angle PID Config */
        anglePID = new PIDController(
                Constants.Swerve.angleKP,
                Constants.Swerve.angleKI,
                Constants.Swerve.angleKD);
        configAnglePID();

        /* Drive Motor Config */
        mDriveMotor = new VictorSP(moduleConstants.driveMotorPWM);
        configDriveMotor();

        /* Drive Encoder Config */
        driveEncoder = new Encoder(
                moduleConstants.driveEncoderDIOA,
                moduleConstants.driveEncoderDIOB,
                moduleConstants.driveEncoderInvert);
        configDriveEncoder();

        lastAngle = getAngle();

    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
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

        percentOutput = MathUtil.clamp(percentOutput, -Constants.Swerve.angleMaxPercentOutput,
                Constants.Swerve.angleMaxPercentOutput);

        mAngleMotor.set(percentOutput);
        lastAngle = angle;
    }

    public Rotation2d getAngle() {
        double degrees = absoluteAngleEncoder.get() - absoluteAngleOffset.getDegrees();
        degrees = MathUtil.inputModulus(degrees, -180, 180);
        return Rotation2d.fromDegrees(degrees);
    }

    public double getRawAbsoluteEncoder() {
        return absoluteAngleEncoder.get();
    }

    public double getRawDriveEncoder() {
        return driveEncoder.getRaw();
    }

    public double getAnglePIDSetpoint() {
        return anglePID.getSetpoint();
    }

    public double getAnglePIDError() {
        return anglePID.getPositionError();
    }

    private void configAngleEncoders() {
        // no config needed
    }

    private void configAngleMotor() {
        mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        mAngleMotor.setSafetyEnabled(false);
    }

    private void configAnglePID() {
        anglePID.enableContinuousInput(-180, 180);
    }

    private void configDriveMotor() {
        mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        mDriveMotor.setSafetyEnabled(false);
    }

    private void configDriveEncoder() {
        driveEncoder.reset();
        // driveEncoder.setDistancePerPulse(something);
    }
}
