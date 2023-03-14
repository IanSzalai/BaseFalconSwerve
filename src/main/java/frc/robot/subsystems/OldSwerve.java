// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OldSwerveModule;

public class OldSwerve extends SubsystemBase {

  public OldSwerveModule[] mSwerveMods;
  public AHRS gyro;

  public OldSwerve() {

    gyro = new AHRS();
    gyro.reset();

    mSwerveMods = new OldSwerveModule[] {
        new OldSwerveModule(0, Constants.Swerve.Mod0.constants),
        new OldSwerveModule(1, Constants.Swerve.Mod1.constants),
        new OldSwerveModule(2, Constants.Swerve.Mod2.constants),
        new OldSwerveModule(3, Constants.Swerve.Mod3.constants)
    };
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
        new ChassisSpeeds(
            translation.getX(),
            translation.getY(),
            rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for (OldSwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber]);
    }
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (OldSwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber]);
    }
  }

  public void zeroGyro() {
    gyro.reset();
  }

  public void setMod0Speed(double speed) {
    mSwerveMods[0].setDesiredState(new SwerveModuleState(speed, new Rotation2d()));
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Gyro Yaw", gyro.getRotation2d().getDegrees());

    for (OldSwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Absolute Encoder", mod.getAngle().getDegrees());
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Raw Absolute Encoder", mod.getRawAbsoluteEncoder());
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Raw Drive Encoder", mod.getRawDriveEncoder());

      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle PID Setpoint", mod.getAnglePIDSetpoint());
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle PID Error", mod.getAnglePIDError());
    }
  }
}
