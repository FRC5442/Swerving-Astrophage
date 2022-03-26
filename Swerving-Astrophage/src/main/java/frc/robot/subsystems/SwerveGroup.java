// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.SharedMethods;
import frc.robot.Constants.RobotConstants;

// The SwerveGroup subsystem tells each individuall swerve module how to work together to move the robot.
// This is where the complicated math and swerve programming lies

public class SwerveGroup extends SubsystemBase {

  private final SwerveModule frontRightModule = 
  new SwerveModule(
    RobotContainer.driveMotor1, 
    RobotContainer.driveMotor2, 
    RobotContainer.frontRightAbsEncoder, 
    Constants.SwerveConstants.FR_INVERTED, 
    Constants.SwerveConstants.FR_OFFSET
    );

  private final SwerveModule frontLeftModule = 
    new SwerveModule(
      RobotContainer.driveMotor3, 
      RobotContainer.driveMotor4, 
      RobotContainer.frontLeftAbsEncoder, 
      Constants.SwerveConstants.FL_INVERTED, 
      Constants.SwerveConstants.FL_OFFSET
      );

  private final SwerveModule backLeftModule = 
    new SwerveModule(
      RobotContainer.driveMotor5, 
      RobotContainer.driveMotor6, 
      RobotContainer.backLeftAbsEncoder, 
      Constants.SwerveConstants.FL_INVERTED, 
      Constants.SwerveConstants.FL_OFFSET
      );

  private final SwerveModule backRightModule = 
    new SwerveModule(
      RobotContainer.driveMotor7, 
      RobotContainer.driveMotor8, 
      RobotContainer.backRightAbsEncoder, 
      Constants.SwerveConstants.BR_INVERTED, 
      Constants.SwerveConstants.BR_OFFSET
      );
  
  
  // SwerveModule frontRightModule;
  // SwerveModule frontLeftModule;
  // SwerveModule backLeftModule;
  // SwerveModule backRightModule;

  double convertedGyroAngle = 0;

  public SwerveGroup() {}

  // Swerve control using the built in WPILib kinematics calculations, see WPILib documentation for instructions
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative){

    var swerveModuleStates =
        Constants.SwerveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, RobotContainer.navX.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, Constants.SwerveConstants.kMaxSpeedMetersPerSecond);
    frontLeftModule.setDesiredState(swerveModuleStates[0]);
    frontRightModule.setDesiredState(swerveModuleStates[1]);
    backLeftModule.setDesiredState(swerveModuleStates[2]);
    backRightModule.setDesiredState(swerveModuleStates[3]);
  }

  public void stopSwerve(){
  }


  public void calibrate() {
    frontRightModule.calibrate();
    frontLeftModule.calibrate();
    backLeftModule.calibrate();
    backRightModule.calibrate();
  }



  public void switchDriveState(Constants.DRIVE_STATE driveState) {
    frontRightModule.switchTranslationMod(driveState.getValue());
    frontLeftModule.switchTranslationMod(driveState.getValue());
    backLeftModule.switchTranslationMod(driveState.getValue());
    backRightModule.switchTranslationMod(driveState.getValue());
  }

  public double getConvertedGyroAngle() {
    double rawGyroAngle = (RobotContainer.navX.getAngle() + Constants.RobotConstants.GYRO_OFFSET); //in degrees
    SmartDashboard.putNumber("Raw Gyro Angle", rawGyroAngle);
    double convertedRawGyroAngle = ((360 - rawGyroAngle + 90) % 360);
    if (convertedRawGyroAngle < 0) {
      return SharedMethods.roundTo(360 + convertedRawGyroAngle, 0);
    }
    else {
      return SharedMethods.roundTo(convertedRawGyroAngle, 0);
    }
  }

  @Override
  public void periodic() {

  }
}