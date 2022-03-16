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
  
  SwerveModule frontRightModule;
  SwerveModule frontLeftModule;
  SwerveModule backLeftModule;
  SwerveModule backRightModule;

  double convertedGyroAngle = 0;

  public SwerveGroup() {
    frontRightModule = RobotContainer.frontRightModule;
    frontLeftModule = RobotContainer.frontLeftModule;
    backLeftModule = RobotContainer.backLeftModule;
    backRightModule = RobotContainer.backRightModule;
  }




  // Crab swerve control
  public void moveCrab(Vector2d translation, double rotation) {
    double yawAngle = RobotContainer.navX.getAngle()/* - Constants.GYRO_OFFSET*/;  //get the yaw angle from the RoboRio gyroscope
    SmartDashboard.putNumber("Gyro Angle: ", getConvertedGyroAngle());

    double joystickAngle = (Math.atan2(-translation.y, translation.x) * (180/Math.PI)) + 180 + yawAngle;   //preform fancy math to find the angle the joystick is at

    if (Math.abs(translation.magnitude()) > Constants.RobotConstants.JOYSTICK_DEAD_ZONE) {
      //Move each module at the speed determined from the translation vector at the angle of the joystick
      frontRightModule.move(translation.magnitude(), joystickAngle);
      frontLeftModule.move(translation.magnitude(), joystickAngle);
      backLeftModule.move(translation.magnitude(), joystickAngle);
      backRightModule.move(translation.magnitude(), joystickAngle);
    }
    else if (Math.abs(rotation) > Constants.RobotConstants.JOYSTICK_DEAD_ZONE) {
      
      frontLeftModule.move(rotation, 53); //225
      backRightModule.move(rotation, 233);  //45

      frontRightModule.move(rotation, 127);  //135
      backLeftModule.move(rotation, 307);  //315
    }
    else {
      // If none of the requirements are met, make the modules stop
      frontRightModule.stop();
      frontLeftModule.stop();
      backLeftModule.stop();
      backRightModule.stop();
    }
  }




  // Swerve control using the built in WPILib kinematics calculations, see WPILib documentation for instructions
  public void moveSwerveWPILib(Vector2d translation, double rotation){
    // Establishing the location of each module relative to the center of the robot.
    Translation2d frontLeftLocation = new Translation2d(SharedMethods.convertInchesToMeters(Constants.RobotConstants.ROBOT_WIDTH) / 2, SharedMethods.convertInchesToMeters(Constants.RobotConstants.ROBOT_LENGTH) / 2);
    Translation2d frontRightLocation = new Translation2d(SharedMethods.convertInchesToMeters(Constants.RobotConstants.ROBOT_WIDTH) / 2, SharedMethods.convertInchesToMeters(Constants.RobotConstants.ROBOT_LENGTH) / -2);
    Translation2d backLeftLocation = new Translation2d(SharedMethods.convertInchesToMeters(Constants.RobotConstants.ROBOT_WIDTH) / -2, SharedMethods.convertInchesToMeters(Constants.RobotConstants.ROBOT_LENGTH) / 2);
    Translation2d backRightLocation = new Translation2d(SharedMethods.convertInchesToMeters(Constants.RobotConstants.ROBOT_WIDTH) / -2, SharedMethods.convertInchesToMeters(Constants.RobotConstants.ROBOT_LENGTH) / -2);

    SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(
      frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation
    );

    double FWD = translation.x;
    double STR = translation.y;
    double RCW = rotation * ((-0.75 * translation.magnitude()) + 1);  //rotate variable (z-axis)

    if (Math.abs(translation.magnitude()) <= Constants.RobotConstants.JOYSTICK_DEAD_ZONE){
      FWD = 0; 
      STR = 0;
    }
    if (Math.abs(rotation) <= Constants.RobotConstants.JOYSTICK_DEAD_ZONE) RCW = 0;

    // field/robot oriented drive system.
    SmartDashboard.putNumber("Gyro Angle: ", getConvertedGyroAngle()); 


    // double gyroRadians = getConvertedGyroAngle() * (Math.PI / 180); //in radians
    // double temp = (FWD * Math.cos(gyroRadians)) + (STR * Math.sin(gyroRadians));
    // STR = (-FWD * Math.sin(gyroRadians)) + (STR * Math.cos(gyroRadians));
    // FWD = temp;
    
    // Code for Field Oriented drive, dosn't work
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(FWD, STR, RCW);
    // chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(FWD, STR, RCW, Rotation2d.fromDegrees(RobotContainer.navX.getRotation2d().getDegrees() + Constants.RobotConstants.GYRO_OFFSET));
    
    SwerveModuleState[] moduleStates = swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    //SmartDashboard.putNumber("Field Angle", RobotContainer.navX.getRotation2d().getDegrees());

    SwerveModuleState frontLeftState = moduleStates[0];
    SwerveModuleState frontRightState = moduleStates[1];
    SwerveModuleState backLeftState = moduleStates[2];
    SwerveModuleState backRightState = moduleStates[3];

    frontLeftState = SwerveModuleState.optimize(frontLeftState, new Rotation2d(SharedMethods.convertDegreesToRadians(frontLeftModule.currentAngle)));
    frontRightState = SwerveModuleState.optimize(frontRightState, new Rotation2d(SharedMethods.convertDegreesToRadians(frontRightModule.currentAngle)));
    backLeftState = SwerveModuleState.optimize(backLeftState, new Rotation2d(SharedMethods.convertDegreesToRadians(backLeftModule.currentAngle)));
    backRightState = SwerveModuleState.optimize(backRightState, new Rotation2d(SharedMethods.convertDegreesToRadians(backRightModule.currentAngle)));

    SmartDashboard.putNumber("FL State Angle", frontLeftState.angle.getDegrees());
    SmartDashboard.putNumber("FR State Angle", frontRightState.angle.getDegrees());
    SmartDashboard.putNumber("BL State Angle", backLeftState.angle.getDegrees());
    SmartDashboard.putNumber("BR State Angle", backRightState.angle.getDegrees());

    if (Math.abs(translation.magnitude()) > Constants.RobotConstants.JOYSTICK_DEAD_ZONE || Math.abs(rotation) > Constants.RobotConstants.JOYSTICK_DEAD_ZONE) {
      frontLeftModule.move(frontLeftState.speedMetersPerSecond, frontLeftState.angle.getDegrees());
      frontRightModule.move(frontRightState.speedMetersPerSecond, frontRightState.angle.getDegrees());
      backLeftModule.move(backLeftState.speedMetersPerSecond, backLeftState.angle.getDegrees());
      backRightModule.move(backRightState.speedMetersPerSecond, backRightState.angle.getDegrees());
    }
    else {
      frontRightModule.stop();
      frontLeftModule.stop();
      backLeftModule.stop();
      backRightModule.stop();
    }
  }




  // Hand calculated Swerve Control, still buggy
  public void moveSwerve(Vector2d translation, double rotation) {
    double gyroRadians = getConvertedGyroAngle() * (Math.PI / 180); //in radians
    SmartDashboard.putNumber("Gyro Angle: ", getConvertedGyroAngle());

    double STR = translation.x;  //strafe variable (x-axis)
    double FWD = translation.y;  //forward variable (y-axis)
    double RCW = rotation * ((-0.75 * translation.magnitude()) + 1);  //rotate variable (z-axis)
    /**
     * rotation linearly adjusted for translation speed
     * states that the rotation is 1x if the translation speed is 0
     *  and 0.25x is the translation speed = full (1)
     *  based on the equation of a line (0.75x + 1), where x is the translation speed
    */

    
    // field/robot oriented drive system. Consider swithcing to robot oriented to avoid gyro drift
    // and related complications, will require more driver practice
    double temp = (FWD * Math.cos(gyroRadians)) + (STR * Math.sin(gyroRadians));
    STR = (-FWD * Math.sin(gyroRadians)) + (STR * Math.cos(gyroRadians));
    FWD = temp;
    

    if (Math.abs(translation.magnitude()) <= Constants.RobotConstants.JOYSTICK_DEAD_ZONE) {
      FWD = 0;
      STR = 0;
    }
    if (Math.abs(rotation) <= Constants.RobotConstants.JOYSTICK_DEAD_ZONE) RCW = 0;
    SmartDashboard.putNumber("STR", STR);
    SmartDashboard.putNumber("FWD", FWD);
    SmartDashboard.putNumber("RCW", RCW);


    // This section is where the speed for each swerve module is calculated
    // variables A, B, C, and D are established
    // If modules are inverted, switch numbers and letters around until it is fixed

    double A = STR - RCW * (Constants.RobotConstants.ROBOT_LENGTH / Constants.RobotConstants.ROBOT_RADIUS);
    double B = STR + RCW * (Constants.RobotConstants.ROBOT_LENGTH / Constants.RobotConstants.ROBOT_RADIUS);
    double C = FWD - RCW * (Constants.RobotConstants.ROBOT_WIDTH / Constants.RobotConstants.ROBOT_RADIUS);
    double D = FWD + RCW * (Constants.RobotConstants.ROBOT_WIDTH / Constants.RobotConstants.ROBOT_RADIUS);

    SmartDashboard.putNumber("A, STR - RCW", A);
    SmartDashboard.putNumber("B, STR + RCW", B);
    SmartDashboard.putNumber("C, FWD - RCW", C);
    SmartDashboard.putNumber("D, FWD + RCW", D);

    double frontRightSpeed = getMovementAttributes(B, C)[0]; //good
    double frontRightAngle = getMovementAttributes(B, C)[1];
    double maxSpeed = frontRightSpeed;  // The module to control all speeds

    //B and D
    double frontLeftSpeed = getMovementAttributes(B, D)[0]; 
    double frontLeftAngle = getMovementAttributes(B, D)[1];
    if (frontLeftSpeed > maxSpeed) frontLeftSpeed = maxSpeed;

    //A and D
    double backLeftSpeed = getMovementAttributes(A, D)[0]; //good
    double backLeftAngle = getMovementAttributes(A, D)[1];
    if (backLeftSpeed > maxSpeed) backLeftSpeed = maxSpeed;

    //A and C - Back Right
    double backRightSpeed = getMovementAttributes(A, C)[0];
    double backRightAngle = getMovementAttributes(A, C)[1];
    if (backRightSpeed > maxSpeed) backRightSpeed = maxSpeed;


    if (Math.abs(translation.magnitude()) > Constants.RobotConstants.JOYSTICK_DEAD_ZONE || Math.abs(rotation) > Constants.RobotConstants.JOYSTICK_DEAD_ZONE) {
      frontRightModule.move(frontRightSpeed, frontRightAngle);
      frontLeftModule.move(frontLeftSpeed, frontLeftAngle);
      backLeftModule.move(backLeftSpeed, backLeftAngle);
      backRightModule.move(backRightSpeed, backRightAngle);
    }
    else {
      frontRightModule.stop();
      frontLeftModule.stop();
      backLeftModule.stop();
      backRightModule.stop();
    }
  }
  public double[] getMovementAttributes(double c1, double c2) {
    double speed = Math.sqrt(Math.pow(c1, 2) + Math.pow(c2, 2));
    double angle = Math.atan2(c1, c2) * (180 / Math.PI) + 90;  // Math.atan2(x,y) returns the angle between the x-axis and the coordinate (x,y)

    return new double[] { speed, angle };
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