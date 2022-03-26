// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SharedMethods;

// This is the parent class for each individual swerve module. Each individual swerve module extends
// this class, so it will have these properties.
// Specifically, this class tells the swerve module how to move at a passed speed or rotate to a 
// given angle.

public class SwerveModule extends SubsystemBase {
  TalonFX topGear, bottomGear;
  AnalogPotentiometer absEncoder;
  RelativeEncoder topEncoder, bottomEncoder;
  public double currentAngle = 0.0;
  double rawAngle = 0.0;
  double startTime = 0.0;
  double elapsedTime = 500;
  double zeroOffset = 0;

  double TRANSLATE_MOD = 0.4;
  double ROTATE_MOD = 0.3;
  double ERROR_BOUND = 1;

  // private final PIDController drivePIDController = new PIDController(1, 0, 0);
  // private final ProfiledPIDController turningPIDController = new ProfiledPIDController.Constraints(1, 0, 0, new TrapezoidProfile(Math.PI, 2 * Math.PI));

  private final PIDController m_topGearController =
      new PIDController(Constants.SwerveConstants.kTopGearController, 0, 0);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_bottomGearController =
      new ProfiledPIDController(
        Constants.SwerveConstants.kBottomGearController,
          0,
          0,
          new TrapezoidProfile.Constraints(
              Constants.SwerveConstants.kMaxModuleAngularSpeedRadiansPerSecond,
              Constants.SwerveConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  private final SimpleMotorFeedforward topGearFeedForward = new SimpleMotorFeedforward(1, 3);
  private final SimpleMotorFeedforward bottomGearFeedForward = new SimpleMotorFeedforward(1, 0.5);

  double topGearSpeed = 0;
  double bottomGearSpeed = 0;

  String moduleID = "";





  public SwerveModule(TalonFX topGear, TalonFX bottomGear, AnalogPotentiometer absEncoder, boolean invertedTranslate, double zeroOffset) {
    this.moduleID = moduleID.toUpperCase();

    this.zeroOffset = zeroOffset;

    this.topGear = topGear;
    this.bottomGear = bottomGear;
    this.absEncoder = absEncoder;

    m_bottomGearController.enableContinuousInput(-Math.PI, Math.PI);

    if (invertedTranslate) {
      TRANSLATE_MOD *= -1;
    }
  }






  public void setDesiredState(SwerveModuleState desiredState){
    SwerveModuleState state = 
    SwerveModuleState.optimize(desiredState, new Rotation2d(currentAngle));
    
    final double topGearOutput =
    m_bottomGearController.calculate(getTopGearSpeed(), state.speedMetersPerSecond);

  // Calculate the turning motor output from the turning PID controller.
    final double bottomGearOutput =
    m_bottomGearController.calculate(getBottomGearPosition(), state.angle.getRadians());

  // Calculate the turning motor output from the turning PID controller.
    topGearSpeed = topGearOutput;
    bottomGearSpeed = bottomGearOutput;
  }

  public void calibrate() {
    zeroOffset = rawAngle;
  }

  public double getTopGearPosition(){
    return topGear.getSelectedSensorPosition();
  }

  public double getBottomGearPosition(){
    return bottomGear.getSelectedSensorPosition();
  }

  public double getTopGearSpeed(){
    return topGear.getSelectedSensorVelocity();
  }

  public double getBottomGearSpeed(){
    return bottomGear.getSelectedSensorVelocity();
  }

  public void switchTranslationMod(double value) {
    TRANSLATE_MOD = value;
  }

  public void stop() {
    bottomGearSpeed = 0;
    topGearSpeed = 0;
    // bottomGearSpeed = 0;
  }

  @Override
  public void periodic() {
    updateGearSpeeds();
    updateCurrentAngle();
    updateSmartDashboard();
  }

  public void updateCurrentAngle() {
    elapsedTime = (System.nanoTime() / 1000000) - startTime;
    if (elapsedTime >= 10) {
      startTime = System.nanoTime() / 1000000;

      //convert absolute encoder voltage to degrees and post to smartdashboard for testing
      rawAngle = (SharedMethods.roundTo(((absEncoder.get() - Constants.RobotConstants.ENCODER_OFFSET) / 335) * 360, 0));

      //do if statement with 360 minus for negative numbers
      double newAngle = rawAngle - zeroOffset;

      if (newAngle < 0) {
        currentAngle = 360 + newAngle; //new angle is always negative so current angle = 360 - (a negative number)
      }
      else {
        currentAngle = newAngle;
      }
    }
  }

  public void updateSmartDashboard() {
    //override in module specific class
  }

  public void updateGearSpeeds() {
    topGear.set(TalonFXControlMode.PercentOutput, topGearSpeed);
    bottomGear.set(TalonFXControlMode.PercentOutput, bottomGearSpeed);
  }
}
