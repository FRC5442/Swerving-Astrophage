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
    m_topGearController.calculate(getTopGearSpeed(), state.speedMetersPerSecond);

  // Calculate the turning motor output from the turning PID controller.
    final double bottomGearOutput =
    m_bottomGearController.calculate(getBottomGearPosition(), state.angle.getRadians());

  // Calculate the turning motor output from the turning PID controller.
    // topGearSpeed = topGearOutput * bottomGearOutput;
    // bottomGearSpeed = -topGearOutput * bottomGearOutput;

    // topGearSpeed = state.speedMetersPerSecond * state.angle.getRadians();
    // bottomGearSpeed = -state.speedMetersPerSecond * state.angle.getRadians();

    topGearSpeed = state.speedMetersPerSecond;
    bottomGearSpeed = state.angle.getRadians();
  }

  public void moveSwerveTheOldWay(double speed, double angle) {
    //By rotating both the top gear and the bottom gear at equal and opposite speeds, the wheel will drive in a straight direction.
    topGearSpeed = 0;
    bottomGearSpeed = 0;
    double error = currentAngle - (angle);
    double desiredAngle = currentAngle + angle;

    //angle = SharedMethods.convertDegreesToRadians(angle);

    topGearSpeed += (-speed * TRANSLATE_MOD);
    bottomGearSpeed += (speed * TRANSLATE_MOD);

    // topGearSpeed = ((speed * TRANSLATE_MOD) + (SharedMethods.convertDegreesToRadians(angle) * ROTATE_MOD)) / 1;
    // bottomGearSpeed = -((speed * TRANSLATE_MOD) + (SharedMethods.convertDegreesToRadians(angle) * ROTATE_MOD)) / 1;

  //  topGearSpeed = (speed * TRANSLATE_MOD);
  //  bottomGearSpeed = -(speed * TRANSLATE_MOD);


  //  bottomGearSpeed = -(SharedMethods.convertDegreesToRadians(angle) * ROTATE_MOD);
   // ROTATE_MOD = 0.3 - (((Math.abs(topGearSpeed) + Math.abs(bottomGearSpeed)) / 2) * 0.15);

    

    // if (angle > currentAngle) angle += currentAngle;
    // if (angle < currentAngle) angle -= currentAngle;
    if (angle <= -1) angle = angle + 360;
    if (Math.abs(currentAngle - angle) >= ERROR_BOUND && Math.abs(currentAngle - angle) <= 360 - ERROR_BOUND) {
      ROTATE_MOD = 0.3 - (((Math.abs(topGearSpeed) + Math.abs(bottomGearSpeed)) / 2) * 0.15);
      // if (desiredAngle > currentAngle){
      //   topGearSpeed = angle * (ROTATE_MOD / 150);
      //   bottomGearSpeed = angle * (ROTATE_MOD / 150);
      // } else {
      // topGearSpeed = error * (ROTATE_MOD / 150);
      // bottomGearSpeed = error * (ROTATE_MOD / 150);
      // }
      turnToAngle(angle);
    }

  }

  public void turnToAngle(double desiredAngle) {
    //get the error
    double error = 0;

    // error = currentAngle - desiredAngle;

    // topGearSpeed = Math.abs(currentAngle - desiredAngle) / (1500*ROTATE_MOD);
    // bottomGearSpeed = Math.abs(currentAngle - desiredAngle) / (1500*ROTATE_MOD);

    // if (desiredAngle < currentAngle){
    //   error = desiredAngle - currentAngle;
    //   topGearSpeed += (error) / (1500*ROTATE_MOD);
    //   bottomGearSpeed += (error) / (1500*ROTATE_MOD);
    // } else if (desiredAngle > currentAngle){
    //   error = currentAngle - desiredAngle;
    //   topGearSpeed -= (error) / (1500*ROTATE_MOD);
    //   bottomGearSpeed -= (error) / (1500*ROTATE_MOD);
    // }

    // if (error > 0){
    //   topGearSpeed += Math.abs(360 - error) / 150 * ROTATE_MOD;
    //   bottomGearSpeed += Math.abs(360 - error) / 150 * ROTATE_MOD;
    // } else if (error < 0){
    //   topGearSpeed += -Math.abs(error) / 150 * ROTATE_MOD;
    //   bottomGearSpeed += -Math.abs(error) / 150 * ROTATE_MOD;
    // }

    

    if (desiredAngle > currentAngle) {
      error = desiredAngle - currentAngle;
      if (error < 180) {
        //move D by increasing C
        topGearSpeed += Math.abs(error) / 150 * ROTATE_MOD;
        bottomGearSpeed += Math.abs(error) / 150 * ROTATE_MOD;
      }
      else if (error >= 180) {
        //move towards D by decreasing C
        topGearSpeed += -Math.abs(360 - error) / 150 * ROTATE_MOD;
        bottomGearSpeed += -Math.abs(360 - error) / 150 * ROTATE_MOD;
      }
    }
    else if (desiredAngle < currentAngle) {
      error = currentAngle - desiredAngle;
      if (error < 180) {
        //move towards D decreasing C
        topGearSpeed += -Math.abs(error) / 150 * ROTATE_MOD;
        bottomGearSpeed += -Math.abs(error) / 150 * ROTATE_MOD;
      }
      else if (error >= 180) {
        //move towards D by increasing C
        topGearSpeed += Math.abs(360 - error) / 150 * ROTATE_MOD;
        bottomGearSpeed += Math.abs(360 - error) / 150 * ROTATE_MOD;
      }
    }
  }

  public void calibrate() {
    zeroOffset = rawAngle;
  }

  public double getModuleSpeed(){
    double speed = ((topGear.getSelectedSensorVelocity() - bottomGear.getSelectedSensorVelocity()) / 2.0) 
    * (10.0 / 2048) * ((10  / 88.0) * (54 / 14.0) * (1 / 3.0)) * (4 * 0.0254 * Math.PI * 1.10);
    return speed;
  }

  public double getTopGearPosition(){
    return (topGear.getSelectedSensorPosition() / 2048) * 2 * Math.PI;
  }

  public double getBottomGearPosition(){
    return (bottomGear.getSelectedSensorPosition() / 2048) * 2 * Math.PI;
  }

  public double getTopGearSpeed(){
    return (topGear.getSelectedSensorVelocity() / 2048) * 2 * Math.PI;
  }

  public double getBottomGearSpeed(){
    return (bottomGear.getSelectedSensorVelocity() / 2048) * 2 * Math.PI;
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
