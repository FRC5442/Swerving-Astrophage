// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */
  WPI_VictorSPX turretMotor;
  Encoder turretEncoder;
  AHRS robotGyro;
  double gyroOffset;

  public Turret() {
    turretMotor = RobotContainer.turretMotor;
    turretEncoder = RobotContainer.turretEncoder;
    robotGyro = RobotContainer.navX;
  }

  public void moveTurret(double speed){
    turretMotor.set(speed);
  }

  public void moveTurretToAngle(double desiredAngle){
    double currentTurretAngle = robotGyro.getAngle() + turretEncoder.get();
    double error = desiredAngle - currentTurretAngle;  // calculate the distance between the current angle and the desired angle
    double speed = -error/Constants.TurretConstants.TURRET_kP;  // set the speed proportional to the value of the error

    // Control loop to keep the speed from exceding a certain value
    if (speed >= Constants.TurretConstants.MAX_SPEED){
      speed = Constants.TurretConstants.MAX_SPEED;
    } else if (speed <= -Constants.TurretConstants.MAX_SPEED){
      speed = -Constants.TurretConstants.MAX_SPEED;
    }

    // Control loop to stop speed signal below a specified threshold
    if (Constants.TurretConstants.MIN_SPEED >= speed && speed >= -Constants.TurretConstants.MIN_SPEED){
      speed = 0;
    }

    moveTurret(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler 
    SmartDashboard.putNumber("Turret Encoder", RobotContainer.turretEncoder.getDistance());
  }
}
