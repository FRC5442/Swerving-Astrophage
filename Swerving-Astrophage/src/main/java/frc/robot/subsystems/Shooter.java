// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */

  PWMSparkMax shooterWheel1, shooterWheel2;
  // WPI_VictorSPX shooterHood;

  RelativeEncoder wheel1Encoder, wheel2Encoder;

  // SparkMaxPIDController wheel1PIDController, wheel2PIDController;
  
  public Shooter() {
    //Shooter Wheel 1
    shooterWheel1 = RobotContainer.shooterWheel1;
    shooterWheel2 = RobotContainer.shooterWheel2;
    // wheel1Encoder = RobotContainer.shooterWheel1.getEncoder();
  }

  public void shoot(double speed) {
    shooterWheel1.set(speed);
    shooterWheel2.set(-speed);
  }

  // public void moveHood(double speed) {
  //   shooterHood.set(speed);
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
