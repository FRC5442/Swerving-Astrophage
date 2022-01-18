// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */
  WPI_VictorSPX turretMotor;
  public Turret(double speed) {
    turretMotor = RobotContainer.turretMotor;
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
