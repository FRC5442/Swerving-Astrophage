// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  WPI_VictorSPX intakeMotor;
  WPI_VictorSPX elevatorMotr;

  public Intake() {
    intakeMotor = RobotContainer.intakeMotor;
    elevatorMotr = RobotContainer.elevatorMotor;
  }

  public void moveIntake(double speed) {
    intakeMotor.set(speed);
    elevatorMotr.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
