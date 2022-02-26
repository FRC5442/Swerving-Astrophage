// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  WPI_VictorSPX intakeMotorField;
  WPI_VictorSPX intakeMotorPivot;
  WPI_VictorSPX intakeMotorElevator1, intakeMotorElevator2;

  public Intake() {
    intakeMotorField = RobotContainer.intakeMotorField;
    intakeMotorPivot = RobotContainer.intakeMotorPivot;
    intakeMotorElevator1 = RobotContainer.intakeMotorElevator1;
    intakeMotorElevator2 = RobotContainer.intakeMotorElevator2;
  }

  public void moveIntake(double speed, WPI_VictorSPX motor) {   //A genearic method to set the speed of any parsed intake motor to the parsed speed
    motor.set(speed);
  }   //Not confident about the conventions relating to doing this, not sure if it will work.

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
