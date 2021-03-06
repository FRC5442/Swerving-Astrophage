// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  //WPI_VictorSPX intakeMotorField;
  WPI_VictorSPX intakeMotorPivot;
  WPI_VictorSPX intakeMotorElevator1, intakeMotorElevator2;

  public Intake() {
    //intakeMotorField = RobotContainer.intakeMotorField;
    intakeMotorPivot = RobotContainer.intakeMotorPivot;
    intakeMotorElevator1 = RobotContainer.intakeMotorElevator1;
    intakeMotorElevator2 = RobotContainer.intakeMotorElevator2;
  }

  public void moveIntake(double speed){
    // intakeMotorField.set(speed);
    intakeMotorElevator1.set(speed * Constants.IntakeConstants.INTAKE_FIELD_SPEED);
    intakeMotorElevator2.set(speed * Constants.IntakeConstants.INTAKE_ELEVATOR_SPEED);
  }

  public void moveIntakePivot(double speed) {   //A genearic method to set the speed of any parsed intake motor to the parsed speed
    intakeMotorPivot.set(speed);
  }

  public void moveIntakeElevator1(double speed){
    intakeMotorElevator1.set(speed);
  }

  public void moveIntakeElevator2(double speed){
    intakeMotorElevator2.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putBoolean("Intake Laser Switch", RobotContainer.intakeLaserSwitch.get());
    SmartDashboard.putNumber("Intake Color Sensor", RobotContainer.intakeColorSensor.getVoltage());
    SmartDashboard.putNumber("Intake Pivot Encoder", RobotContainer.intakePivotEncoder.getDistance());
  }
}
