// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  TalonFX winchRight, winchLeft, pivotRight, pivotLeft;

  public Climber() {  
    winchRight = RobotContainer.winchRight;
    winchLeft = RobotContainer.winchLeft;
    pivotRight = RobotContainer.pivotRight;
    pivotLeft = RobotContainer.pivotLeft;
    
  }

  public void moveClimber(TalonFX winchMotor, double speed){
    winchMotor.set(TalonFXControlMode.PercentOutput, speed);

  }

  public void stopClimber(TalonFX motor){
    motor.set(TalonFXControlMode.PercentOutput, 0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Winch Right Encoder", winchRight.getSelectedSensorPosition());
    SmartDashboard.putNumber("Winch Left Encoder", winchLeft.getSelectedSensorPosition());
    SmartDashboard.putNumber("Pivot Left Encoder", pivotLeft.getSelectedSensorPosition());
    SmartDashboard.putNumber("Pivot Right Encoder", pivotRight.getSelectedSensorPosition());
  }
}
