// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  TalonFX pivotMotor, winchMotor;
  public Climber(TalonFX pivotMotor, TalonFX winchMotor) {
    this.winchMotor = winchMotor;
    this.pivotMotor = pivotMotor;
  }

  public void winchClimber(double speed){
    winchMotor.set(TalonFXControlMode.PercentOutput, speed);
  }

  public void pivotClimber(double speed){
    pivotMotor.set(TalonFXControlMode.PercentOutput, speed);
  }

  public void stopClimberWinch(){
    winchMotor.set(TalonFXControlMode.PercentOutput, 0);
  }
  public void stopClimberExtension(){
    pivotMotor.set(TalonFXControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
