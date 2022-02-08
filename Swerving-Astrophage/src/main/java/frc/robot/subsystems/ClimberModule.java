// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberModule extends SubsystemBase {
  /** Creates a new Climber. */
  TalonFX pivotMotor, winch;
  
  public ClimberModule(TalonFX pivotMotor, TalonFX winch) {
    this.pivotMotor = pivotMotor;
    this.winch = winch;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
