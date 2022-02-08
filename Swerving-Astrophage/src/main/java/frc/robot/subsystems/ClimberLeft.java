// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class ClimberLeft extends ClimberModule {
  /** Creates a new ClimberLeft. */
  public ClimberLeft(TalonFX pivotMotor, TalonFX winch) {
    super(pivotMotor, winch);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}