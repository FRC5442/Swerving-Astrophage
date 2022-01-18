// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.RobotContainer;

public class Turret extends PIDSubsystem {
  /** Creates a new Turret. */
  WPI_VictorSPX turretMotor = RobotContainer.turretMotor;
  AHRS turretGyro = RobotContainer.turretGyro;
  public double variableSetpoint = 0;

  public Turret() {
    super(new PIDController(0, 0, 0));        // The PIDController used by the subsystem
    setSetpoint(variableSetpoint);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    turretMotor.set(output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return (turretGyro.getAngle() - m_controller.getSetpoint());
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }
}
