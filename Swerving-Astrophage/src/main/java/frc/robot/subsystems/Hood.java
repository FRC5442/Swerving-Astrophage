// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Hood extends SubsystemBase {
  /** Creates a new Intake. */
  WPI_VictorSPX hoodMotor;

  public Hood() {
    hoodMotor = RobotContainer.hoodMotor;
  }

  public void moveHood(double speed) {
    hoodMotor.set(speed);
    //System.out.println(RobotContainer.hoodEncoder.getDistance());
    //SmartDashboard.putNumber("Encoder Test Value", RobotContainer.hoodEncoder.getDistance() / 60);
  }

  public void stopHood(){
    hoodMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Hood Encoder", Math.round(RobotContainer.hoodEncoder.getDistance() / 1000));

  }
}
