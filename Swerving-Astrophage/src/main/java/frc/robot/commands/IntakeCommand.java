// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class IntakeCommand extends CommandBase {
  /** Creates a new IntakeCommand. */
  double speed;
  WPI_VictorSPX motor;
  boolean useLimitSwitch;
  public IntakeCommand(double speed, WPI_VictorSPX motor, boolean useLimitSwitch) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.intake);
    this.speed = speed;
    this.motor = motor;
    this.useLimitSwitch = useLimitSwitch;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (useLimitSwitch && RobotContainer.intakeLaserSwitch.get()){
      RobotContainer.intake.moveIntake(0, motor);
    } else {
      RobotContainer.intake.moveIntake(speed, motor);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intake.moveIntake(0, motor);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
