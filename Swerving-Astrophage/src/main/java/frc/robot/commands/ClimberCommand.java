// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class ClimberCommand extends CommandBase {
  /** Creates a new ClimberCommand. */
  TalonFX motor;
  double minPosition;
  double maxPosition;
  double speed;
  boolean isFinished = false;

  public ClimberCommand(TalonFX motor, double minPosition, double maxPosition, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    // recieve which climber to move and what angle it should be at
    this.motor = motor;
    this.minPosition = minPosition;
    this.maxPosition = maxPosition;
    this.speed = speed;


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if (RobotContainer.climber.useClimberLimits){
        if (motor.getSelectedSensorPosition() >= maxPosition && speed > 0){
          RobotContainer.climber.moveClimber(motor, 0);
          isFinished = true;
        } else if (motor.getSelectedSensorPosition() <= minPosition && speed < 0){
          RobotContainer.climber.moveClimber(motor, 0);
          isFinished = true;  
        } else {RobotContainer.climber.moveClimber(motor, speed);}
      } else {RobotContainer.climber.moveClimber(motor, (speed * 0.75));}
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.climber.stopClimber(motor);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return false;
    // finish when climber is at desired position
    return (isFinished);
  }
}
