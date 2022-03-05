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
  double maxSpeed;
  double target;
  double direction;

  public ClimberCommand(TalonFX motor, double maxSpeed, double target, double direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    // recieve which climber to move and what angle it should be at
    this.motor = motor;
    this.maxSpeed = maxSpeed;
    this.target = target;
    this.direction = direction;


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // test if climber is at desired angle using encoder
    // move motor at speed towards target
    
    double error = Math.abs(motor.getSelectedSensorPosition()) - target;  // Calculate the distance from the target
    double speed = Math.abs(error/Constants.ClimberConstants.CLIMBER_kP) * direction;  // Divide the distance by a constant to get a speed value
    // Control loop to keep speed from exceding a max setpoint
    SmartDashboard.putNumber("Climber Error", error);
    SmartDashboard.putNumber("Climber speed", speed);
    if (speed >= maxSpeed){
      speed = maxSpeed;
    } else if (speed <= -maxSpeed){
      speed = -maxSpeed;
    }

    RobotContainer.climber.winchClimber(motor, speed);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.climber.stopClimber(motor);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    // finish when climber is at desired position
  }
}
