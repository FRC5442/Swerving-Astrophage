// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class HoodCommand extends CommandBase {
  /** Creates a new HoodCommand. */
  double target;
  public HoodCommand(double target) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooter);
    this.target = target;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double error = target - RobotContainer.hoodEncoder.getDistance()/10000;  // Calculate the distance from the target
    // double speed = -error/Constants.ShooterConstants.HOOD_kP;  // Divide the distance by a constant to get a speed value

    // // Control loop to keep speed from exceding a max setpoint
    // if (speed >=0.15){
    //   speed = 0.15;
    // } else if (speed <= -0.15){
    //   speed = -0.15;
    // }

    // SmartDashboard.putNumber("Hood PID Speed", speed);
    // RobotContainer.shooter.moveHood(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // RobotContainer.shooter.moveHood(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
