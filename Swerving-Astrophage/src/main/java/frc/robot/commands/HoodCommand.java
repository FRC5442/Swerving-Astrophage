// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class HoodCommand extends CommandBase {
  /** Creates a new HoodCommand. */
  double target;
  int hoodEncoderValue;
  double kP = 2.5;

  public HoodCommand(double target) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.hood);
    this.target = target;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double error = target - RobotContainer.hoodEncoder.getDistance()/10000;  // Calculate the distance from the target
    double speed = -error/kP;  // Divide the distance by a constant to get a speed value

    // Control loop to keep speed from exceding a max setpoint
    if (speed >=0.15){
      speed = 0.15;
    } else if (speed <= -0.15){
      speed = -0.15;
    }

    // Rumble Feature
    if (speed <= 0.1 && speed > 0.01){
      RobotContainer.xboxController.setRumble(RumbleType.kLeftRumble, 1);
    } else if (speed >= -0.1 && speed < -0.01){
      RobotContainer.xboxController.setRumble(RumbleType.kRightRumble, 1);
    }

    SmartDashboard.putNumber("Hood PID Speed", speed);
    RobotContainer.hood.moveHood(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.hood.moveHood(0);
    RobotContainer.xboxController.setRumble(RumbleType.kRightRumble, 0);
    RobotContainer.xboxController.setRumble(RumbleType.kLeftRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
