// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class HoodCommand extends CommandBase {
  /** Creates a new HoodCommand. */
  double speed;
  public HoodCommand(double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.hood);
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (RobotContainer.hoodEncoder.getDistance() < 40 && speed > 0){
    //   RobotContainer.hood.moveHood(0);
    // }else if (RobotContainer.hoodEncoder.getDistance() > 340 && speed < 0){
    //   RobotContainer.hood.moveHood(0);
    // } else {
    //   RobotContainer.hood.moveHood(speed);
    // }
    RobotContainer.hood.moveHood(speed);
    
    int hoodEncoderValue = (int) Math.round(RobotContainer.hoodEncoder.getDistance() / 20);
    
    SmartDashboard.putNumber("Hood Encoder", hoodEncoderValue);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.hood.moveHood(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
