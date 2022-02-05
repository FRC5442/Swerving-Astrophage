// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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
    // if (RobotContainer.hoodEncoder.getDistance() <= 0 && speed > 0){
    //   RobotContainer.hood.moveHood(0);
    // }else if (RobotContainer.hoodEncoder.getDistance() >= 10000 && speed < 0){
    //   RobotContainer.hood.moveHood(0);
    // } else {
    //   RobotContainer.hood.moveHood(speed);
    // }

    double error = target - RobotContainer.hoodEncoder.getDistance()/10000;
    double speed = -error/kP;
    if (speed >=0.2){
      speed = 0.2;
    } else if (speed <= -0.2){
      speed = -0.2;
    }
    SmartDashboard.putNumber("Hood PID Speed", error/kP);
    RobotContainer.hood.moveHood(speed);
    //RobotContainer.hood.moveHood(target);
    
    //hoodEncoderValue = (int) Math.round(RobotContainer.hoodEncoder.getDistance()/1000);
    
    //SmartDashboard.putNumber("Hood Encoder", hoodEncoderValue);
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
