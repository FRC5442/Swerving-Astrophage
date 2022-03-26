// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Drive extends CommandBase {
  double xValue;  //variable to store x translation value
  double yValue;  //variable to store y translation value
  double rValue;  //variable to store rotation value
  double leftX;
  double leftY;
  double rightX;

  private boolean useAuto = false;
  SlewRateLimiter xFilter = new SlewRateLimiter(3);
  SlewRateLimiter yFilter = new SlewRateLimiter(3);
  SlewRateLimiter rFilter = new SlewRateLimiter(0);

  public Drive() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.swerveGroup);
    useAuto = false;
  }

  // auto contructor
  public Drive(double _leftX, double _leftY, double _rightX) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.swerveGroup);
    useAuto = true; //alterd, normally true
    xValue = _leftX;
    yValue = _leftY;
    rValue = _rightX;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    if(!useAuto){
        Joystick driveStick = RobotContainer.xbox1;
        leftX = xFilter.calculate(driveStick.getRawAxis(1)); //0
        leftY = yFilter.calculate(driveStick.getRawAxis(0));  //1
        rightX = driveStick.getRawAxis(4);

      RobotContainer.swerveGroup.drive(leftX, leftY, rightX, true);
    } else {
      //leftX = _leftX; //0
      //leftY = _leftY;  //1
      leftX = xValue;
      leftY = yValue;
      rightX = rValue;      
      RobotContainer.swerveGroup.drive(leftX, leftY, rightX, false);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.swerveGroup.stopSwerve();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}






