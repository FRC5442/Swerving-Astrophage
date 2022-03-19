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

  public static double rightX; //The variable to be accessed by "MoveCrabButton" Command with a + or - value

  private double leftX;//0
  private double leftY;

  double xValue;  //variable to store x translation value
  double yValue;  //variable to store y translation value
  double rValue;  //variable to store rotation value

  private boolean useAuto = false;

  public boolean logitechController = false;
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
        leftX = xFilter.calculate(driveStick.getRawAxis(1)*Constants.SwerveConstants.X_MULTIPLIER); //0
        leftY = yFilter.calculate(driveStick.getRawAxis(0)*Constants.SwerveConstants.Y_MULTIPLIER) ;  //1
        //rightX = rFilter.calculate(driveStick.getRawAxis(4)*Constants.SwerveConstants.R_MULTIPLIER);
        // leftX = driveStick.getRawAxis(1) * Constants.SwerveConstants.X_MULTIPLIER;
        // leftY = driveStick.getRawAxis(0) * Constants.SwerveConstants.Y_MULTIPLIER;
        rightX = driveStick.getRawAxis(4) * Constants.SwerveConstants.R_MULTIPLIER;

      //double rightX = driveStick.getRawAxis(2)*-1;   //.getRawAxis(4) for xboxController     //.getRawAxis(2) for logitech
       //.getRawAxis(4) for xboxController     //.getRawAxis(2) for logitech
      
      //Vector2d translation = new Vector2d(leftX * Math.pow(Math.abs(leftX), 1), leftY * Math.pow(Math.abs(leftY), 1));
      Vector2d translation = new Vector2d(leftX, leftY);
      RobotContainer.swerveGroup.moveSwerveWPILib(translation, rightX * Math.pow(Math.abs(rightX), 1));
      // RobotContainer.swerveGroup.moveSwerve(translation, rightX * Math.pow(Math.abs(rightX), 1));


    } else {
      //leftX = _leftX; //0
      //leftY = _leftY;  //1
      leftX = xValue;
      leftY = yValue;
      rightX = rValue;

      //double rightX = driveStick.getRawAxis(2)*-1;   //.getRawAxis(4) for xboxController     //.getRawAxis(2) for logitech
      //rightX = 0.0;   //.getRawAxis(4) for xboxController     //.getRawAxis(2) for logitech
      
      Vector2d translation = new Vector2d(leftX, leftY);
      RobotContainer.swerveGroup.moveSwerveWPILib(translation, rightX * Math.pow(Math.abs(rightX), 1));

    }

    /*
    if (rightX == 0){  //If buttons for crab rotation are not pressed...
      RobotContainer.swerveGroup.moveSwerve(translation, rightX * Math.pow(Math.abs(rightX), 1)); //move translation
    } else {   //Else, the buttons for crab rotation are pressed...
      RobotContainer.swerveGroup.moveSwerve(new Vector2d(0,0), rightX * Math.pow(Math.abs(rightX), 1));  //move crab
    }
    */
    
    //RobotContainer.swerveGroup.moveSwerve(translation, rightX * Math.pow(Math.abs(rightX), 1));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.swerveGroup.moveSwerveWPILib(new Vector2d(0, 0), 0);
    //rightX = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}






