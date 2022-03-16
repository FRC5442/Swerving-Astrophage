// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.SharedMethods;

public class TurretCommand extends CommandBase {
  /** Creates a new TurretCommand. */
  int incrementValue;
  double startTime;
  double speed;
  double axisValue;
  double newSpeed;
  double LEFT_POS = 800;
  double RIGHT_POS = -800;
  boolean returnToCenter = false;

  public TurretCommand(double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.turret);
    this.speed = speed;
    returnToCenter = false;
  }
  public TurretCommand(){
    returnToCenter = true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // startTime = System.currentTimeMillis();  // Store the time at which the command is scheduled
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!returnToCenter){
      axisValue = RobotContainer.xbox2.getRawAxis(0);

      if (RobotContainer.turretEncoder.getDistance() >= LEFT_POS && axisValue > 0) newSpeed = 0;
      else if (RobotContainer.turretEncoder.getDistance() <= RIGHT_POS && axisValue < 0) newSpeed = 0;
      else newSpeed = speed;
  
      RobotContainer.turret.moveTurret(newSpeed * axisValue);
    } else {
      if (RobotContainer.turretEncoder.getDistance() >= 0) newSpeed = -0.6; 
      else if (RobotContainer.turretEncoder.getDistance() <= 0) newSpeed = 0.6;
      else newSpeed = 0;
      newSpeed = newSpeed * Math.abs(RobotContainer.turretEncoder.getDistance() / 100);
      //if (Math.abs(newSpeed) <= 0.2) newSpeed = 0.2 * SharedMethods.positiveOrNegative(newSpeed);
      RobotContainer.turret.moveTurret(newSpeed);
    }




    // double currentTime = (System.currentTimeMillis() - startTime) / 1000;  // determine the current time in seconds
    // Constants.TurretConstants.TURRET_ENCODER_OFFSET = Constants.TurretConstants.TURRET_ENCODER_OFFSET + (axisValue * currentTime);
    // // // This formula is the equation of a line, setting the output equal to the current offset + a value that will scale slightly every millisecond.
    // // // A positive axis value will result in an increase, and a negative value a decrease.

    // RobotContainer.turret.moveTurretToAngle(Constants.TurretConstants.TURRET_ENCODER_OFFSET);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.turret.moveTurret(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return (axisValue <= Constants.RobotConstants.TRIGGER_DEADZONE); // If the trigger falls below the threshlod, finish the command
    return false;
  }
}
