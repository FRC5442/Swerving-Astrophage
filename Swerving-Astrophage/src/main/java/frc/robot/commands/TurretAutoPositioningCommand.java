// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class TurretAutoPositioningCommand extends CommandBase {
  /** Creates a new TurretAutoPositioningCommand. */
  public TurretAutoPositioningCommand() {
    addRequirements(RobotContainer.turret);
  } 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (RobotContainer.xbox1LTrigger >= Constants.RobotConstants.TRIGGER_DEADZONE){  // If the left trigger is pressed, 
      RobotContainer.turretMoveLeftCommand.schedule();  // Schedule the turret to move left
    } else if (RobotContainer.xbox1LTrigger >= Constants.RobotConstants.TRIGGER_DEADZONE){    // else see if right trigger is pressed
      RobotContainer.turretMoveRightCommand.schedule(); // schedule the turret to move right
    } else {
      RobotContainer.turret.moveTurretToAngle(Constants.TurretConstants.TURRET_GYRO_OFFSET);  // if none of the above are true, just update the turret position based on the gyroscope.
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
