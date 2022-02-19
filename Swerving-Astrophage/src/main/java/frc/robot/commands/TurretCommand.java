// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class TurretCommand extends CommandBase {
  /** Creates a new TurretCommand. */
  int incrementValue;
  double startTime;
  double axisValue;

  public TurretCommand(double axisValue) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.turret);
    this.axisValue = axisValue;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();  // Store the time at which the command is scheduled
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentTime = (System.currentTimeMillis() - startTime) / 1000;  // determine the current time in seconds
    Constants.TurretConstants.TURRET_GYRO_OFFSET = Constants.TurretConstants.TURRET_GYRO_OFFSET + (axisValue * currentTime);
    // This formula is the equation of a line, setting the output equal to the current offset + a value that will scale slightly every millisecond.
    // A positive axis value will result in an increase, and a negative value a decrease.

    RobotContainer.turret.moveTurretToAngle(Constants.TurretConstants.TURRET_GYRO_OFFSET);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //RobotContainer.turret.moveTurret(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (axisValue <= Constants.RobotConstants.TRIGGER_DEADZONE); // If the trigger falls below the threshlod, finish the command
  }
}
