// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class TurretCommand extends CommandBase {
  /** Creates a new TurretCommand. */
  int incrementValue;
  double startTime;

  public TurretCommand(int incrementValue) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.turret);
    this.incrementValue = incrementValue;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentTime = System.currentTimeMillis();

    // Control loop to increment TURRET_GYRO_OFFSET every constant milliseconds while button is pressed
    if (currentTime == startTime + Constants.TurretConstants.INCREMENT_MILLIS){
      Constants.TurretConstants.TURRET_GYRO_OFFSET = Constants.TurretConstants.TURRET_GYRO_OFFSET + incrementValue;
      startTime = currentTime;
    }

    RobotContainer.turret.moveTurretToAngle(Constants.TurretConstants.TURRET_GYRO_OFFSET);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.turret.moveTurret(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
