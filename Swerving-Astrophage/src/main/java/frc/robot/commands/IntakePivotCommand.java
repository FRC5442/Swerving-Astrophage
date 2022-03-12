// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class IntakePivotCommand extends CommandBase {
  /** Creates a new IntakePivotCommand. */
  double speed;
  boolean isFinished = false;
  public IntakePivotCommand(double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(RobotContainer.intake);
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Constants.IntakeConstants.USE_INTAKE_PIVOT_LIMITS){
      if (RobotContainer.intakePivotEncoder.getDistance() >= Constants.IntakeConstants.INTAKE_PIVOT_MAX_POS && speed > 0){
        speed = 0;
        isFinished = true;
      } else if (RobotContainer.intakePivotEncoder.getDistance() <= Constants.IntakeConstants.INTAKE_PIVOT_MIN_POS && speed < 0){
        speed = 0;
        isFinished = true;  
      } else {RobotContainer.intake.moveIntakePivot(speed);}
    } else {RobotContainer.intake.moveIntakePivot(speed);}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intake.moveIntakePivot(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
