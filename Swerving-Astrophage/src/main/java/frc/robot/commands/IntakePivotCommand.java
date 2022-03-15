// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class IntakePivotCommand extends CommandBase {
  /** Creates a new IntakePivotCommand. */
  double speed = 0.2;
  double returnSpeed = 0.4;
  double calculatedSpeed;
  boolean isFinished = false;
  double posTop = -500;
  double posBottom = 25000;
  double posTarget;
  double triggerValue;
  boolean useAuto = false;
  double height;

  double AUTO_TARGET_HEIGHT = 0.5;

  public IntakePivotCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.intake);
    useAuto = false;
  }
  public IntakePivotCommand(double height){
    this.height = height;
    useAuto = true;
    addRequirements(RobotContainer.intake);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!useAuto) triggerValue = RobotContainer.xbox2.getRawAxis(3);
    else triggerValue = height;
    posTarget = ((posBottom - posTop) * triggerValue) + posTop;
    SmartDashboard.putNumber("Intake Pivot Target", posTarget);

    if (RobotContainer.intakePivotEncoder.getDistance() > posTarget){ // Moving up
      calculatedSpeed = returnSpeed * 1 * ((RobotContainer.intakePivotEncoder.getDistance() - posTarget) / 5000);
    } else if (RobotContainer.intakePivotEncoder.getDistance() < posTarget){ // Moving down
      calculatedSpeed = speed * -1 * ((posTarget - RobotContainer.intakePivotEncoder.getDistance() / 20000));
    } else {
      calculatedSpeed = 0;
    }
    if (calculatedSpeed > returnSpeed) calculatedSpeed = returnSpeed;
    else if (calculatedSpeed < -speed) calculatedSpeed = -speed;

    RobotContainer.intake.moveIntakePivot(calculatedSpeed);


  //   if (Constants.IntakeConstants.USE_INTAKE_PIVOT_LIMITS){
  //     if (RobotContainer.intakePivotEncoder.getDistance() >= Constants.IntakeConstants.INTAKE_PIVOT_MAX_POS && speed > 0){
  //       speed = 0;
  //       isFinished = true;
  //     } else if (RobotContainer.intakePivotEncoder.getDistance() <= Constants.IntakeConstants.INTAKE_PIVOT_MIN_POS && speed < 0){
  //       speed = 0;
  //       isFinished = true;  
  //     } else {RobotContainer.intake.moveIntakePivot(speed);}
  //   } else {RobotContainer.intake.moveIntakePivot(speed);}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //RobotContainer.intake.moveIntakePivot(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
