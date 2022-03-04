// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class IntakeCommand extends CommandBase {
  /** Creates a new IntakeCommand. */
  double speed;
  WPI_VictorSPX motor;
  boolean useLimitSwitch;
  boolean limitSwitchEnabledOnStart;
  public IntakeCommand(double speed, WPI_VictorSPX motor, boolean useLimitSwitch) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.intake);
    this.speed = speed;
    this.motor = motor;
    this.useLimitSwitch = useLimitSwitch;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (RobotContainer.intakeLaserSwitch.get() == true){
      limitSwitchEnabledOnStart = true;
    } else {
      limitSwitchEnabledOnStart = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (useLimitSwitch && RobotContainer.intakeLaserSwitch.get()){
      RobotContainer.intake.moveIntake(motor, 0);
    } else {
      RobotContainer.intake.moveIntake(motor, speed);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intake.moveIntake(motor, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // If there is a ball in the elevator and it didn't start there, stop the motor.
    if (motor == RobotContainer.intakeMotorElevator2 && RobotContainer.intakeLaserSwitch.get() == true && limitSwitchEnabledOnStart == false){
      return true;
    } else {
      return false;
    }
  }
}
