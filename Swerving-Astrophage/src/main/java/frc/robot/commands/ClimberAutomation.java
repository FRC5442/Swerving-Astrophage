// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimberAutomation extends SequentialCommandGroup {
  /** Creates a new ClimberAutomation. */
  public ClimberAutomation() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // ClimberCommand pivotLeft = RobotContainer.pivotLeftCommand;
    // ClimberCommand reversePivotLeft = RobotContainer.reversePivotLeftCommand;
    addCommands(
      new ClimberCommand(RobotContainer.pivotLeft, Constants.ClimberConstants.PIVOT_REAR_POSITION, Constants.ClimberConstants.PIVOT_FRONT_POSITION, Constants.ClimberConstants.PIVOT_SPEED),
      new ClimberCommand(RobotContainer.pivotLeft, Constants.ClimberConstants.PIVOT_REAR_POSITION, Constants.ClimberConstants.PIVOT_FRONT_POSITION, -Constants.ClimberConstants.PIVOT_SPEED)
    );
  }
}
