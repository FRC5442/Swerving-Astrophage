// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
    final ClimberCommand moveLeftForward = new ClimberCommand(RobotContainer.pivotLeft, -Constants.ClimberConstants.PIVOT_FRONT_POSITION, 0, -Constants.ClimberConstants.PIVOT_SPEED);
    final ClimberCommand moveLeftBackward = new ClimberCommand(RobotContainer.pivotLeft, -Constants.ClimberConstants.PIVOT_FRONT_POSITION, 0, Constants.ClimberConstants.PIVOT_SPEED);

    final ClimberCommand moveRightForward = new ClimberCommand(RobotContainer.pivotRight, 0, Constants.ClimberConstants.PIVOT_FRONT_POSITION, Constants.ClimberConstants.PIVOT_SPEED);
    final ClimberCommand moveRightBackward = new ClimberCommand(RobotContainer.pivotRight, 0, Constants.ClimberConstants.PIVOT_FRONT_POSITION, -Constants.ClimberConstants.PIVOT_SPEED);

    final ClimberCommand raiseLeftWinch = new ClimberCommand(RobotContainer.winchLeft, -Constants.ClimberConstants.WINCH_HIGH_POSITION, 0, -Constants.ClimberConstants.WINCH_SPEED);
    final ClimberCommand lowerLeftWinch = new ClimberCommand(RobotContainer.winchLeft, -Constants.ClimberConstants.WINCH_HIGH_POSITION, 0, Constants.ClimberConstants.WINCH_SPEED);

    final ClimberCommand raiseRightWinch = new ClimberCommand(RobotContainer.winchRight, 0, Constants.ClimberConstants.WINCH_HIGH_POSITION, Constants.ClimberConstants.WINCH_SPEED);
    final ClimberCommand lowerRightWinch = new ClimberCommand(RobotContainer.winchRight, 0, Constants.ClimberConstants.WINCH_HIGH_POSITION, -Constants.ClimberConstants.WINCH_SPEED);






    addCommands(
      // Bring Both arms forward
      new ParallelCommandGroup(
        moveLeftForward,
        moveRightForward),

      // Extend both arms
      new ParallelCommandGroup(
        raiseLeftWinch,
        raiseRightWinch),

      // Pivot Right Arm to Bar
      new ClimberCommand(RobotContainer.pivotRight, 0 + (Constants.ClimberConstants.PIVOT_FRONT_POSITION / 2), Constants.ClimberConstants.PIVOT_FRONT_POSITION, -Constants.ClimberConstants.PIVOT_SPEED),
        
      // lower right winch
      lowerRightWinch,

      // Wait at the end to avoid restarting the process:
      new WaitCommand(10)
    );
  }
}
