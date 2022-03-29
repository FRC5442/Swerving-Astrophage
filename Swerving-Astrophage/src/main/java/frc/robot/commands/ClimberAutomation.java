// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandGroupBase;
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
  static ClimberCommand moveLeftForward = new ClimberCommand(RobotContainer.pivotLeft, -Constants.ClimberConstants.PIVOT_FRONT_POSITION, 0, -Constants.ClimberConstants.PIVOT_SPEED);
  static ClimberCommand moveLeftBackward = new ClimberCommand(RobotContainer.pivotLeft, -Constants.ClimberConstants.PIVOT_FRONT_POSITION, 0, Constants.ClimberConstants.PIVOT_SPEED);

  static ClimberCommand moveRightForward = new ClimberCommand(RobotContainer.pivotRight, 0, Constants.ClimberConstants.PIVOT_FRONT_POSITION, Constants.ClimberConstants.PIVOT_SPEED);
  static ClimberCommand moveRightBackward = new ClimberCommand(RobotContainer.pivotRight, 0, Constants.ClimberConstants.PIVOT_FRONT_POSITION, -Constants.ClimberConstants.PIVOT_SPEED);

  static ClimberCommand raiseLeftWinch = new ClimberCommand(RobotContainer.winchLeft, -Constants.ClimberConstants.WINCH_HIGH_POSITION, 0, -Constants.ClimberConstants.WINCH_SPEED);
  static ClimberCommand lowerLeftWinch = new ClimberCommand(RobotContainer.winchLeft, -Constants.ClimberConstants.WINCH_HIGH_POSITION, 0, Constants.ClimberConstants.WINCH_SPEED);

  static ClimberCommand raiseRightWinch = new ClimberCommand(RobotContainer.winchRight, 0, Constants.ClimberConstants.WINCH_HIGH_POSITION, Constants.ClimberConstants.WINCH_SPEED);
  static ClimberCommand lowerRightWinch = new ClimberCommand(RobotContainer.winchRight, 0, Constants.ClimberConstants.WINCH_HIGH_POSITION, -Constants.ClimberConstants.WINCH_SPEED);
  
  
  public ClimberAutomation() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    /*
    Climbing Instructions:
    1. retract right winch
    2. extend left winch
    3. Pivot left arm to bar
    4. retract left winch
    5. before the left winch is fully retracted, begin extending right winch
    6. Retract right winch
    7. pivot right arm towards intake
    9. extend right winch
    9. pivot right arm towards bar
    10. retract right arm
    11. before the right arm is fully retracted, begin extending left winch
    */

    addCommands(

      new ParallelCommandGroup(
        new ClimberCommand(RobotContainer.winchRight, 0, Constants.ClimberConstants.WINCH_HIGH_POSITION, -Constants.ClimberConstants.WINCH_SPEED), // lowerRightWinch,
        new ClimberCommand(RobotContainer.winchLeft, -Constants.ClimberConstants.WINCH_HIGH_POSITION, 0, -Constants.ClimberConstants.WINCH_SPEED) // raiseLeftWinch
        ),
      new ClimberCommand(RobotContainer.pivotLeft, -Constants.ClimberConstants.PIVOT_FRONT_POSITION, 0 + (-Constants.ClimberConstants.PIVOT_FRONT_POSITION / 2), Constants.ClimberConstants.PIVOT_SPEED),
      
      new ParallelCommandGroup(
        new ClimberCommand(RobotContainer.winchLeft, -Constants.ClimberConstants.WINCH_HIGH_POSITION, 0, Constants.ClimberConstants.WINCH_SPEED), // lowerLeftWinch,
        new SequentialCommandGroup(new WaitCommand(1), new ClimberCommand(RobotContainer.winchRight, 0, Constants.ClimberConstants.WINCH_HIGH_POSITION, Constants.ClimberConstants.WINCH_SPEED)) // raise right winch
        ),

      new ClimberCommand(RobotContainer.winchRight, 0, Constants.ClimberConstants.WINCH_HIGH_POSITION, -Constants.ClimberConstants.WINCH_SPEED), // lowerRightWinch,,
      new ClimberCommand(RobotContainer.pivotRight, 0, Constants.ClimberConstants.PIVOT_FRONT_POSITION, Constants.ClimberConstants.PIVOT_SPEED), // moveRightForward,
      new ClimberCommand(RobotContainer.winchRight, 0, Constants.ClimberConstants.WINCH_HIGH_POSITION, Constants.ClimberConstants.WINCH_SPEED), // raiseRightWinch,
      new ClimberCommand(RobotContainer.pivotRight, 0 + (Constants.ClimberConstants.PIVOT_FRONT_POSITION / 3), Constants.ClimberConstants.PIVOT_FRONT_POSITION, -Constants.ClimberConstants.PIVOT_SPEED), // Pivot Right Backwards
      
      new ParallelCommandGroup(
        new ClimberCommand(RobotContainer.winchRight, 0, Constants.ClimberConstants.WINCH_HIGH_POSITION, -Constants.ClimberConstants.WINCH_SPEED), // lowerRightWinch,,
        new SequentialCommandGroup(new WaitCommand(1), new ClimberCommand(RobotContainer.winchLeft, -Constants.ClimberConstants.WINCH_HIGH_POSITION, 0, -Constants.ClimberConstants.WINCH_SPEED))
        ), // raise left winch

      new WaitCommand(10)
    );

    // addCommands(
    //   // Bring Both arms forward
    //   new ParallelCommandGroup(
    //     moveLeftForward,
    //     moveRightForward),

    //   // Extend both arms
    //   new ParallelCommandGroup(
    //     raiseLeftWinch,
    //     raiseRightWinch),

    //   // Pivot Right Arm to Bar
    //   //new ClimberCommand(RobotContainer.pivotRight, 0 + (Constants.ClimberConstants.PIVOT_FRONT_POSITION / 2), Constants.ClimberConstants.PIVOT_FRONT_POSITION, -Constants.ClimberConstants.PIVOT_SPEED),
    //   new ClimberCommand(RobotContainer.pivotLeft, -Constants.ClimberConstants.PIVOT_FRONT_POSITION, 0 + (-Constants.ClimberConstants.PIVOT_FRONT_POSITION / 2), Constants.ClimberConstants.PIVOT_SPEED),
    //   // lower right winch
    //   //lowerRightWinch,
    //   lowerLeftWinch,

    //   // Wait at the end to avoid restarting the process:
    //   new WaitCommand(10)
    // );
  }
}
