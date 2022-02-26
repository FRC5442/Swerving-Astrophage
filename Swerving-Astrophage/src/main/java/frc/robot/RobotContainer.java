// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/************************* IMPORTS *************************/
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.subsystems.*;
import frc.robot.commands.*;
/***********************************************************/

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  /************************* VARIABLES *************************/
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  public static AHRS navX;
  public static PowerDistribution pdp;

  // CONTROLLER \\
  public static Joystick xboxController;
  public static JoystickButton xboxControllerA;
  public static JoystickButton xboxControllerB;
  public static JoystickButton xboxControllerX;
  public static JoystickButton xboxControllerStart;

  public static Joystick xboxController2;
  public static JoystickButton xbox2A, xbox2B, xbox2X, xbox2Y;


  // DRIVE \\
  public static TalonFX driveMotor1, driveMotor2; //front right
  public static TalonFX driveMotor3, driveMotor4; //front left
  public static TalonFX driveMotor5, driveMotor6; //back left
  public static TalonFX driveMotor7, driveMotor8; //back right

  public static AnalogPotentiometer frontRightAbsEncoder;
  public static AnalogPotentiometer frontLeftAbsEncoder;
  public static AnalogPotentiometer backRightAbsEncoder;
  public static AnalogPotentiometer backLeftAbsEncoder;
  public static SwerveGroup swerveGroup;
  public static SwerveModule frontRightModule, frontLeftModule, backLeftModule, backRightModule;
  public static Drive drive;
  public static CalibrateGyro calibrateGyro;
  public static CalibrateModules calibrateModules;
  public static MoveSingleSwerve moveSingleSwerveFrontLeft, moveSingleSwerveFrontRight, moveSingleSwerveBackLeft, moveSingleSwerveBackRight;


  // INTAKE \\
  public static WPI_VictorSPX intakeMotor;
  public static WPI_VictorSPX elevatorMotor;
  public static Intake intake;
  public static IntakeCommand intakeCommand;
  public static IntakeCommand reverseIntakeCommand;
  /************************* VARIABLES *************************/

  public RobotContainer() {

    /************************* JOYSTICKS *************************/
    // CONTROLLER 1 \\
    xboxController = new Joystick(0);
    xboxControllerA = new JoystickButton(xboxController, 1);
    xboxControllerB = new JoystickButton(xboxController, 2);
    xboxControllerX = new JoystickButton(xboxController, 3);
    xboxControllerStart = new JoystickButton(xboxController, 8);

    // CONTROLLER 2 \\
    xboxController2 = new Joystick(2);
    xbox2A = new JoystickButton(xboxController2, 1);
    xbox2B = new JoystickButton(xboxController2, 2);
    xbox2X = new JoystickButton(xboxController2, 3);
    xbox2Y = new JoystickButton(xboxController2, 4);
    /************************* JOYSTICKS *************************/



    /************************* DRIVE *************************/
    // DRIVE MOTORS \\
    driveMotor1 = new TalonFX(1);
    driveMotor2 = new TalonFX(2);
    driveMotor3 = new TalonFX(3);
    driveMotor4 = new TalonFX(4);
    driveMotor5 = new TalonFX(5);
    driveMotor6 = new TalonFX(6);
    driveMotor7 = new TalonFX(7);
    driveMotor8 = new TalonFX(8);

    // ENCODERS \\
    frontRightAbsEncoder = new AnalogPotentiometer(0, 360, 0);
    frontLeftAbsEncoder = new AnalogPotentiometer(1, 360, 0);
    backLeftAbsEncoder = new AnalogPotentiometer(3, 360, 0);
    backRightAbsEncoder = new AnalogPotentiometer(2, 360, 0);

    frontRightModule = new FrontRightModule(driveMotor1, driveMotor2, frontRightAbsEncoder);
    frontLeftModule = new FrontLeftModule(driveMotor3, driveMotor4, frontLeftAbsEncoder);
    backLeftModule = new BackLeftModule(driveMotor5, driveMotor6, backLeftAbsEncoder);
    backRightModule = new BackRightModule(driveMotor7, driveMotor8, backRightAbsEncoder);
    swerveGroup = new SwerveGroup();
    drive = new Drive();

    // MOVE SINGLE SWERVE \\
    moveSingleSwerveFrontLeft = new MoveSingleSwerve(frontLeftModule, 5);
    moveSingleSwerveFrontRight = new MoveSingleSwerve(frontRightModule, 5);
    moveSingleSwerveBackLeft = new MoveSingleSwerve(backLeftModule, 5);
    moveSingleSwerveBackRight = new MoveSingleSwerve(backRightModule, 5);
    /************************* DRIVE *************************/



    /************************* INTAKE *************************/
    intakeMotor = new WPI_VictorSPX(15);
    elevatorMotor = new WPI_VictorSPX(16);
    intake = new Intake();
    intakeCommand = new IntakeCommand(0.5);
    reverseIntakeCommand = new IntakeCommand(-0.5);
    /************************* INTAKE *************************/


    /************************* OTHER *************************/
    navX = new AHRS(SerialPort.Port.kMXP);
    calibrateGyro = new CalibrateGyro();
    calibrateModules = new CalibrateModules();
    /************************* OTHER *************************/


    /************************* BUTTON BINDING METHOD(S) *************************/
    configureButtonBindings();
    /************************* BUTTON BINDING METHOD(S) *************************/
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    xboxControllerB.whileHeld(intakeCommand);
    xboxControllerX.whileHeld(reverseIntakeCommand);
    xboxControllerStart.whenPressed(calibrateModules);

    // switch controller input to 3rd slot to move swerves individually
    // xbox2A.whileHeld(moveSingleSwerveBackLeft);
    // xbox2B.whileHeld(moveSingleSwerveBackRight);
    // xbox2X.whileHeld(moveSingleSwerveFrontRight);
    // xbox2Y.whileHeld(moveSingleSwerveFrontLeft);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
