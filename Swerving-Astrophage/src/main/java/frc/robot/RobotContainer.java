// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/************************* IMPORTS *************************/
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
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
  /************************* Variables *************************/
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  public static AHRS navX;
  public static PowerDistribution pdp;

  //AUTONOMOUS\\
  private final DefaultAutoPath autoDefault;
  private final ComplexAutoPath autoComplex;
  SendableChooser<Command> autoChooser;

  //CONTROLLER\\
  public static Joystick xboxController;
  public static JoystickButton xboxControllerA;
  public static JoystickButton xboxControllerB;
  public static JoystickButton xboxControllerX;
  public static JoystickButton xboxControllerStart;
  public static JoystickButton xboxControllerLBumper, xboxControllerRBumper;

  


  //DRIVE\\
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


  //HOOD\\
  /***********************************************************/
  public static WPI_VictorSPX hoodMotor;
  public static Hood hood;
  public static HoodCommand hoodCommand;
  public static HoodCommand reverseHoodCommand;
  public static Encoder hoodEncoder;
 /***********************************************************/

  public RobotContainer() {

    /************************* JOYSTICKS *************************/
    xboxController = new Joystick(0);
    xboxControllerA = new JoystickButton(xboxController, 1);
    xboxControllerB = new JoystickButton(xboxController, 2);
    xboxControllerX = new JoystickButton(xboxController, 3);
    xboxControllerStart = new JoystickButton(xboxController, 8);
    xboxControllerLBumper = new JoystickButton(xboxController, 5);
    xboxControllerRBumper = new JoystickButton(xboxController, 6);
    /***********************************************************/



    /************************* DRIVE *************************/
    driveMotor1 = new TalonFX(1);
    driveMotor2 = new TalonFX(2);
    driveMotor3 = new TalonFX(3);
    driveMotor4 = new TalonFX(4);
    driveMotor5 = new TalonFX(5);
    driveMotor6 = new TalonFX(6);
    driveMotor7 = new TalonFX(7);
    driveMotor8 = new TalonFX(8);

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
    /***********************************************************/

    

    /************************* HOOD *************************/
    hoodMotor = new WPI_VictorSPX(9);
    hood = new Hood();
    hoodCommand = new HoodCommand(0.1);
    reverseHoodCommand = new HoodCommand(-0.1);
    hoodEncoder = new Encoder(0, 1, true, Encoder.EncodingType.k4X);
    hoodEncoder.setDistancePerPulse(1/360);

    /***********************************************************/

   






    /************************* OTHER *************************/
    navX = new AHRS(SerialPort.Port.kMXP);
    calibrateGyro = new CalibrateGyro();
    calibrateModules = new CalibrateModules();
    /***********************************************************/

    /************************* AUTO *************************/
    autoDefault = new DefaultAutoPath();
    autoComplex = new ComplexAutoPath();
    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("Default Auto", autoDefault);
    autoChooser.addOption("Complex Auto", autoComplex);
    SmartDashboard.putData(autoChooser);
    /***********************************************************/



    /************************* BUTTON BINDING METHOD(S) *************************/
    configureButtonBindings();
    /***********************************************************/
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    xboxControllerLBumper.whileHeld(hoodCommand);
    xboxControllerRBumper.whileHeld(reverseHoodCommand);
    xboxControllerStart.whenPressed(calibrateGyro);
    

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return new DefaultAutoPath();
    return m_autoCommand;
    //return autoChooser.getSelected();
  }
}
