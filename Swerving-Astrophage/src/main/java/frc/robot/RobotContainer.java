// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/************************* IMPORTS *************************/
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.commands.ExampleCommand;
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
  public static Joystick xbox1;
  public static JoystickButton xbox1A, xbox1B, xbox1X, xbox1Y;
  public static JoystickButton xbox1LB, xbox1RB;

  public static Joystick xbox2;
  public static JoystickButton xbox2A, xbox2B, xbox2X, xbox2Y;
  public static JoystickButton xbox2LB, xbox2RB;



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

  //SHOOTER\\
  public static Shooter shooter;
  public static ShootCommand shootCommand, reverseShootCommand;
  public static CANSparkMax shooterWheel1;
  public static CANSparkMax shooterWheel2;
  public static WPI_VictorSPX shooterHood;

  //HOOD\\
  public static HoodCommand raiseHood, lowerHood;
  public static Encoder hoodEncoder;


  //INTAKE\\
  public static WPI_VictorSPX intakeMotorField;
  public static WPI_VictorSPX intakeMotorPivot;
  public static WPI_VictorSPX intakeMotorElevator1, intakeMotorElevator2;
  public static Intake intake;
  public static IntakeCommand intakeFieldCommand, reverseIntakeFieldCommand;
  public static IntakeCommand intakePivotCommand, reverseIntakePivotCommand;
  public static IntakeCommand intakeElevator1Command, reverseIntakeElevator1Command;
  public static IntakeCommand intakeElevator2Command, reverseIntakeElevator2Command;
  public static DigitalInput intakeLaserSwitch;


   //TURRET\\
  public static WPI_VictorSPX turretMotor;
  public static AHRS turretGyro;
  public static Turret turret;
  public static Encoder turretEncoder;

   //CLIMBER\\
  public static ClimberLeft climberLeft;
  public static ClimberRight climberRight;
  public static TalonFX winchLeft, winchRight;
  public static TalonFX pivotLeft, pivotRight;


 

  public RobotContainer() {

    /************************* JOYSTICKS *************************/
    xbox1 = new Joystick(0);
    xbox1A = new JoystickButton(xbox1, Button.kA.value);
    xbox1B = new JoystickButton(xbox1, Button.kB.value);
    xbox1X = new JoystickButton(xbox1, Button.kX.value);
    xbox1Y = new JoystickButton(xbox1, Button.kY.value);
    xbox1LB = new JoystickButton(xbox1, Button.kLeftBumper.value);
    xbox1RB = new JoystickButton(xbox1, Button.kRightBumper.value);

    xbox2 = new Joystick(1);
    xbox2A = new JoystickButton(xbox2, Button.kA.value);
    xbox2B = new JoystickButton(xbox2, Button.kB.value);
    xbox2X = new JoystickButton(xbox2, Button.kX.value);
    xbox2Y = new JoystickButton(xbox2, Button.kY.value);
    xbox2LB = new JoystickButton(xbox2, Button.kLeftBumper.value);
    xbox2RB = new JoystickButton(xbox2, Button.kRightBumper.value);
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

    /************************* SHOOTER *************************/
    shooter = new Shooter();
    shooterWheel1 = new CANSparkMax(Constants.ShooterConstants.SHOOTER_MOTOR_ONE, CANSparkMaxLowLevel.MotorType.kBrushless);
    shooterWheel2 = new CANSparkMax(Constants.ShooterConstants.SHOOTER_MOTOR_TWO, CANSparkMaxLowLevel.MotorType.kBrushless);

    shootCommand = new ShootCommand(Constants.ShooterConstants.SHOOTER_RPM);
    reverseShootCommand = new ShootCommand(-Constants.ShooterConstants.SHOOTER_RPM);
    /***********************************************************/

    /************************* HOOD *************************/
    shooterHood = new WPI_VictorSPX(Constants.ShooterConstants.SHOOTER_MOTOR_HOOD);
    raiseHood = new HoodCommand(Constants.ShooterConstants.HOOD_HIGH);
    lowerHood = new HoodCommand(Constants.ShooterConstants.HOOD_LOW);
    /***********************************************************/


    /************************* INTAKE *************************/
    intakeMotorField = new WPI_VictorSPX(Constants.IntakeConstants.INTAKE_MOTOR_FIELD);
    intakeMotorPivot = new WPI_VictorSPX(Constants.IntakeConstants.INTAKE_MOTOR_PIVOT);
    intakeMotorElevator1 = new WPI_VictorSPX(Constants.IntakeConstants.INTAKE_MOTOR_ELEVATOR_ONE);
    intakeMotorElevator2 = new WPI_VictorSPX(Constants.IntakeConstants.INTAKE_MOTOR_ELEVATOR_TWO);

    intake = new Intake();
    intakeFieldCommand = new IntakeCommand(Constants.IntakeConstants.INTAKE_FIELD_SPEED, intakeMotorField, false);
    reverseIntakeFieldCommand = new IntakeCommand(-Constants.IntakeConstants.INTAKE_FIELD_SPEED, intakeMotorField, false);

    intakePivotCommand = new IntakeCommand(Constants.IntakeConstants.INTAKE_PIVOT_SPEED, intakeMotorPivot, false);
    reverseIntakePivotCommand = new IntakeCommand(-Constants.IntakeConstants.INTAKE_PIVOT_SPEED, intakeMotorPivot, false);
    
    intakeElevator1Command = new IntakeCommand(Constants.IntakeConstants.INTAKE_ELEVATOR_SPEED, intakeMotorElevator1, false);
    reverseIntakeElevator1Command = new IntakeCommand(-Constants.IntakeConstants.INTAKE_ELEVATOR_SPEED, intakeMotorElevator1, false);
    intakeElevator2Command = new IntakeCommand(Constants.IntakeConstants.INTAKE_ELEVATOR_SPEED, intakeMotorElevator2, false);
    reverseIntakeElevator1Command = new IntakeCommand(-Constants.IntakeConstants.INTAKE_ELEVATOR_SPEED, intakeMotorElevator2, false);

    intakeLaserSwitch = new DigitalInput(Constants.IntakeConstants.INTAKE_LASER_SWITCH);
    /***********************************************************/


    /************************* CLIMBER *************************/
    winchLeft = new TalonFX(Constants.ClimberConstants.WINCH_LEFT);
    winchRight = new TalonFX(Constants.ClimberConstants.WINCH_RIGHT);
    pivotLeft = new TalonFX(Constants.ClimberConstants.PIVOT_LEFT);
    pivotRight = new TalonFX(Constants.ClimberConstants.PIVOT_RIGHT);
    climberLeft = new ClimberLeft(pivotLeft, winchLeft);
    climberRight = new ClimberRight(pivotRight, winchRight);

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
    // Primary driver controls: intake, drive(not initialized here), climb?
    xbox1LB.whileHeld(intakeFieldCommand);
    xbox1RB.whileHeld(reverseIntakeFieldCommand);
    //xboxControllerX.whenPressed(new DefaultAutoPath());


    // Second driver controls: shooter, turret, and hood
    // xbox2LB.whileHeld(TurretCommandLeft);
    // xbox2RB.whileHeld(TurretCommandRight); 
    xbox2B.whileHeld(shootCommand);
    xbox2X.whileHeld(reverseShootCommand);
    xbox2A.whileHeld(lowerHood);
    xbox2Y.whileHeld(raiseHood);


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
