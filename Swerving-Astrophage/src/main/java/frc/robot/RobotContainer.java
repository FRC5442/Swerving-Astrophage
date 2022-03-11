// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/************************* IMPORTS *************************/
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
// import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import frc.robot.subsystems.*;
import frc.robot.commands.*;
/************************* IMPORTS *************************/

public class RobotContainer {

/************************* VARIABLES *************************/
private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  public static AHRS navX;
  public static PowerDistribution pdp;
  public static ClimberAutomation climberAutomation;

  // AUTO \\
  //private final ComplexAutoPath autoComplex;
  SendableChooser<Command> autoChooser;

  // CONTROLLER \\
  public static Joystick xbox1;
  public static JoystickButton xbox1A, xbox1B, xbox1X, xbox1Y, xbox1Back, xbox1Start;
  public static JoystickButton xbox1LB, xbox1RB;
  public static double xbox1LTrigger, xbox1RTrigger;

  public static Joystick xbox2;
  public static JoystickButton xbox2A, xbox2B, xbox2X, xbox2Y;
  public static JoystickButton xbox2LB, xbox2RB;

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
  public static InstantCommand calibrateGyro;
  public static InstantCommand calibrateModules;
  public static InstantCommand calibrateClimbers;

  // SHOOTER \\
  public static Shooter shooter;
  public static ShootCommand shootCommand, reverseShootCommand;
  public static CANSparkMax shooterWheel1, shooterWheel2;
  // public static WPI_VictorSPX shooterHood;

  // HOOD \\
  public static HoodCommand raiseHood, lowerHood;
  public static Encoder hoodEncoder;

  // INTAKE \\
  public static WPI_VictorSPX intakeMotorField;
  public static WPI_VictorSPX intakeMotorPivot;
  public static WPI_VictorSPX intakeMotorElevator1, intakeMotorElevator2;
  public static Intake intake;
  public static StartEndCommand intakeFieldCommand, reverseIntakeFieldCommand;
  public static StartEndCommand intakePivotCommand, reverseIntakePivotCommand;
  public static StartEndCommand intakeElevator1Command, reverseIntakeElevator1Command;
  public static StartEndCommand intakeElevator2Command, reverseIntakeElevator2Command;
  public static StartEndCommand intakeAllCommand, reverseIntakeAllCommand;
  
  public static DigitalInput intakeLaserSwitch;
  public static AnalogInput intakeColorSensor;

  // TURRET \\
  public static WPI_VictorSPX turretMotor;
  public static AHRS turretGyro;
  public static Turret turret;
  public static AnalogPotentiometer turretEncoder;
  public static TurretAutoPositioningCommand turretAutoPositioningCommand;
  public static TurretCommand turretMoveLeftCommand, turretMoveRightCommand;
  public static StartEndCommand turretRight, turretLeft;

  // CLIMBER \\
  public static TalonFX winchLeft, winchRight;
  public static TalonFX pivotLeft, pivotRight;
  public static Climber climber;
  public static ClimberCommand winchLeftCommand, winchRightCommand, pivotLeftCommand, pivotRightCommand;
  public static ClimberCommand lowerWinchLeftCommand, lowerWinchRightCommand, reversePivotLeftCommand, reversePivotRightCommand;
  public static ClimberCommand auto_winchLeftCommand, auto_winchRightCommand, auto_pivotLeftCommand, auto_pivotRightCommand;
  public static ClimberCommand auto_lowerWinchLeftCommand, auto_lowerWinchRightCommand, auto_reversePivotLeftCommand, auto_reversePivotRightCommand;
/************************* VARIABLES *************************/

  public RobotContainer() {

  /************************* JOYSTICKS *************************/
    xbox1 = new Joystick(0);
    xbox1A = new JoystickButton(xbox1, Button.kA.value);
    xbox1B = new JoystickButton(xbox1, Button.kB.value);
    xbox1X = new JoystickButton(xbox1, Button.kX.value);
    xbox1Y = new JoystickButton(xbox1, Button.kY.value);
    xbox1LB = new JoystickButton(xbox1, Button.kLeftBumper.value);
    xbox1RB = new JoystickButton(xbox1, Button.kRightBumper.value);
    xbox1Back = new JoystickButton(xbox1, Button.kBack.value);
    xbox1Start = new JoystickButton(xbox1, Button.kStart.value);
    xbox1LTrigger = xbox1.getRawAxis(2);
    xbox1RTrigger = xbox1.getRawAxis(3);

    xbox2 = new Joystick(1);
    xbox2A = new JoystickButton(xbox2, Button.kA.value);
    xbox2B = new JoystickButton(xbox2, Button.kB.value);
    xbox2X = new JoystickButton(xbox2, Button.kX.value);
    xbox2Y = new JoystickButton(xbox2, Button.kY.value);
    xbox2LB = new JoystickButton(xbox2, Button.kLeftBumper.value);
    xbox2RB = new JoystickButton(xbox2, Button.kRightBumper.value);
  /************************* JOYSTICKS *************************/


  /************************* DRIVE *************************/
    driveMotor1 = new TalonFX(1);
    driveMotor2 = new TalonFX(2);
    driveMotor3 = new TalonFX(3);
    driveMotor4 = new TalonFX(4);
    driveMotor5 = new TalonFX(5);
    driveMotor6 = new TalonFX(6);
    driveMotor7 = new TalonFX(7);
    driveMotor8 = new TalonFX(8);

    // driveMotor1.setStatusFramePeriod(StatusFrame.Status_1_General, 500);
    // driveMotor2.setStatusFramePeriod(StatusFrame.Status_1_General, 500);
    // driveMotor3.setStatusFramePeriod(StatusFrame.Status_1_General, 500);
    // driveMotor4.setStatusFramePeriod(StatusFrame.Status_1_General, 500);
    // driveMotor5.setStatusFramePeriod(StatusFrame.Status_1_General, 500);
    // driveMotor6.setStatusFramePeriod(StatusFrame.Status_1_General, 500);
    // driveMotor7.setStatusFramePeriod(StatusFrame.Status_1_General, 500);
    // driveMotor8.setStatusFramePeriod(StatusFrame.Status_1_General, 500);

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
  /************************* DRIVE *************************/


  /************************* SHOOTER *************************/
    shooterWheel1 = new CANSparkMax(Constants.ShooterConstants.SHOOTER_MOTOR_ONE, MotorType.kBrushless);
    shooterWheel2 = new CANSparkMax(Constants.ShooterConstants.SHOOTER_MOTOR_TWO, MotorType.kBrushless);
    shooter = new Shooter();

    shooterWheel1.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 500);
    shooterWheel2.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 500);

    shootCommand = new ShootCommand(-Constants.ShooterConstants.SHOOTER_RPM);
    reverseShootCommand = new ShootCommand(-Constants.ShooterConstants.SHOOTER_RPM_HALF);
  /************************* SHOOTER *************************/


  /************************* HOOD *************************/
    // shooterHood = new WPI_VictorSPX(Constants.ShooterConstants.SHOOTER_MOTOR_HOOD);
    // raiseHood = new HoodCommand(Constants.ShooterConstants.HOOD_HIGH);
    // lowerHood = new HoodCommand(Constants.ShooterConstants.HOOD_LOW);
  /************************* HOOD *************************/


  /************************* INTAKE *************************/
    intakeMotorField = new WPI_VictorSPX(Constants.IntakeConstants.INTAKE_MOTOR_FIELD);
    intakeMotorElevator1 = new WPI_VictorSPX(Constants.IntakeConstants.INTAKE_MOTOR_ELEVATOR_ONE);
    intakeMotorElevator2 = new WPI_VictorSPX(Constants.IntakeConstants.INTAKE_MOTOR_ELEVATOR_TWO);
    intakeColorSensor = new AnalogInput(Constants.IntakeConstants.INTAKE_COLOR_SENSOR);

    intake = new Intake();
    
    intakeFieldCommand = new StartEndCommand(
      () -> intake.moveIntakeField(Constants.IntakeConstants.INTAKE_FIELD_SPEED),
      () -> intake.moveIntakeField(0),
      intake
    );
    reverseIntakeFieldCommand = new StartEndCommand(
      () -> intake.moveIntakeField(-Constants.IntakeConstants.INTAKE_FIELD_SPEED),
      () -> intake.moveIntakeField(0),
      intake
    );

    intakeElevator1Command = new StartEndCommand(
      () -> intake.moveIntakeElevator1(Constants.IntakeConstants.INTAKE_ELEVATOR_SPEED), 
      () -> intake.moveIntakeElevator1(0),
      intake
    );
    reverseIntakeElevator1Command = new StartEndCommand(
      () -> intake.moveIntakeElevator1(-Constants.IntakeConstants.INTAKE_ELEVATOR_SPEED), 
      () -> intake.moveIntakeElevator1(0),
      intake
    );

    intakeElevator2Command = new StartEndCommand(
      () -> intake.moveIntakeElevator2(Constants.IntakeConstants.INTAKE_ELEVATOR_SPEED), 
      () -> intake.moveIntakeElevator2(0),
      intake
    );
    reverseIntakeElevator2Command = new StartEndCommand(
      () -> intake.moveIntakeElevator2(-Constants.IntakeConstants.INTAKE_ELEVATOR_SPEED), 
      () -> intake.moveIntakeElevator2(0),
      intake
    );

    intakeAllCommand = new StartEndCommand(
      () -> intake.moveIntake(1), 
      () -> intake.moveIntake(0),
      intake
    );
    reverseIntakeAllCommand = new StartEndCommand(
      () -> intake.moveIntake(-1), 
      () -> intake.moveIntake(0),
      intake
    );
    

  /************************* INTAKE *************************/


  /************************* TURRET *************************/
    turretEncoder = new AnalogPotentiometer(Constants.TurretConstants.TURRET_ENCODER, 360, 0);
    turretMotor = new WPI_VictorSPX(Constants.TurretConstants.TURRET_MOTOR);
    turret = new Turret();
    turretRight = new StartEndCommand(
      () -> turret.moveTurret(0.2),
      () -> turret.moveTurret(0),
      turret
    );
    turretLeft = new StartEndCommand(
      () -> turret.moveTurret(-0.2),
      () -> turret.moveTurret(0),
      turret
    );
    // turretGyro = new AHRS(SPI.Port.kMXP);
    // turretAutoPositioningCommand = new TurretAutoPositioningCommand();
    // turret.setDefaultCommand(turretAutoPositioningCommand);
    // turretMoveLeftCommand = new TurretCommand(xbox1LTrigger);
    // turretMoveLeftCommand = new TurretCommand(xbox1RTrigger);
  /************************* TURRET *************************/


  /************************* CLIMBER *************************/
    winchLeft = new TalonFX(Constants.ClimberConstants.WINCH_LEFT);
    winchRight = new TalonFX(Constants.ClimberConstants.WINCH_RIGHT);
    pivotLeft = new TalonFX(Constants.ClimberConstants.PIVOT_LEFT);
    pivotRight = new TalonFX(Constants.ClimberConstants.PIVOT_RIGHT);

    winchLeft.setStatusFramePeriod(StatusFrame.Status_1_General, 500);
    winchRight.setStatusFramePeriod(StatusFrame.Status_1_General, 500);
    pivotLeft.setStatusFramePeriod(StatusFrame.Status_1_General, 500);
    pivotRight.setStatusFramePeriod(StatusFrame.Status_1_General, 500);

    climber = new Climber();
    boolean useLimits = true;

    winchLeftCommand = new ClimberCommand(winchLeft, Constants.ClimberConstants.WINCH_HIGH_POSITION, Constants.ClimberConstants.WINCH_LOW_POSITION, -Constants.ClimberConstants.WINCH_SPEED, useLimits);
    lowerWinchLeftCommand = new ClimberCommand(winchLeft, Constants.ClimberConstants.WINCH_HIGH_POSITION, Constants.ClimberConstants.WINCH_LOW_POSITION, Constants.ClimberConstants.WINCH_SPEED, useLimits);
    pivotLeftCommand = new ClimberCommand(pivotLeft, Constants.ClimberConstants.PIVOT_REAR_POSITION, Constants.ClimberConstants.PIVOT_FRONT_POSITION, Constants.ClimberConstants.PIVOT_SPEED, useLimits);
    reversePivotLeftCommand = new ClimberCommand(pivotLeft, Constants.ClimberConstants.PIVOT_REAR_POSITION, Constants.ClimberConstants.PIVOT_FRONT_POSITION, -Constants.ClimberConstants.PIVOT_SPEED, useLimits);

    winchRightCommand = new ClimberCommand(winchRight, Constants.ClimberConstants.WINCH_LOW_POSITION, -Constants.ClimberConstants.WINCH_HIGH_POSITION, Constants.ClimberConstants.WINCH_SPEED, useLimits);
    lowerWinchRightCommand = new ClimberCommand(winchRight, Constants.ClimberConstants.WINCH_LOW_POSITION, -Constants.ClimberConstants.WINCH_HIGH_POSITION, -Constants.ClimberConstants.WINCH_SPEED, useLimits);
    pivotRightCommand = new ClimberCommand(pivotRight, Constants.ClimberConstants.PIVOT_REAR_POSITION, Constants.ClimberConstants.PIVOT_FRONT_POSITION, Constants.ClimberConstants.PIVOT_SPEED, useLimits);
    reversePivotRightCommand = new ClimberCommand(pivotRight, Constants.ClimberConstants.PIVOT_REAR_POSITION, Constants.ClimberConstants.PIVOT_FRONT_POSITION, -Constants.ClimberConstants.PIVOT_SPEED, useLimits);

    climberAutomation = new ClimberAutomation();
    SmartDashboard.putData(climberAutomation);
  /************************* CLIMBER *************************/


  /************************* OTHER *************************/
    navX = new AHRS(SerialPort.Port.kMXP);
    calibrateGyro = new InstantCommand(() -> Constants.RobotConstants.GYRO_OFFSET = -navX.getAngle() + 180);
    calibrateModules = new InstantCommand(() -> swerveGroup.calibrate());
    SmartDashboard.putData("Calibrate Modules", calibrateModules);
    calibrateClimbers = new InstantCommand(
      () -> {
      winchRight.setSelectedSensorPosition(Constants.ClimberConstants.WINCH_RESET_POSITION);
      winchLeft.setSelectedSensorPosition(Constants.ClimberConstants.WINCH_RESET_POSITION);
      pivotLeft.setSelectedSensorPosition(Constants.ClimberConstants.PIVOT_RESET_POSITION);
      pivotRight.setSelectedSensorPosition(Constants.ClimberConstants.PIVOT_RESET_POSITION);
      }
    );
  /************************* OTHER *************************/


  /************************* AUTO *************************/
    // autoComplex = new ComplexAutoPath();
    // autoChooser = new SendableChooser<>();
    // autoChooser.setDefaultOption("Default Auto", autoComplex);
    // autoChooser.addOption("Complex Auto", autoComplex);
    // SmartDashboard.putData(autoChooser);
  /************************* AUTO *************************/


  /************************* BUTTON BINDING METHOD(S) *************************/
    configureButtonBindings();
  /************************* BUTTON BINDING METHOD(S) *************************/
  }

  private void configureButtonBindings() {
  // Primary driver controls: intake, drive(not initialized here), climb?
    xbox1LB.whileHeld(pivotRightCommand);
    xbox1RB.whileHeld(reversePivotRightCommand);
    xbox1Back.whenPressed(calibrateClimbers);

    // xbox1LB.whileHeld(pivotLeftCommand);
    // xbox1RB.whileHeld(reversePivotLeftCommand);
    // xbox1LB.whileHeld(turretLeft);
    // xbox1RB.whileHeld(turretRight);

    xbox1A.whileHeld(winchRightCommand);
    xbox1B.whileHeld(lowerWinchRightCommand);
    

    xbox1X.whileHeld(intakeAllCommand);
    xbox1Y.whileHeld(reverseIntakeAllCommand);
    xbox1Start.whenPressed(calibrateGyro);


  // Secondary driver controls: shooter, turret, and hood
    // xbox2B.whileHeld(shootCommand);
    // xbox2X.whileHeld(reverseShootCommand);
    // xbox2A.whileHeld(lowerHood);
    // xbox2Y.whileHeld(raiseHood);
  xbox2A.whileHeld(winchLeftCommand);
  xbox2B.whileHeld(lowerWinchLeftCommand);
  xbox2LB.whileHeld(pivotLeftCommand);
  xbox2RB.whileHeld(reversePivotLeftCommand);

  xbox2X.whileHeld(shootCommand);
  xbox2Y.whileHeld(reverseShootCommand);
  }

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
