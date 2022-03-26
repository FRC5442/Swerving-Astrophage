// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Preferences;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    
  /************************* DRIVE *************************/
    public static final class SwerveConstants {
        public static final double FL_OFFSET = 214; //Preferences.getDouble("FL_OFFSET", 0);//91;
        public static final double FR_OFFSET = 145; //Preferences.getDouble("FR_OFFSET", 0);//348;
        public static final double BL_OFFSET = 153; //Preferences.getDouble("BL_OFFSET",0);//94;
        public static final double BR_OFFSET = 52; //Preferences.getDouble("BR_OFFSET",0);//61;
        public static final boolean FL_INVERTED = true;
        public static final boolean FR_INVERTED = false;
        public static final boolean BL_INVERTED = true;
        public static final boolean BR_INVERTED = false;

        public static final double X_MULTIPLIER = 1.35;
        public static final double Y_MULTIPLIER = 1.35;
        public static final double R_MULTIPLIER = 1.5;
    }
  /************************* DRIVE *************************/



  /************************* SHOOTER *************************/
    public static final class ShooterConstants {
        public static final int SHOOTER_MOTOR_ONE = 0; //20
        public static final int SHOOTER_MOTOR_TWO = 1;  //21
        // public static final int SHOOTER_MOTOR_HOOD = 21; //19

        public static final double SHOOTER_RPM = 0.51; //2700
        public static final double SHOOTER_RPM_HALF = SHOOTER_RPM / 2;
        public static final double HOOD_HIGH = 15;
        public static final double HOOD_LOW = 0;

        public static final double HOOD_kP = 1;
    }
  /************************* SHOOTER *************************/



  /************************* INTAKE *************************/
    public static final class IntakeConstants {
        public static final double INTAKE_FIELD_SPEED = 0.85;
        public static final double INTAKE_PIVOT_SPEED = 0.2;
        public static final double INTAKE_ELEVATOR_SPEED = 0.95;

        public static final int INTAKE_MOTOR_PIVOT = 18;
        // public static final int INTAKE_MOTOR_PIVOT = 0;
        public static final int INTAKE_MOTOR_ELEVATOR_ONE = 15;
        public static final int INTAKE_MOTOR_ELEVATOR_TWO = 16; //16

        public static final int INTAKE_LASER_SWITCH = 0;
        public static final int INTAKE_COLOR_SENSOR = 7;

        public static final double INTAKE_PIVOT_MIN_POS = 0;
        public static final double INTAKE_PIVOT_MAX_POS = 20000;

        public static final boolean USE_INTAKE_PIVOT_LIMITS = false;
    }
  /************************* INTAKE *************************/



  /************************* TURRET *************************/
    public static final class TurretConstants {
        public static final int TURRET_MOTOR = 17;
        public static final int TURRET_ENCODER = 6;
        
        public static final double TURRET_kP = 1;
        public static final double TURRET_kI = 0;
        public static final double TURRET_kD = 0;

        public static final double MAX_SPEED = 1.5;
        public static final double MIN_SPEED = 0.1;

        public static final double INCREMENT_MILLIS = 100;
        public static double TURRET_ENCODER_OFFSET = 0; //in degrees
        public static final double TURRET_ENCODER_CENTER_POSITION = 0;
    }
  /************************* TURRET *************************/



  /************************* CLIMBER *************************/
    public static final class ClimberConstants {
        public static final int PIVOT_LEFT = 14;
        public static final int PIVOT_RIGHT = 13;
        public static final int WINCH_LEFT = 12;
        public static final int WINCH_RIGHT = 11;

        public static final double WINCH_SPEED = 0.5;
        public static final double PIVOT_SPEED = 0.2;

        public static final double PIVOT_FRONT_POSITION = 78000;
        public static final double PIVOT_REAR_POSITION = 0;
        public static final double PIVOT_RESET_POSITION = 30000;


        public static final double WINCH_HIGH_POSITION = -200000;
        public static final double WINCH_LOW_POSITION = 0;
        public static final double WINCH_RESET_POSITION = 0;

        public static final double CLIMBER_kP = 20000;

        public static boolean CLIMBER_RESET_LIMITS = true;
    }
  /************************* CLIMBER *************************/


    public static final class RobotConstants {
        public static final double ENCODER_OFFSET = 15.0; //in degrees
        public static double GYRO_OFFSET = 0; //in degrees
        public static final double TRIGGER_DEADZONE = 0.1;
        public static final double JOYSTICK_DEAD_ZONE = 0.1; //joystick values 0-1
        public static final double ROBOT_WIDTH = 27; //in inches
        public static final double ROBOT_LENGTH = 19.5; //in inches
        public static final double ROBOT_RADIUS = Math.sqrt(Math.pow(ROBOT_WIDTH, 2) + Math.pow(ROBOT_LENGTH, 2));
    }


    public static enum DRIVE_STATE {
        HIGH_GEAR(0.5), LOW_GEAR(0.2);

        private double value;

        private DRIVE_STATE(double value) {
            this.value = value;
        }

        public double getValue() {
            return this.value;
        }
    }
}
