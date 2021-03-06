// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class SharedMethods {
    public static double roundTo(double value, int places) {
        double scale = Math.pow(10, places);
        return Math.round(value * scale) / scale;
    }
    public static double convertInchesToMeters(double inches){
        double meters = inches / 39.37;
        return meters;
    }
    public static double convertDegreesToRadians(double degrees){
        return (degrees * Math.PI) / 180;
    }
    public static int positiveOrNegative(double number){
        if (number >= 0) return 1;
        else return -1;
    }
}
