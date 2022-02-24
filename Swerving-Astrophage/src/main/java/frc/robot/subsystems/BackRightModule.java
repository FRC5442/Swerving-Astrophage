// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BackRightModule extends SwerveModule {
    
    public BackRightModule(TalonFX topGear, TalonFX bottomGear, AnalogPotentiometer absEncoder) {
        super(topGear, bottomGear, absEncoder, false, 65);
    }

    @Override
    public void updateSmartDashboard() {
        SmartDashboard.putNumber("Back Right Encoder: ", this.currentAngle);
        SmartDashboard.putNumber("Back Right Encoder Zero Offset: ", this.zeroOffset);
        SmartDashboard.putNumber("Back Right New Angle: ", this.rawAngle);

    }
}
