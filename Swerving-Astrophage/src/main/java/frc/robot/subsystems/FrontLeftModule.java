// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FrontLeftModule extends SwerveModule {

    public FrontLeftModule(TalonFX topGear, TalonFX bottomGear, AnalogPotentiometer absEncoder) {
        super(topGear, bottomGear, absEncoder, false, 280);
    }

    @Override
    public void updateSmartDashboard() {
        SmartDashboard.putNumber("Front Left Encoder: ", this.currentAngle);
        SmartDashboard.putNumber("Front Left Encoder Zero Offset: ", this.zeroOffset);
    }
}
