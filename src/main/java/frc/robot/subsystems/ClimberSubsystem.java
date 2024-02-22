// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    public TalonFX climberLeftMotor = new TalonFX(Constants.ClimberConstants.CLIMBER_LEFT_MOTOR_ID);
    public TalonFX climberRightMotor = new TalonFX(Constants.ClimberConstants.CLIMBER_RIGHT_MOTOR_ID);

    public ClimberSubsystem(){
        climberLeftMotor.setInverted(false);
        climberRightMotor.setInverted(true);
    }
}
