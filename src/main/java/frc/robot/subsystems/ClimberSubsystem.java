// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    public TalonFX climberLeftMotor = new TalonFX(Constants.ClimberConstants.CLIMBER_LEFT_MOTOR_ID);
    public TalonFX climberRightMotor = new TalonFX(Constants.ClimberConstants.CLIMBER_RIGHT_MOTOR_ID);

    StatusSignal<Double> leftClimberPos;
    StatusSignal<Double> rightClimberPos;

    public ClimberSubsystem(){

        TalonFXConfiguration climberLeftConfig = new TalonFXConfiguration();
        climberLeftConfig.Slot0.kP = 3;
        climberLeftConfig.Slot0.kI = 1;
        climberLeftConfig.Slot0.kD = 0.5;
        climberLeftMotor.getConfigurator().apply(climberLeftConfig);
        climberLeftMotor.setInverted(true);
        climberLeftMotor.setNeutralMode(NeutralModeValue.Brake);
        leftClimberPos = climberLeftMotor.getPosition();
        leftClimberPos.setUpdateFrequency(50);
        rightClimberPos = climberLeftMotor.getPosition();
        rightClimberPos.setUpdateFrequency(50);
        
        TalonFXConfiguration climberRightConfig = new TalonFXConfiguration();
        climberRightConfig.Slot0.kP = 3;
        climberRightConfig.Slot0.kI = 1;
        climberRightConfig.Slot0.kD = 0.5;
        climberRightMotor.getConfigurator().apply(climberRightConfig);
        climberRightMotor.setInverted(true);
        climberRightMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Left Climber Position", leftClimberPos.refresh().getValue());
        Logger.recordOutput("Right Climber Position", rightClimberPos.refresh().getValue());
    }
}
