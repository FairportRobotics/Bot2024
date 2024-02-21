// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    public TalonFX feederMotor = new TalonFX(Constants.IntakeConstants.FEEDER_MOTOR_ID);
    public DigitalInput feederSensor = new DigitalInput(Constants.IntakeConstants.FEEDER_SENSOR_ID);

    public TalonSRX intakeLeftMotor = new TalonSRX(Constants.IntakeConstants.INTAKE_LEFT_MOTOR_ID);
    public TalonSRX intakeRightMotor = new TalonSRX(Constants.IntakeConstants.INTAKE_RIGHT_MOTOR_ID);
}
