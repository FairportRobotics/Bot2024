// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class IntakeSubsystem extends SubsystemBase {
    public TalonFX feederMotor = new TalonFX(Constants.IntakeConstants.FEEDER_MOTOR_ID);
    public DigitalInput feederSensor = new DigitalInput(Constants.IntakeConstants.FEEDER_SENSOR_ID);
    public DigitalInput shooterSensor = new DigitalInput(Constants.IntakeConstants.SHOOTER_SENSOR_ID);

    StatusSignal<Double> feederError;
    StatusSignal<Double> feederPos;

    public TalonFX intakeLeftMotor = new TalonFX(Constants.IntakeConstants.INTAKE_LEFT_MOTOR_ID);
    // public TalonSRX intakeRightMotor = new
    // TalonSRX(Constants.IntakeConstants.INTAKE_RIGHT_MOTOR_ID);

    public IntakeSubsystem() {
        TalonFXConfiguration feederConfig = new TalonFXConfiguration();
        feederConfig.Slot0.kP = 4;
        feederConfig.Slot0.kI = 1;
        feederConfig.Slot0.kD = 0;
        feederMotor.getConfigurator().apply(feederConfig);
        feederPos = feederMotor.getPosition();
        feederPos.setUpdateFrequency(50);
        feederError = feederMotor.getClosedLoopError();
        feederError.setUpdateFrequency(50);
        feederMotor.optimizeBusUtilization();

        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        intakeConfig.Slot0.kP = 2;
        intakeConfig.Slot0.kI = 1;
        intakeConfig.Slot0.kD = 0.0;
        intakeLeftMotor.getConfigurator().apply(intakeConfig);
        intakeLeftMotor.optimizeBusUtilization();
    }

    @Override
    public void periodic() {
        if (DriverStation.isTeleopEnabled()) {
            if (!shooterSensor.get() || RobotContainer.noteAquired) {
                RobotContainer.noteAquired = true;
                RobotContainer.LIGHTING_SUBSYSTEM.setColor(0, 255, 0, 1.0);
            } else {
                RobotContainer.LIGHTING_SUBSYSTEM.setColor(255, 0, 0, 1.0);
            }
        }
        // SmartDashboard.putBoolean("FEEDER SENSOR", feederSensor.get());
        SmartDashboard.putBoolean("NOTE ACQUIRED", !shooterSensor.get());

        Logger.recordOutput("Feeder Position", feederPos.refresh().getValue());
        Logger.recordOutput("Note Acquired", !shooterSensor.get());
    }

}
