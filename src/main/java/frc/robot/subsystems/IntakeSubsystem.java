// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    TalonFX feederMotor = new TalonFX(Constants.IntakeConstants.FEEDER_MOTOR_ID);
    DigitalInput feederSensor = new DigitalInput(Constants.IntakeConstants.FEEDER_SENSOR_ID);

    TalonSRX intakeLeftMotor = new TalonSRX(Constants.IntakeConstants.INTAKE_LEFT_MOTOR_ID);
    TalonSRX intakeRightMotor = new TalonSRX(Constants.IntakeConstants.INTAKE_RIGHT_MOTOR_ID);

    public IntakeSubsystem() {
    }

    // To turn on intake
    public Command intakeOn(double intakeSpeed) {
        return new Command() {
            @Override
            public void initialize() {
                intakeLeftMotor.set(ControlMode.PercentOutput, intakeSpeed);
                intakeRightMotor.set(ControlMode.PercentOutput, intakeSpeed);
            }
        };
    }

    // To turn off intake
    public Command intakeOff() {
        return new Command() {
            @Override
            public void initialize() {
                intakeLeftMotor.set(ControlMode.PercentOutput, 0.0);
                intakeRightMotor.set(ControlMode.PercentOutput, 0.0);
            }
        };
    }

    public Command feederOn(double feederSpeed) {
        return new Command() {
            @Override
            public void initialize(){
                feederMotor.set(feederSpeed);
            }
        };
    }

    public Command feederOff() {
        return new Command() {
            @Override
            public void initialize(){
                feederMotor.set(0.0);
            }
        };
    }

    public Command intakeNoteToFeeder(){
        return new Command() {
            @Override
            public void initialize() {
                feederMotor.set(0.5);

                intakeLeftMotor.set(ControlMode.PercentOutput, 0.5);
                intakeRightMotor.set(ControlMode.PercentOutput, 0.5);
            }

            @Override
            public boolean isFinished() {
                return feederSensor.get();
            }

            @Override
            public void end(boolean interrupted) {
                feederMotor.set(0.0);

                intakeLeftMotor.set(ControlMode.PercentOutput, 0.0);
                intakeRightMotor.set(ControlMode.PercentOutput, 0.0);
            }
        };
    }

    public Command outtakeNote(){
        return new Command(){
            @Override
            public void initialize() {
                feederMotor.set(-0.5);

                intakeLeftMotor.set(ControlMode.PercentOutput, -0.5);
                intakeRightMotor.set(ControlMode.PercentOutput, -0.5);
            }
        };
    }
}
