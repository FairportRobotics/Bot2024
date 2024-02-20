// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    TalonFX int1 = new TalonFX(190);
    TalonFX int2 = new TalonFX(100);
    TalonFX int3 = new TalonFX(20);

    public IntakeSubsystem() {
    }

    // To turn on intake
    public Command intakeOn(double intakeSpeed) {
        return new Command() {
            @Override
            public void execute() {
                int1.set(intakeSpeed);
                int2.set(intakeSpeed);
                int3.set(intakeSpeed);
            }
        };
    }

    // To turn on intake
    public Command intakeOff(double intakeSpeed) {
        return new Command() {
            @Override
            public void execute() {
                int1.set(intakeSpeed);
                int2.set(intakeSpeed);
                int3.set(intakeSpeed);
            }
        };
    }
}
