// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    TalonFX cmb1 = new TalonFX(180);
    TalonFX cmb2 = new TalonFX(94);
    TalonFX cmb3 = new TalonFX(24330);

    public Command climbUp(double climbSpeed) {
        return new Command() {
            @Override
            public void execute() {
                cmb1.set(climbSpeed);
                cmb2.set(climbSpeed);
                cmb3.set(climbSpeed);
            }
        };
    }

    public Command climbDown(double climbSpeed) {
        return new Command() {
            @Override
            public void execute() {
                cmb1.set(climbSpeed);
                cmb2.set(climbSpeed);
                cmb3.set(climbSpeed);
            }
        };
    }
}
