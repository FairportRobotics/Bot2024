
package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class FeederRotateCommand extends Command {

    IntakeSubsystem _intakeSubsystem;
    double rotations;
    double startingPos = 0;

    PositionVoltage posRequest;

    StatusSignal<Double> feederPos;

    public FeederRotateCommand(IntakeSubsystem intakeSubsystem, double rotations) {
        _intakeSubsystem = intakeSubsystem;
        this.rotations = rotations;
        addRequirements(_intakeSubsystem);

        feederPos = _intakeSubsystem.feederMotor.getPosition();
        posRequest = new PositionVoltage(0).withSlot(0);
    }

    @Override
    public void initialize() {
        feederPos.waitForUpdate(0.1);
        startingPos = feederPos.getValue();
        _intakeSubsystem.feederMotor.setControl(posRequest.withPosition(rotations + startingPos));
        Logger.recordOutput("Feeder Request Pos", rotations + startingPos);
    }


    @Override
    public boolean isFinished() {
        feederPos.refresh();

        Logger.recordOutput("Feeder Error",  Math.abs(feederPos.getValue() - (rotations + startingPos)));

        if (feederPos.hasUpdated()) {
            return Math.abs(feederPos.getValue() - (rotations + startingPos)) <= 0.1;
        }

        return false;

    }

    @Override
    public void end(boolean interrupted) {
        _intakeSubsystem.feederMotor.stopMotor();
    }
}
