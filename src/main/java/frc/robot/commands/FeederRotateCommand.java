
package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class FeederRotateCommand extends Command {

    IntakeSubsystem _intakeSubsystem;
    double rotations;
    double startingPos = 0;

    PositionVoltage posRequest;

    StatusSignal<Angle> feederPos;

    public FeederRotateCommand(IntakeSubsystem intakeSubsystem, double rotations) {
        _intakeSubsystem = intakeSubsystem;
        this.rotations = rotations;
        addRequirements(_intakeSubsystem);

        feederPos = _intakeSubsystem.feederMotor.getPosition();
        posRequest = new PositionVoltage(0).withSlot(0);
    }

    @Override
    public void initialize() {
        //feederPos.waitForUpdate(0.1);
        _intakeSubsystem.feederMotor.setPosition(0);
        //startingPos = feederPos.refresh().getValue();
        _intakeSubsystem.feederMotor.setControl(posRequest.withPosition(rotations));
        Logger.recordOutput("Feeder Request Pos", rotations);
    }


    @Override
    public boolean isFinished() {
        feederPos.refresh();

        Logger.recordOutput("Feeder Error",  Math.abs(feederPos.getValueAsDouble() - (rotations)));

        if (feederPos.hasUpdated()) {
            return Math.abs(feederPos.getValueAsDouble() - (rotations)) <= 0.1;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // _intakeSubsystem.feederMotor.stopMotor();
    }
}
