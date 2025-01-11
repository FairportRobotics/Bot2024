package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeOnCommand extends Command{

    IntakeSubsystem _IntakeSubsystem;
    double intakeSpeed;

    VelocityVoltage intakeRequest;

    public IntakeOnCommand(IntakeSubsystem intakeSubsystem, double intakeSpeed){
        _IntakeSubsystem = intakeSubsystem;
        this.intakeSpeed = intakeSpeed;

        addRequirements(_IntakeSubsystem);

        intakeRequest = new VelocityVoltage(0).withSlot(0);
    }

    @Override
    public void initialize() {
        _IntakeSubsystem.intakeLeftMotor.setControl(intakeRequest.withVelocity(intakeSpeed));
//        _IntakeSubsystem.intakeRightMotor.set(ControlMode.PercentOutput, intakeSpeed);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        _IntakeSubsystem.intakeLeftMotor.stopMotor();
    }
}
