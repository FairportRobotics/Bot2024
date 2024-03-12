package frc.robot.commands;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberGoToPosCommand extends Command{

    public enum ClimberPos{
        kDown,
        kUp
    }

    private ClimberSubsystem _climberSubsystem;
 
    private PositionVoltage requestClimberLeft;
    private PositionVoltage requestClimberRight;

    private ClimberPos requestPos;

    private StatusSignal<Double> leftClimberPos;
    private StatusSignal<Double> rightClimberPos;


    public ClimberGoToPosCommand(ClimberSubsystem climberSubsystem, ClimberPos climberPos){

        _climberSubsystem = climberSubsystem;

        addRequirements(_climberSubsystem);

        requestClimberLeft = new PositionVoltage(0).withSlot(0);

        requestClimberRight = new PositionVoltage(0).withSlot(0);

        requestPos = climberPos;

        leftClimberPos = _climberSubsystem.climberLeftMotor.getPosition();
        rightClimberPos = _climberSubsystem.climberRightMotor.getPosition();
    }

    @Override
    public void initialize() {
        if(requestPos == ClimberPos.kDown){
            _climberSubsystem.climberLeftMotor.setControl(requestClimberLeft.withPosition(0));
            _climberSubsystem.climberRightMotor.setControl(requestClimberRight.withPosition(0));
        }else if(requestPos == ClimberPos.kUp){
            _climberSubsystem.climberLeftMotor.setControl(requestClimberLeft.withPosition(165));
            _climberSubsystem.climberRightMotor.setControl(requestClimberRight.withPosition(165));
        }
    }

    @Override
    public boolean isFinished() {
        if(requestPos == ClimberPos.kDown)
        {
            return leftClimberPos.refresh().getValue() <= 0 || rightClimberPos.refresh().getValue() <= 0;
        }
        else if(requestPos == ClimberPos.kUp)
        {
            return leftClimberPos.refresh().getValue() >= 165 || rightClimberPos.refresh().getValue() >= 165;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        _climberSubsystem.climberLeftMotor.stopMotor();
        _climberSubsystem.climberRightMotor.stopMotor();
    }

}
