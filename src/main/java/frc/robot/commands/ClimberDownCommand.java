package frc.robot.commands;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberDownCommand extends Command{

    ClimberSubsystem _climberSubsystem = new ClimberSubsystem();
    TalonFX _climberLeftMotor = _climberSubsystem.climberLeftMotor;
    TalonFX _climberRightMotor = _climberSubsystem.climberRightMotor;
    private StatusSignal<Double> leftPosition;
    private StatusSignal<Double> rightPosition;
    double speed;

    public ClimberDownCommand(ClimberSubsystem climberSubsystem, double speed){
        _climberSubsystem = climberSubsystem;
        this.speed = speed;
        addRequirements(_climberSubsystem);
    }

    @Override
    public void initialize() {
        _climberSubsystem.climberLeftMotor.set(-speed);
        _climberSubsystem.climberRightMotor.set(-speed);
    }

    @Override
    public void execute()
    {
        rightPosition = _climberSubsystem.climberLeftMotor.getPosition();
        leftPosition = _climberSubsystem.climberRightMotor.getPosition();
        if(leftPosition.getValue() <= 0 && leftPosition.getValue() >= 165)
        {
            _climberLeftMotor.stopMotor();
        }
        if(rightPosition.getValue() <= 1.0 || rightPosition.getValue() >= 165)
        {
            _climberRightMotor.stopMotor();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
