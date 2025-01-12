package frc.robot.commands;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.LightingSubsystem;

public class ClimberUpCommand extends Command{

    ClimberSubsystem _climberSubsystem = new ClimberSubsystem();
    LightingSubsystem _LightingSubsystem = new LightingSubsystem();
    TalonFX _climberLeftMotor = _climberSubsystem.climberLeftMotor;
    TalonFX _climberRightMotor = _climberSubsystem.climberRightMotor;
    private StatusSignal<Angle> leftPosition;
    private StatusSignal<Angle> rightPosition;
    double speed;

    public ClimberUpCommand(ClimberSubsystem climberSubsystem, double speed){
        _climberSubsystem = climberSubsystem;
        this.speed = speed;
        addRequirements(_climberSubsystem);
    }

    @Override
    public void initialize() {
        _climberSubsystem.climberLeftMotor.set(speed);
        _climberSubsystem.climberRightMotor.set(speed);
    }

    @Override
    public void execute()
    {
        rightPosition = _climberSubsystem.climberLeftMotor.getPosition();
        leftPosition = _climberSubsystem.climberRightMotor.getPosition();
        if(leftPosition.getValueAsDouble() <= 0 && leftPosition.getValueAsDouble() >= 165)
        {
            _climberLeftMotor.stopMotor();
        }
        if(rightPosition.getValueAsDouble() <= 1.0 || rightPosition.getValueAsDouble() >= 165)
        {
            _climberRightMotor.stopMotor();
        }
        if(rightPosition.getValueAsDouble() > 0.1 || leftPosition.getValueAsDouble() > 0.1)
        {
            LightingSubsystem._CANdle.configBrightnessScalar(rightPosition.getValueAsDouble());
            _LightingSubsystem.setClimbColor();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
