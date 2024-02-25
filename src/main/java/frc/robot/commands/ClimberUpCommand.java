package frc.robot.commands;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberUpCommand extends Command{

    ClimberSubsystem _climberSubsystem = new ClimberSubsystem();
    TalonFX _climberLeftMotor = _climberSubsystem.climberLeftMotor;
    TalonFX _climberRightMotor = _climberSubsystem.climberRightMotor;
    private StatusSignal<Double> leftPosition;
    private StatusSignal<Double> rightPosition;
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

    public void stopBot()
    {
        rightPosition = _climberSubsystem.climberLeftMotor.getPosition();
        leftPosition = _climberSubsystem.climberRightMotor.getPosition();
        SmartDashboard.putString("Climber Right position", rightPosition.toString());
        SmartDashboard.putString("Climber Left position", leftPosition.toString());
        // if(rightPosition.getValue() == 1.0 && leftPosition.getValue() == 1.0) //hardcode later
        // {
        //     "Robot" speed = 0;
        // }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
