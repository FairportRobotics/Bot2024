package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ScoringSubsystem;

public class ShooterOnCommand extends Command{

    ScoringSubsystem _scoringSubsystem;
    double speed;

    public ShooterOnCommand(ScoringSubsystem scoringSubsystem, double speed){
        _scoringSubsystem = scoringSubsystem;
        addRequirements(_scoringSubsystem);
        this.speed = speed;
    }

    @Override
    public void initialize() {
        _scoringSubsystem.shooterTopMotor.set(speed);
        _scoringSubsystem.shooterBottomMotor.set(speed);
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Shooter Bot speed", _scoringSubsystem.shooterBottomMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter Top speed", _scoringSubsystem.shooterTopMotor.getVelocity().getValueAsDouble());
    }

    @Override
    public boolean isFinished() {
        return _scoringSubsystem.shooterBottomMotor.getVelocity().getValue() >= 100 && _scoringSubsystem.shooterTopMotor.getVelocity().getValue() >= 100;
    }
}
