package frc.robot.commands;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ScoringSubsystem;

public class ShooterOnCommand extends Command{

    ScoringSubsystem _scoringSubsystem;
    double speed;

    StatusSignal<AngularVelocity> topVelocity;
    StatusSignal<AngularVelocity> botVelocity;

    final VelocityVoltage speedRequest;

    public ShooterOnCommand(ScoringSubsystem scoringSubsystem, double speed){
        _scoringSubsystem = scoringSubsystem;
        addRequirements(_scoringSubsystem);
        this.speed = speed;

        topVelocity = _scoringSubsystem.shooterTopMotor.getVelocity();
        botVelocity = _scoringSubsystem.shooterBottomMotor.getVelocity();
        speedRequest = new VelocityVoltage(0).withSlot(0);
    }

    @Override
    public void initialize() {
        _scoringSubsystem.shooterTopMotor.setControl(speedRequest.withVelocity(speed));
        _scoringSubsystem.shooterBottomMotor.setControl(speedRequest.withVelocity(speed));
    }

    @Override
    public void execute() {

        botVelocity.refresh();
        topVelocity.refresh();

        SmartDashboard.putNumber("Shooter Bot speed", botVelocity.getValueAsDouble());
        SmartDashboard.putNumber("Shooter Top speed", topVelocity.getValueAsDouble());
    }

    @Override
    public boolean isFinished() {
        return botVelocity.getValueAsDouble() >= speed && topVelocity.getValueAsDouble() >= speed;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Shooter Up to Speed");
    }
}