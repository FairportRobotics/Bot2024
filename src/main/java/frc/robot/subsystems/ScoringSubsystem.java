package frc.robot.subsystems;//lol "package" HA AH AHAHAHAGGGG *Cough noise *Cough noise*5 *Falls down stairs... - Lukas

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ScoringSubsystem extends SubsystemBase {

    private double elevatorHomPos = -1.1111;

    TalonFX shooterLeftMotor = new TalonFX(Constants.ShooterContants.SHOOTER_LEFT_MOTOR_ID);
    TalonFX shooterRightMotor = new TalonFX(Constants.ShooterContants.SHOOTER_RIGHT_MOTOR_ID);

    TalonFX elevatorLeftMotor = new TalonFX(Constants.ElevatorConstants.ELEVATOR_LEFT_MOTOR_ID);
    TalonFX elevatorRightMotor = new TalonFX(Constants.ElevatorConstants.ELEVATOR_RIGHT_MOTOR_ID);
    //DigitalInput toplimitSwitch;
    DigitalInput bottomlimitSwitch;

    public ScoringSubsystem() {
        //toplimitSwitch = new DigitalInput(8);
        bottomlimitSwitch = new DigitalInput(Constants.ElevatorConstants.ELEVATOR_BOTTOM_SWITCH_ID);

        elevatorLeftMotor.setInverted(false);
        TalonFXConfiguration elevatorMotor1Config = new TalonFXConfiguration();
        elevatorMotor1Config.Slot0.kP = 1;
        elevatorLeftMotor.getConfigurator().apply(elevatorMotor1Config);


        elevatorRightMotor.setInverted(true);

    }

    public Command ScoringMethodCommand() {
        // Inline construction of command goes here.
        return runOnce(
                () -> {
                    /* one-time action goes here */
                });
    }

    public boolean ScoringCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    // SHOOTER
    public Command Shoot(double shootforce) {
        return new Command() {

            @Override
            public void initialize() {
                shooterLeftMotor.set(shootforce);
            }

        };
    }

    public Command shooterOff(){
        return new Command() {
            @Override
            public void initialize() {
                shooterLeftMotor.set(0.0);
            }
        };
    }

    // Elevator
    public Command elevatorUp(double speed) {
        return new Command() {
            @Override
            public void execute() {
                elevatorLeftMotor.set(speed);
                elevatorRightMotor.set(speed);
            }

            @Override
            public void end(boolean interrupted) {
                elevatorLeftMotor.set(0);
                elevatorRightMotor.set(0);
            }
        };
    }

    public Command elevatorDown(double speed) {
        return new Command() {
            @Override
            public void execute() {
                elevatorLeftMotor.set(speed);
                elevatorRightMotor.set(speed);
            }

            @Override
            public void end(boolean interrupted) {
                elevatorLeftMotor.set(0);
                elevatorRightMotor.set(0);
            }
        };
    }

    public Command elevatorGoToPos(double pos){
        return new Command(){
            @Override
            public void initialize() {
                elevatorLeftMotor.setPosition(elevatorHomPos + pos);
            }

            @Override
            public boolean isFinished() {
                return elevatorLeftMotor.getPosition().getValueAsDouble() == elevatorHomPos + pos;
            }
        };
    }

    public Command elevatorFindHome(){
        return new Command() {
            @Override
            public void initialize() {
                elevatorLeftMotor.set(-0.5);
                elevatorRightMotor.set(-0.5);
            }

            @Override
            public boolean isFinished() {
                return bottomlimitSwitch.get();
            }

            @Override
            public void end(boolean interrupted) {
                elevatorLeftMotor.set(0.0);
                elevatorRightMotor.set(0.0);

                elevatorHomPos = elevatorLeftMotor.getPosition().getValueAsDouble();
            }
        };
    }
}