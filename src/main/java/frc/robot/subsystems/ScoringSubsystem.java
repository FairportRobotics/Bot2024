package frc.robot.subsystems;//lol "package" HA AH AHAHAHAGGGG *Cough noise *Cough noise*5 *Falls down stairs... - Lukas

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ScoringSubsystem extends SubsystemBase {

    public double rightHomePos = -1.1111;
    public double leftHomePos = -1.1111;

    public TalonFX shooterLeftMotor = new TalonFX(Constants.ShooterContants.SHOOTER_LEFT_MOTOR_ID);
    public TalonFX shooterRightMotor = new TalonFX(Constants.ShooterContants.SHOOTER_RIGHT_MOTOR_ID);

    public TalonFX elevatorLeftMotor = new TalonFX(Constants.ElevatorConstants.ELEVATOR_LEFT_MOTOR_ID);
    public TalonFX elevatorRightMotor = new TalonFX(Constants.ElevatorConstants.ELEVATOR_RIGHT_MOTOR_ID);
    //DigitalInput toplimitSwitch;
    public DigitalInput bottomlimitSwitch;

    public ScoringSubsystem() {
        //toplimitSwitch = new DigitalInput(8);
        bottomlimitSwitch = new DigitalInput(Constants.ElevatorConstants.ELEVATOR_BOTTOM_SWITCH_ID);

        elevatorLeftMotor.setInverted(false);
        TalonFXConfiguration elevatorMotor1Config = new TalonFXConfiguration();
        elevatorMotor1Config.Slot0.kP = 1;
        elevatorMotor1Config.Slot0.kI = 0;
        elevatorMotor1Config.Slot0.kD = 0;
        elevatorLeftMotor.getConfigurator().apply(elevatorMotor1Config);


        elevatorRightMotor.setInverted(true);
        elevatorRightMotor.getConfigurator().apply(elevatorMotor1Config);

        shooterLeftMotor.setInverted(false);
        shooterRightMotor.setInverted(true);

    }

}