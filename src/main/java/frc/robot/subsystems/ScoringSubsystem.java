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

    public TalonFX shooterTopMotor = new TalonFX(Constants.ShooterContants.SHOOTER_TOP_MOTOR_ID);
    public TalonFX shooterBottomMotor = new TalonFX(Constants.ShooterContants.SHOOTER_BOTTOM_MOTOR_ID);

    public TalonFX elevatorLeftMotor = new TalonFX(Constants.ElevatorConstants.ELEVATOR_LEFT_MOTOR_ID);
    public TalonFX elevatorRightMotor = new TalonFX(Constants.ElevatorConstants.ELEVATOR_RIGHT_MOTOR_ID);
    //DigitalInput toplimitSwitch;
    public DigitalInput bottomlimitSwitch;

    public ScoringSubsystem() {
        //toplimitSwitch = new DigitalInput(8);
        bottomlimitSwitch = new DigitalInput(Constants.ElevatorConstants.ELEVATOR_BOTTOM_SWITCH_ID);

        TalonFXConfiguration elevatorMotor1Config = new TalonFXConfiguration();
        elevatorMotor1Config.Slot0.kP = 1;
        elevatorMotor1Config.Slot0.kI = 0;
        elevatorMotor1Config.Slot0.kD = 0;
        elevatorLeftMotor.getConfigurator().apply(elevatorMotor1Config);
        elevatorLeftMotor.setInverted(false);

        TalonFXConfiguration elevatorMotor2Config = new TalonFXConfiguration();
        elevatorMotor2Config.Slot0.kP = 1;
        elevatorMotor2Config.Slot0.kI = 0;
        elevatorMotor2Config.Slot0.kD = 0;
        elevatorRightMotor.getConfigurator().apply(elevatorMotor2Config);
        elevatorRightMotor.setInverted(true);

        shooterTopMotor.setInverted(false);
        shooterBottomMotor.setInverted(false);

    }

}