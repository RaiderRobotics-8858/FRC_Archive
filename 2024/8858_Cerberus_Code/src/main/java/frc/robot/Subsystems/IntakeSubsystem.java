package frc.robot.Subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {
    public static final DigitalInput limitSwitch = new DigitalInput(0);

    public static SparkMax intakeMotor = new SparkMax(Constants.kIntakeMotorId, MotorType.kBrushless) {
        {
            set(0);
        }
    };

    // This is Used to create an intance of this class whitin other files

    public IntakeSubsystem() {
        // intakeMotor.setInverted(false);
        // intakeMotor.setIdleMode(IdleMode.kBrake);
    }

    // sets the motor speed with the parameter intakeSpeed
    public static void setMotor(double intakeSpeed) {
        intakeMotor.set(intakeSpeed);
    }
}
