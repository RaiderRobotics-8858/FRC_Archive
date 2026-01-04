package frc.robot.Subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class HangingSubsystem extends SubsystemBase {
    public static SparkMax hangMotorLeft = new SparkMax(Constants.kHangerLeftMotorId, MotorType.kBrushless) {
        {
            set(0);
        }
    };
    public static SparkMax hangMotorRight = new SparkMax(Constants.kHangerRightMotorId, MotorType.kBrushless) {
        {
            set(0);
        }
    };

    public static RelativeEncoder m_LeftEncoder = hangMotorLeft.getEncoder();;
    public static RelativeEncoder m_RightEncoder = hangMotorRight.getEncoder();;

    public final PIDController m_PIDController = new PIDController(0, 0, 0);

    public HangingSubsystem() {

        // hangMotorLeft.setInverted(true);
        // hangMotorRight.setInverted(false);

        // hangMotorLeft.setIdleMode(IdleMode.kBrake);
        // hangMotorRight.setIdleMode(IdleMode.kBrake);

    }

    // sets the motor speed with the parameter hangSpeed
    public static void setMotors(double LeftHangSpeed, double rightHangSpeed) {
        hangMotorLeft.set(-LeftHangSpeed);
        hangMotorRight.set(rightHangSpeed);
    }

    public static void setMotor(double hangSpeed) {
        hangMotorLeft.set(-hangSpeed);
        hangMotorRight.set(hangSpeed);
    }
}