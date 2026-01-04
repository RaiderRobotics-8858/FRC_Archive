package frc.robot.Subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class LauncherSubsystem extends SubsystemBase {
    public static SparkMax launchMotorRight = new SparkMax(Constants.kLaunchMotorRId, MotorType.kBrushless) {
        {
            set(0);
        }
    };
    public static SparkMax launchMotorLeft = new SparkMax(Constants.kLaunchMotorLId, MotorType.kBrushless) {
        {
            set(0);
        }
    };

    public static double getLeftMotorSpeed() {
        return launchMotorLeft.getEncoder().getVelocity();
    }

    public static double getRightMotorSpeed() {
        return launchMotorRight.getEncoder().getVelocity();
    }

    public LauncherSubsystem() {

    }

    // sets motor speeds with the parameter launchSpeed
    public static void setMotors(double launchSpeed) {
        SmartDashboard.putNumber("Left Speed", getLeftMotorSpeed());
        SmartDashboard.putNumber("Right Speed", getRightMotorSpeed());
        launchMotorRight.set(launchSpeed);
        launchMotorLeft.set(-launchSpeed);
    }
}
