// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class DriveSubsystem extends SubsystemBase {

    // Defines Each of the motors with a spark ID
    public static SparkMax leftFrontMotor = new SparkMax(Constants.kDrivetrainFLMotorId, MotorType.kBrushless) {
        {
            set(0);
        }
    };
    public static SparkMax leftBackMotor = new SparkMax(Constants.kDrivetrainBLMotorId, MotorType.kBrushless) {
        {
            set(0);
        }
    };
    public static SparkMax rightFrontMotor = new SparkMax(Constants.kDrivetrainFRMotorId, MotorType.kBrushless) {
        {
            set(0);
        }
    };
    public static SparkMax rightBackMotor = new SparkMax(Constants.kDrivetrainBRMotorId, MotorType.kBrushless) {
        {
            set(0);
        }
    };

    public static final DifferentialDrive m_robotDrive = new DifferentialDrive(leftFrontMotor, rightFrontMotor);
    public static final DifferentialDrive m_robotDrive2 = new DifferentialDrive(leftBackMotor, rightBackMotor);

    public DriveSubsystem() {
    }

    // Sets the Motor Speeds with the parameters ySpeed and xSpeed
    public static void setMotors(double ySpeed, double xSpeed) {
        double left_motor_speed = ySpeed - xSpeed;
        double right_motor_speed = -ySpeed - xSpeed;

        leftBackMotor.set(left_motor_speed);
        leftFrontMotor.set(left_motor_speed);
        rightBackMotor.set(-right_motor_speed);
        rightFrontMotor.set(right_motor_speed);
    }

}
