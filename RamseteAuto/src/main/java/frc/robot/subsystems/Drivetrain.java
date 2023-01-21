// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  
  WPI_TalonFX leftMotorOne = new WPI_TalonFX(Constants.DRIVETRAIN_LEFT_MOTOR_ONE);
  WPI_TalonFX leftMotorTwo = new WPI_TalonFX(Constants.DRIVETRAIN_LEFT_MOTOR_TWO);
  WPI_TalonFX leftMotorThree = new WPI_TalonFX(Constants.DRIVETRAIN_LEFT_MOTOR_THREE);
  WPI_TalonFX rightMotorOne = new WPI_TalonFX(Constants.DRIVETRAIN_RIGHT_MOTOR_ONE);
  WPI_TalonFX rightMotorTwo = new WPI_TalonFX(Constants.DRIVETRAIN_RIGHT_MOTOR_TWO);
  WPI_TalonFX rightMotorThree = new WPI_TalonFX(Constants.DRIVETRAIN_RIGHT_MOTOR_THREE);

  MotorControllerGroup leftMotors = new MotorControllerGroup(leftMotorOne, leftMotorTwo, leftMotorThree);
  MotorControllerGroup rightMotors = new MotorControllerGroup(rightMotorOne, rightMotorTwo, rightMotorThree);

  WPI_Pigeon2 gyro = new WPI_Pigeon2(Constants.DRIVETRAIN_GYRO);

  DifferentialDrive differentialDrive = new DifferentialDrive(leftMotors, rightMotors);

  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

  public Drivetrain() {
    leftMotorOne.setNeutralMode(NeutralMode.Brake);
    leftMotorTwo.setNeutralMode(NeutralMode.Brake);
    leftMotorThree.setNeutralMode(NeutralMode.Brake);

    rightMotorOne.setNeutralMode(NeutralMode.Brake);
    rightMotorTwo.setNeutralMode(NeutralMode.Brake);
    rightMotorThree.setNeutralMode(NeutralMode.Brake);

    leftMotors.setInverted(true);
    rightMotors.setInverted(false);

    resetEncoders();
    resetGyro();
    resetOdometry(new Pose2d());
  }

  public Pose2d getPose2d(){
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(leftEncoderVelocity(), rightEncoderVelocity());
  }

  public double leftEncoderPosition(){
    double pulses = leftMotorTwo.getSelectedSensorPosition();

    double wheelRotations = (pulses / Constants.ENCODER_PULSES_PER_ROT) / Constants.ENCODER_GEARING_MOTOR_TO_WHEEL;

    double distance = wheelRotations * Constants.ENCODER_WHEEL_CIRCUMFRENCE_METERS;

    return -distance;
  }

  public double leftEncoderVelocity(){
    double pulsesPer100ms = leftMotorTwo.getSelectedSensorVelocity();

    double rotationsPerSecond = (pulsesPer100ms / Constants.ENCODER_PULSES_PER_ROT) * 10;

    double metersPerSecond = rotationsPerSecond * Constants.ENCODER_WHEEL_CIRCUMFRENCE_METERS;

    return -metersPerSecond;
  }

  public double rightEncoderPosition(){
    double pulses = rightMotorTwo.getSelectedSensorPosition();

    double wheelRotations = (pulses / Constants.ENCODER_PULSES_PER_ROT) / Constants.ENCODER_GEARING_MOTOR_TO_WHEEL;

    double distance = wheelRotations * Constants.ENCODER_WHEEL_CIRCUMFRENCE_METERS;

    return distance;
  }

  public double rightEncoderVelocity(){
    double pulsesPer100ms = rightMotorTwo.getSelectedSensorVelocity();

    double rotationsPerSecond = (pulsesPer100ms / Constants.ENCODER_PULSES_PER_ROT) * 10;

    double metersPerSecond = rotationsPerSecond * Constants.ENCODER_WHEEL_CIRCUMFRENCE_METERS;

    return metersPerSecond;
  }

  public void arcadeDrive(double move, double turn){
    differentialDrive.arcadeDrive(-move, -turn);
  }

  public void voltDrive(double leftVolts, double rightVolts){
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(rightVolts);

    differentialDrive.feed();
  }

  public void resetOdometry(Pose2d resetPose){
    resetEncoders();
    odometry.resetPosition(resetPose, gyro.getRotation2d());
  }

  public void resetGyro(){
    gyro.reset();
  }

  public void resetEncoders(){
    leftMotorTwo.getSensorCollection().setIntegratedSensorPosition(0, 0);
    rightMotorTwo.getSensorCollection().setIntegratedSensorPosition(0, 0);
  }

  @Override
  public void periodic() {
    odometry.update(gyro.getRotation2d(), leftEncoderPosition(), rightEncoderPosition());

    SmartDashboard.putNumber("Gyro Rotation", gyro.getRotation2d().getDegrees());
    SmartDashboard.putNumber("Odometry", odometry.getPoseMeters().getRotation().getDegrees());
    SmartDashboard.putNumber("X", odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Y", odometry.getPoseMeters().getY());
  }
}
