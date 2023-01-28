// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
//import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // ID Values
    public static final int DRIVETRAIN_LEFT_MOTOR_ONE = 1;
    public static final int DRIVETRAIN_LEFT_MOTOR_TWO = 2;
    public static final int DRIVETRAIN_LEFT_MOTOR_THREE = 3;
    public static final int DRIVETRAIN_RIGHT_MOTOR_ONE = 4;
    public static final int DRIVETRAIN_RIGHT_MOTOR_TWO = 5;
    public static final int DRIVETRAIN_RIGHT_MOTOR_THREE = 6;

    public static final int DRIVETRAIN_GYRO = 15;

    // Chassis Values
    public static final double CHASSIS_MAX_POSSIBLE_VELOCITY = 0.8 * (319 * Units.inchesToMeters(6) * Math.PI) / 45.9;
    public static final double CHASSIS_MAX_POSSIBLE_ACCELERATION = 0.8 * (12 * 4.69 * 15.3) / (Units.lbsToKilograms(125.4) * Units.inchesToMeters(6));

    // Auto Values
    public static final double AUTO_TRACK_WIDTH_METERS = Units.inchesToMeters(25.85);

    public static final double AUTO_Ks = 0.56344;
    public static final double AUTO_Kv = 3.4377;
    public static final double AUTO_Ka = 0.22455;
    public static final double AUTO_Kp = 0.011616;

    public static final double AUTO_RAMSETE_B = 2;
    public static final double AUTO_RAMSETE_ZETA = 0.7;

    public static final DifferentialDriveKinematics AUTO_KINEMATICS = new DifferentialDriveKinematics(AUTO_TRACK_WIDTH_METERS);

    // Encoder Values
    public static final double ENCODER_PULSES_PER_ROT = 2048;
    public static final double ENCODER_GEARING_MOTOR_TO_WHEEL = 15.3;
    public static final double ENCODER_WHEEL_CIRCUMFRENCE_METERS = Units.inchesToMeters(6) * Math.PI;
}
