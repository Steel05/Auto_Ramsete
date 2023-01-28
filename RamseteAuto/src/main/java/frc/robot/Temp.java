package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class Temp {
    double maxVelocity = 1.5;
    double maxAcceleration = 1.5;
    double maxVoltage = 10;

    public Command getSampleAutoCommand(){
        // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
            Constants.AUTO_Ks,
            Constants.AUTO_Kv,
            Constants.AUTO_Ka),
        Constants.AUTO_KINEMATICS,
        maxVoltage);

// Create config for trajectory
TrajectoryConfig config =
    new TrajectoryConfig(maxVelocity, maxAcceleration)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(Constants.AUTO_KINEMATICS)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);

config.setReversed(false);

// An example trajectory to follow.  All units in meters.
Trajectory exampleTrajectory =
    TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        config);

        
PIDController leftPID = new PIDController(Constants.AUTO_Kp, 0, 0);
PIDController rightPID = new PIDController(Constants.AUTO_Kp, 0, 0);

RamseteCommand ramseteCommand =
    new RamseteCommand(
        exampleTrajectory,
        RobotContainer.drivetrain::getPose2d,
        new RamseteController(Constants.AUTO_RAMSETE_B, Constants.AUTO_RAMSETE_ZETA),
        new SimpleMotorFeedforward(
            Constants.AUTO_Ks,
            Constants.AUTO_Kv,
            Constants.AUTO_Ka),
        Constants.AUTO_KINEMATICS,
        RobotContainer.drivetrain::getWheelSpeeds,
        leftPID,
        rightPID,
        // RamseteCommand passes volts to the callback
        RobotContainer.drivetrain::voltDrive,
        RobotContainer.drivetrain);

// Reset odometry to the starting pose of the trajectory.
RobotContainer.drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

// Run path following command, then stop at the end.
return ramseteCommand.andThen(() -> RobotContainer.drivetrain.voltDrive(0, 0));
    }
}
