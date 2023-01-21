// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.commands.Drive_ArcadeDrive;
import frc.robot.subsystems.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public static Drivetrain drivetrain = new Drivetrain();

  public static XboxController driveController = new XboxController(0);

  public RobotContainer() {
    drivetrain.setDefaultCommand(new Drive_ArcadeDrive(drivetrain, driveController));

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(Trajectory autoTrajectory) {

    RamseteCommand ramseteCommand = new RamseteCommand(autoTrajectory, 
      drivetrain::getPose2d,
      new RamseteController(Constants.AUTO_RAMSETE_B, 
        Constants.AUTO_RAMSETE_ZETA), 
      new SimpleMotorFeedforward(Constants.AUTO_Ks,
        Constants.AUTO_Kv,
        Constants.AUTO_Ka), 
      Constants.AUTO_KINEMATICS, 
      drivetrain::getWheelSpeeds, 
      new PIDController(Constants.AUTO_Kp, 0, 0), 
      new PIDController(Constants.AUTO_Kp, 0, 0), 
      drivetrain::voltDrive, 
      drivetrain);

      drivetrain.resetOdometry(autoTrajectory.getInitialPose());

    return ramseteCommand.andThen(() -> drivetrain.voltDrive(0, 0));
  }
}
