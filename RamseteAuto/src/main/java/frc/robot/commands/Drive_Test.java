// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Drive_Test extends CommandBase {
  Drivetrain drivetrain;
  XboxController driverController;

  double Ks = 0;
  boolean moving = false;
  double volts = 0;

  public Drive_Test(Drivetrain drivetrain, XboxController driverController) {
    SmartDashboard.putNumber("Ks", 0);
    moving = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    this.drivetrain = drivetrain;
    this.driverController = driverController;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    volts = driverController.getLeftY();
    drivetrain.voltDrive(volts, volts);

    double velocity = (drivetrain.leftEncoderVelocity() + drivetrain.rightEncoderVelocity()) / 2;
    if (velocity >= 0 && !moving){
      moving = true;
      Ks = volts;
    }

    SmartDashboard.putNumber("Ks", Ks);
    SmartDashboard.putNumber("Controller", volts);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
