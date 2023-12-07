// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Tank;

public class TankDriveXbox extends CommandBase {

  private CommandXboxController driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private Tank tankDrive;
  
  /** Tank Drive with Xbox controller */
  public TankDriveXbox(Tank tankDrive) {
    addRequirements(tankDrive);
    this.tankDrive = tankDrive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forward = MathUtil.applyDeadband(-driverController.getLeftY(), 0.1); // update all controller inputs, Xbox controller has different X and Y directions
    double left = MathUtil.applyDeadband(-driverController.getRightX(), 0.1);
    tankDrive.drive(forward, left);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    tankDrive.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
