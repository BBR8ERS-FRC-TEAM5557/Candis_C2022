// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.subsystems.LaunchSubsystem;

public class LaunchUpperForTimeCmd extends WaitCommand  {
  /** Creates a new IntakeIn. */
  LaunchSubsystem launch;

  public LaunchUpperForTimeCmd(double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(time);
    
    launch = LaunchSubsystem.getInstance();
    addRequirements(launch);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    launch.launchUpper();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    launch.stopLaunch();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
