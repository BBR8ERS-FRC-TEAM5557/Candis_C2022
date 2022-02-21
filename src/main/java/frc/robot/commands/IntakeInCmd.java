// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.Robot;

public class IntakeInCmd extends CommandBase {
  /** Creates a new IntakeIn. */
  PneumaticSubsystem pneumatics;

  public IntakeInCmd() {
    // Use addRequirements() here to declare subsystem dependencies.
    pneumatics = PneumaticSubsystem.getInstance();
    addRequirements(pneumatics);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pneumatics.retractIntake();
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