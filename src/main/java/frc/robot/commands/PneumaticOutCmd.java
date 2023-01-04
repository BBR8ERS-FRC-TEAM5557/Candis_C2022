// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.Robot;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class PneumaticOutCmd extends CommandBase {
  /** Creates a new IntakeOut. */
  PneumaticSubsystem pneumatics;

  public PneumaticOutCmd() {
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
    pneumatics.extendIntake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //pneumatics.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}