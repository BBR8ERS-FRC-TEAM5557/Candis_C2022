// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LaunchSubsystem;
import frc.robot.subsystems.Limelight;

public class LaunchFromDistanceCmd extends CommandBase {
  /** Creates a new IntakeIn. */

  LaunchSubsystem launch;
  IntakeSubsystem intake;
  Limelight limelight;

  double k;
  
  NetworkTableEntry a2Tab, distanceTab;

  public LaunchFromDistanceCmd() {
    // Use addRequirements() here to declare subsystem dependencies.
    launch = LaunchSubsystem.getInstance();
    limelight = Limelight.getInstance();
    intake = IntakeSubsystem.getInstance();
    addRequirements(launch, intake);

    a2Tab = Shuffleboard.getTab("launcher").add("A2", 0).getEntry();
    distanceTab = Shuffleboard.getTab("launcher").add("distance", 0).getEntry();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
        limelight.disableDriverMode();
        limelight.enableLEDs();

        double limelightHeight = 42; // height of the center of the limelight camera lens
        final double targetHeight = 68; // height of center of reflective tape on powerport
        double dh = targetHeight - limelightHeight;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        double limelightHeight = 42.00; // height of the center of the limelight camera lens
        final double targetHeight = 68; // height of center of reflective tape on powerport
        double dh = targetHeight - limelightHeight;

        double a1 = 37; // angle at which the limelight is mounted
        double a2 = Limelight.getInstance().getAngleY(); // Vertical Offset From Crosshair To Target
        double dx = (dh / (Math.tan(Math.toRadians(a1 + a2)))); // this is the distance that the limelight is from the
                                                                // face of the powerport
        a2Tab.setDouble(a2);
        distanceTab.setDouble(dx);
        // LauncherSubsystem.getInstance().launch(k);

        // for the itme being, gonna be using a table here
        if (dx <= 120) {
            launch.launch(-.8);
        } else if (dx > 120 && dx <= 220) {
            launch.launch(-.9);
        } else {
            launch.launch(-1);
        }

        //intake.intakeIn();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
        limelight.enableDriverMode();
        limelight.disableLEDs();

        //intake.stop();
        launch.stopLaunch();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
