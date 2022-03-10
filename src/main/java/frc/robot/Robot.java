// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.PneumaticSubsystem;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.io.IOException;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.XBoxConstants;
import frc.robot.subsystems.SwerveSubsystem;
import team2910.lib.control.Path;
import frc.robot.subsystems.PneumaticSubsystem;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    
    private Command m_autonomousCommand;

    private Limelight m_Limelight;

    private RobotContainer m_robotContainer;

    private final Compressor m_compressor = new Compressor(PneumaticsModuleType.REVPH);

    private double lastTimestamp = 0.0;

    SendableChooser<Command> autonomousModes;
    Command autonomousCommand;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // autonomous chooser on the dashboard.

        m_robotContainer = new RobotContainer();
        m_Limelight = new Limelight();
        
        this.setupAutonomousOptions();


    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and
     * test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.

        CommandScheduler.getInstance().run();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
        m_compressor.disable();
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        //m_autonomousCommand = m_robotContainer.getAutonomousCommand1();

        autonomousCommand = autonomousModes.getSelected();
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
        //autonomousCommand.schedule();

        // schedule the autonomous command (example)
        /** 
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
        */
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        /**
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        */
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }

        m_compressor.enableAnalog(50, 60);
        //m_Limelight.enableLEDs();
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {

        addPeriodic(() -> {
            if((m_Limelight.getAngleX() > -20 && m_Limelight.getAngleX() < 20 && m_Limelight.hasTarget()
            && m_Limelight.getAngleX() != 0)) {
                m_robotContainer.driverJoytick.setRumble(RumbleType.kRightRumble, 1.0);
            }else{
                m_robotContainer.driverJoytick.setRumble(RumbleType.kRightRumble, 0);
            };
        }, 0.01, 0.005);
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }

    private void setupAutonomousOptions() {
        ShuffleboardTab tab = Shuffleboard.getTab("Autonomous Options");
        autonomousModes = new SendableChooser<Command>();
        autonomousModes.setDefaultOption("Red Basic 1",
        new SequentialCommandGroup(
            new FullLaunchForTimeCmd(),
            m_robotContainer.getAutonomousCommand3()
            )
        );
        autonomousModes.addOption("Red Two Ball",
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                new IntakeStoreForTimeCmd(),
                new LaunchUpperForTimeCmd()
                ),
            new ParallelRaceGroup(
                m_robotContainer.getAutonomousCommand3(),
                new FullIntakeForTimeCmd()
                ),
            new ParallelCommandGroup(
                new IntakeStoreForTimeCmd(),
                new LaunchUpperForTimeCmd()
                )
            )
        );
        autonomousModes.addOption("Launch & Move Test",
        new SequentialCommandGroup(
            m_robotContainer.getAutonomousCommand3(),
            new ParallelCommandGroup(
                new IntakeStoreForTimeCmd(),
                new LaunchUpperForTimeCmd()
                ),
            m_robotContainer.getAutonomousCommand3()
            )
        );
        autonomousModes.addOption("Motor Test",
            new LaunchUpperForTimeCmd()
        );
        tab.add("Autonomous Mode", autonomousModes).withWidget(BuiltInWidgets.kComboBoxChooser);
    }


}
