package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
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
import frc.robot.subsystems.PneumaticSubsystem;


import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
//import frc.robot.input.JoystickAxis;

import frc.robot.Robot;


public class RobotContainer {

    final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
    private final XboxController manipulatorController = new XboxController(OIConstants.kManipulatorControllerPort);


    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        //DRIVER BUTTONS
        final JoystickButton DRIVER_RIGHT_BUMPER_BUTTON = new JoystickButton(driverJoytick, XBoxConstants.rightBumperButton);
        final JoystickButton DRIVER_LEFT_BUMPER_BUTTON = new JoystickButton(manipulatorController, XBoxConstants.leftBumperButton);

        DRIVER_LEFT_BUMPER_BUTTON.whenPressed(new PneumaticInCmd());
        DRIVER_RIGHT_BUMPER_BUTTON.whenPressed(new PneumaticOutCmd());
        new JoystickButton(driverJoytick, OIConstants.kDriverFieldOrientedButtonIdx).whenPressed(() -> swerveSubsystem.zeroHeading());
        

        //MANIPULATOR BUTTONS
        final JoystickButton MANIPULATOR_A_BUTTON = new JoystickButton(manipulatorController, XBoxConstants.aButton);
        final JoystickButton MANIPULATOR_B_BUTTON = new JoystickButton(manipulatorController, XBoxConstants.bButton);
        final JoystickButton MANIPULATOR_X_BUTTON = new JoystickButton(manipulatorController, XBoxConstants.xButton);
        final JoystickButton MANIPULATOR_Y_BUTTON = new JoystickButton(manipulatorController, XBoxConstants.yButton);
        final JoystickButton MANIPULATOR_LEFT_BUMPER_BUTTON = new JoystickButton(manipulatorController, XBoxConstants.leftBumperButton);
        final JoystickButton MANIPULATOR_RIGHT_BUMPER_BUTTON = new JoystickButton(manipulatorController, XBoxConstants.rightBumperButton);
        //final JoystickButton MANIPULATOR_START_BUTTON = new JoystickButton(manipulatorController, XBoxConstants.startButton);
        //final JoystickButton MANIPULATOR_BACK_BUTTON = new JoystickButton(manipulatorController, XBoxConstants.backButton);

        MANIPULATOR_X_BUTTON.whileHeld(new IntakeInCmd());
        MANIPULATOR_A_BUTTON.whileHeld(new FullFeedLaunchCmd());
        MANIPULATOR_Y_BUTTON.toggleWhenPressed(new LaunchUpperCmd());
        MANIPULATOR_B_BUTTON.toggleWhenPressed(new LaunchLowerCmd());
        MANIPULATOR_RIGHT_BUMPER_BUTTON.whileHeld(new ClimbUpCmd());
        MANIPULATOR_LEFT_BUMPER_BUTTON.whileHeld(new ClimbDownCmd());

    }

    public Command getAutonomousCommand1() {
        // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);

        // 2. Generate trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(2, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(0, 1),
                        new Translation2d(0, 1)),
                new Pose2d(2, 2, Rotation2d.fromDegrees(180)),
                trajectoryConfig);

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);

        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> swerveSubsystem.stopModules()));
    }

    public Command getAutonomousCommand2() {
        // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);

        // 2. Generate trajectory
        
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(1, 0),
                        new Translation2d(1, 0)),
                new Pose2d(2, 0, new Rotation2d(0)),
                trajectoryConfig);

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);

        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> swerveSubsystem.stopModules()));
    }

    public Command getAutonomousCommand3() {
        String trajectoryJSON = "paths/testpath.wpilib.json";
        Trajectory trajectory = new Trajectory();
        
        try {
                Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
                trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
             } catch (IOException ex) {
                DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
             }

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);

        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))),
                swerveControllerCommand,
                new InstantCommand(() -> swerveSubsystem.stopModules()));
    }

}
