package frc.robot;

import java.util.List;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
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


import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.input.JoystickAxis;



public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

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
        new JoystickButton(driverJoytick, 2).whenPressed(() -> swerveSubsystem.zeroHeading());
        final JoystickButton DRIVER_RIGHT_BUMPER_BUTTON = new JoystickButton(driverJoytick, XBoxConstants.rightBumperButton);

        DRIVER_RIGHT_BUMPER_BUTTON.whileHeld(new LaunchFromDistanceCmd());

        //MANIPULATOR BUTTONS
        final JoystickButton A_BUTTON = new JoystickButton(manipulatorController, XBoxConstants.aButton);
        final JoystickButton B_BUTTON = new JoystickButton(manipulatorController, XBoxConstants.bButton);
        final JoystickButton X_BUTTON = new JoystickButton(manipulatorController, XBoxConstants.xButton);
        final JoystickButton Y_BUTTON = new JoystickButton(manipulatorController, XBoxConstants.yButton);
        final JoystickButton LEFT_BUMPER_BUTTON = new JoystickButton(manipulatorController, XBoxConstants.leftBumperButton);
        final JoystickButton RIGHT_BUMPER_BUTTON = new JoystickButton(manipulatorController, XBoxConstants.rightBumperButton);
        final JoystickButton START_BUTTON = new JoystickButton(manipulatorController, XBoxConstants.startButton);
        final JoystickButton BACK_BUTTON = new JoystickButton(manipulatorController, XBoxConstants.backButton);
        
        BACK_BUTTON.whenPressed(new IntakeInCmd());
        START_BUTTON.whenPressed(new IntakeOutCmd());
        //X_BUTTON.whileHeld(new FullIntakeInCmd());
        //A_BUTTON.whileHeld(new FullFeedLaunchCmd());
        //Y_BUTTON.toggleWhenPressed(new LaunchUpperCmd());
        //B_BUTTON.toggleWhenPressed(new LaunchLowerCmd());
        //RIGHT_BUMPER_BUTTON.whileHeld(new ClimbUpCmd());
        //LEFT_BUMPER_BUTTON.whileHeld(new ClimbDownCmd());

    }

    public Command getAutonomousCommand1() {
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
                        new Translation2d(1, -1)),
                new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
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

}
