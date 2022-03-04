package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 5.8462;
        public static final double kTurningMotorGearRatio = 1 / 18.0;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.5;
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(30);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(30);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 4;
        public static final int kBackLeftDriveMotorPort = 6;
        public static final int kFrontRightDriveMotorPort = 2;
        public static final int kBackRightDriveMotorPort = 8;

        public static final int kFrontLeftTurningMotorPort = 3;
        public static final int kBackLeftTurningMotorPort = 5;
        public static final int kFrontRightTurningMotorPort = 1;
        public static final int kBackRightTurningMotorPort = 7;

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = true;
        public static final boolean kBackRightDriveEncoderReversed = true;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 1;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 2;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 0;
        public static final int kBackRightDriveAbsoluteEncoderPort = 3;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        /**
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -Math.toRadians(42 + 180);
        //-0.254;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -Math.toRadians(336.3 + 180 + 10);
        //-1.252;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -Math.toRadians(169.5);
        //-1.816;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -Math.toRadians(136.8 + 180 + 180);
        //-4.811;
         */

        //ANALOG INPUT 1
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -3.87463 - 0.251465 - (1.710205-1.96411);
        //ANALOG INPUT 2
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -2.9024825 - 4.129 + 3.14 - (4.354248-4.195556);
        //ANALOG INPUT 0
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -2.9583331 - 4.34375 + .33683 - (1.922607-1.989746);
        //ANALOG INPUT 3
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -2.3876104 - .38699 + .2985 - (0.52124-0.526123);

        //CHECK MAX SPEED ROBOT TRAVELS IN NORMALIZE FUNCTION (DEFAULT - 5)
        public static final double kPhysicalMaxSpeedMetersPerSecond = 10;

        //CHECK MAX SPEED ROBOT SPIINS IN NORMALIZE FUNCTION (DEFAULT - 2 * 2 * Math.PI)
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 4 * 2 * Math.PI;

        //TRANSLATION SPEED IN TELOP (DEFAULT - kPhysicalMaxSpeedMetersPerSecond / 4)
        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 2;

        //ROTATION SPEED IN TELOP (DEFAULT - kPhysicalMaxAngularSpeedRadiansPerSecond / 4)
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond / 2;

        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kManipulatorControllerPort = 1;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;

        //???????
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.05;
    }

    public static final class XBoxConstants {
        public static final int aButton = 1;
		public static final int bButton = 2;
		public static final int xButton = 3;
		public static final int yButton = 4;
		public static final int leftBumperButton = 5;
		public static final int rightBumperButton = 6;
		public static final int backButton = 7;
		public static final int startButton = 8;
		public static final int leftStickButton = 9;
		public static final int rightStickButton = 10;
    }

    public static final class MotorControllers {
        public static final int RIGHT_CLIMB_MOTOR_CONTROLLER_1 = 9;
		public static final int RIGHT_CLIMB_MOTOR_CONTROLLER_2 = 10;
		public static final int LEFT_CLIMB_MOTOR_CONTROLLER_1 = 11;
		public static final int LEFT_CLIMB_MOTOR_CONTROLLER_2 = 12;
		public static final int INTAKE_MOTOR_CONTROLLER = 13;
		public static final int STORE_MOTOR_CONTROLLER = 14;
		public static final int UPLIFT_MOTOR_CONTROLLER = 15;
		public static final int LEFT_LAUNCH_MOTOR_CONTROLLER = 16;
		public static final int RIGHT_LAUNCH_MOTOR_CONTROLLER = 17;
    }

   
}

