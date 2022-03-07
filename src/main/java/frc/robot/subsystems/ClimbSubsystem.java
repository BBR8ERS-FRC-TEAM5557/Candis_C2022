package frc.robot.subsystems;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorControllers;;

/**
 * Add your docs here.
 */
public class ClimbSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public static ClimbSubsystem instance = null;

  public static ClimbSubsystem getInstance() {
    if (instance == null) {
      instance = new ClimbSubsystem();
    }
    return instance;
  }

  private CANSparkMax rightClimbMotor1, rightClimbMotor2, leftClimbMotor1, leftClimbMotor2;

  public ClimbSubsystem() {
		this.rightClimbMotor1 = new CANSparkMax(MotorControllers.RIGHT_CLIMB_MOTOR_CONTROLLER_1, MotorType.kBrushless);
    this.rightClimbMotor2 = new CANSparkMax(MotorControllers.RIGHT_CLIMB_MOTOR_CONTROLLER_2, MotorType.kBrushless);
    this.leftClimbMotor1 = new CANSparkMax(MotorControllers.LEFT_CLIMB_MOTOR_CONTROLLER_1, MotorType.kBrushless);
    this.leftClimbMotor2 = new CANSparkMax(MotorControllers.LEFT_CLIMB_MOTOR_CONTROLLER_2, MotorType.kBrushless);
	}

  public void climbUp() {
    rightClimbMotor1.set(0.2);
    rightClimbMotor2.set(-0.2);
    leftClimbMotor1.set(0.2);
    leftClimbMotor2.set(-0.2);
  }

  public void climbDown() {
    rightClimbMotor1.set(-0.2);
    rightClimbMotor2.set(0.2);
    leftClimbMotor1.set(-0.2);
    leftClimbMotor2.set(0.2);
  }

  public void climbStop() {
    rightClimbMotor1.set(0);
    rightClimbMotor2.set(0);
    leftClimbMotor1.set(0);
    leftClimbMotor2.set(0);
  }

}