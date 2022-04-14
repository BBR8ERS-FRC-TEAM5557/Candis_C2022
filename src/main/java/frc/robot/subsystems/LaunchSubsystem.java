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
public class LaunchSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public static LaunchSubsystem instance = null;

  public static LaunchSubsystem getInstance() {
    if (instance == null) {
      instance = new LaunchSubsystem();
    }
    return instance;
  }

  private CANSparkMax leftLaunchMotor, rightLaunchMotor;

  public LaunchSubsystem() {
		this.leftLaunchMotor = new CANSparkMax(MotorControllers.LEFT_LAUNCH_MOTOR_CONTROLLER, MotorType.kBrushless);
    this.rightLaunchMotor = new CANSparkMax(MotorControllers.RIGHT_LAUNCH_MOTOR_CONTROLLER, MotorType.kBrushless);
        
	}

  public void launchUpper() {
    leftLaunchMotor.set(-0.7);
    rightLaunchMotor.set(0.7);
  }

  public void launchLower() {
    leftLaunchMotor.set(-0.6);
    rightLaunchMotor.set(0.6);
  }

  public void launch(double speed) {
    leftLaunchMotor.set(speed);
    rightLaunchMotor.set(-speed);
  }

  public void stopLaunch() {
    leftLaunchMotor.set(0);
    rightLaunchMotor.set(0);
  }


}