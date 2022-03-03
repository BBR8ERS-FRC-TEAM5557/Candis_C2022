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
public class IntakeSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public static IntakeSubsystem instance = null;

  public static IntakeSubsystem getInstance() {
    if (instance == null) {
      instance = new IntakeSubsystem();
    }
    return instance;
  }

  private CANSparkMax intakeMotor, storeMotor;

  public IntakeSubsystem() {
		//this.intakeMotor = new CANSparkMax(MotorControllers.INTAKE_MOTOR_CONTROLLER, MotorType.kBrushless);
    //this.storeMotor = new CANSparkMax(MotorControllers.STORE_MOTOR_CONTROLLER, MotorType.kBrushless);
        
	}

  public void spinIntakeIn() {
    intakeMotor.set(0.5);
  }

  public void spinIntakeOut() {
    intakeMotor.set(-0.5);
  }

  public void stopSpinIntake() {
    intakeMotor.set(0);
  }

  public void spinStore(){
    storeMotor.set(0.5);
  }

}