package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * Add your docs here.
 */
public class PneumaticSubsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public Solenoid leftIntake, rightIntake;

  public PneumaticSubsystem() {
		//Solenoid leftIntake = new Solenoid(PneumaticsModuleType.REVPH, 0);
        
	}

  public void extendIntake() {
    leftIntake.set(false);
  }

  public void retractIntake() {
    leftIntake.set(true);
  }
}