package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Add your docs here.
 */
public class PneumaticSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  static PneumaticSubsystem instance;

  public Solenoid leftIntake, rightIntake;

  public static PneumaticSubsystem getInstance() {
    if (instance == null) {
      instance = new PneumaticSubsystem();
    }
    return instance;
  }

  public PneumaticSubsystem() {
		Solenoid leftIntake = new Solenoid(PneumaticsModuleType.REVPH, 0);
        
	}

  public void extendIntake() {
    leftIntake.set(false);
  }

  public void retractIntake() {
    leftIntake.set(true);
  }
}