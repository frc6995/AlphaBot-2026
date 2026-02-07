package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

public class IntakeRollerS extends SubsystemBase {
    public static class RollerConstants {
        public static int kCANID = 41;
    }   
    private TalonFX rollerMotor = new TalonFX(RollerConstants.kCANID, TunerConstants.kCANBus);
    
    // robot init, set voltage compensation to 12 V
// class member variable



 final VoltageOut m_request = new VoltageOut(2);

public Command VZVZ(){

    return Commands.run(() -> rollerMotor.setControl(m_request.withOutput(12)));
    //return Commands.parallel(rollerMotor.setControl(m_request.withOutput(Volts.of(12.0))));
}
// the control request `with` methods also accept unit types
//rollerMotor.setControl(m_request.withOutput(Volts.of(12.0)));
}