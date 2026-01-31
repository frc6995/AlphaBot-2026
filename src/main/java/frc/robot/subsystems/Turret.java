package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.io.ObjectInputFilter.Config;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Turret extends SubsystemBase {

  public static class TurretConstants {
     public static Angle halfRotation = Degrees.of(180);
     public static Angle full = Degrees.of(360);
     public static Angle resetAngle = Degrees.of(0);
  }

 
  private final TalonFX turretMotor = new TalonFX(60, TunerConstants.kCANBus);
  private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withClosedLoopController(40, 0, 5, DegreesPerSecond.of(1000), DegreesPerSecondPerSecond.of(1000))
      .withSoftLimit(Degrees.of(-30), Degrees.of(100))
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(10.5)))
      .withIdleMode(MotorMode.BRAKE)
      .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
      .withStatorCurrentLimit(Amps.of(40))
      .withMotorInverted(false)
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25))
      .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
      .withControlMode(ControlMode.CLOSED_LOOP);
  private final SmartMotorController  TurretmotorSMC  = new TalonFXWrapper(turretMotor,DCMotor.getKrakenX60(1), motorConfig);

  private final MechanismPositionConfig  robotToMechanism = new MechanismPositionConfig()
      .withMaxRobotHeight(Meters.of(1.5))
      .withMaxRobotLength(Meters.of(0.75))
      .withRelativePosition(new Translation3d(Meters.of(-0.25), Meters.of(0), Meters.of(0.5)));
  private final PivotConfig  m_config  = new PivotConfig(TurretmotorSMC)
      .withHardLimit(Degrees.of(0), Degrees.of(360))
      .withTelemetry("TurretExample", TelemetryVerbosity.HIGH)
      .withStartingPosition(Degrees.of(0))
      .withMechanismPositionConfig(robotToMechanism)
      .withMOI(Meters.of(0.25), Pounds.of(4));
  private final Pivot turret = new Pivot(m_config);

  @Override
  public void periodic(){
    turret.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    turret.simIterate();
  }

  public Command turretCmd(double dutycycle)  {
    return turret.set(dutycycle);
  }
  public Command sysId() {
    return turret.sysId(Volts.of(3), Volts.of(3).per(Second), Second.of(30));
  }

  public Command setAngle(Angle angle) {
    return turret.setAngle(angle);
  }

  public Command fullRotation(Angle angle) {
    return turret.setAngle(angle);
  }
  public Command reset(Angle angle) {
    return turret.setAngle(angle);
  }

      /**
     * Sets the turret motor voltage.
     * 
     * @param voltage (as a double)
     * @return
     */
    public Command setVoltage(Voltage voltage) {
        return turret.setVoltage(voltage);
    }

    public Command setVoltage(Supplier<Voltage> voltageSupplier) {
        return turret.setVoltage(voltageSupplier);
    }

        public Current getSupplyCurrent() {
        return turretMotor.getSupplyCurrent().getValue();
    }

    public Command driveToHome() {
    return Commands.sequence(
      setVoltage(Volts.of(-1.0)).until(()-> getSupplyCurrent().magnitude() > 40),
      this.runOnce(()->turretMotor.getConfigurator().setPosition(Degrees.of(0))).ignoringDisable(true)
      
    ).withTimeout(1.0).andThen(setVoltage(Volts.of(0)));
  }

}