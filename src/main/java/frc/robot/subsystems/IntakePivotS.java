package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class IntakePivotS extends SubsystemBase {
    public class intakeConstants {
    public static final Distance OFFSET_X = Inches.of(5.6);
    public static final Distance OFFSET_Y = Inches.of(0);
    public static final Distance OFFSET_Z = Inches.of(4);

    public static final Angle CW_LIMIT = Degrees.of(-10);
    public static final Angle CCW_LIMIT = Degrees.of(95);

    public static final Angle ALGAE_INTAKE = Degrees.of(45);
    public static final Angle STOW = Degrees.of(95);
    public static final Angle L1_POST_SCORE = Degrees.of(80);
    public static final Angle ALGAE_POST_SCORE = Degrees.of(70);

    public static final double KP = 18;
    public static final double KI = 0;
    public static final double KD = 0.2;
    public static final double KS = -0.1;
    public static final double KG = 1.2;
    public static final double KV = 0;
    public static final double KA = 0;
    public static final double VELOCITY = 458;
    public static final double ACCELERATION = 688;
    public static final int MOTOR_ID = 40;
    public static final double STATOR_CURRENT_LIMIT = 120;
    public static final double MOI = 48.569;
    public static Angle L1_ANGLE;
  }

  private SmartMotorControllerConfig smcConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          // Feedback Constants (PID Constants)
          .withClosedLoopController(
              14, 0, 0.02, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
          .withSimClosedLoopController(
              14, 0, 0.02, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
          // Feedforward Constants
          .withFeedforward(new ArmFeedforward(0, .605, .464))
          .withSimFeedforward(new ArmFeedforward(0, .605, .464))
          // Telemetry name and verbosity level
          .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH)
          // Gearing from the motor rotor to final shaft.
          // In this example GearBox.fromReductionStages(3,4) is the same as
          // GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your
          // motor.
          // You could also use .withGearing(12) which does the same thing.
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
          // Motor properties to prevent over currenting.
          .withMotorInverted(true)
          .withIdleMode(MotorMode.BRAKE)
          .withStatorCurrentLimit(Amps.of(40))
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25));

  private TalonFX armMotor = new TalonFX(40, TunerConstants.kCANBus);


  private SmartMotorController TalonFXSmartMotorController =
      new TalonFXWrapper(armMotor, DCMotor.getKrakenX44(1), smcConfig);

  private ArmConfig armCfg =
      new ArmConfig(TalonFXSmartMotorController)
          .withSoftLimits(Degrees.of(-25), Degrees.of(141))
          .withHardLimit(Degrees.of(-25), Degrees.of(141))
          .withStartingPosition(Degrees.of(-25))
          .withLength(Inches.of(10.5))
          .withMass(Pounds.of(3.875))
          .withTelemetry("Arm", TelemetryVerbosity.HIGH);

  private Arm arm = new Arm(armCfg);

/* 
  public IntakePivotS(){
  TalonFXConfiguration configs = new TalonFXConfiguration();
  configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
  armMotor.getConfigurator().apply(configs);
}
*/
  

  // set arm angle
  public Command setAngle(Angle angle) {
    return arm.setAngle(angle);
  }

  // move the arm
  public Command set(double dutycycle) {
    return arm.set(dutycycle);
  }

  public Command sysId() {
    return arm.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
  }

  @Override
  public void periodic() {
    arm.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    arm.simIterate();
  }
}
