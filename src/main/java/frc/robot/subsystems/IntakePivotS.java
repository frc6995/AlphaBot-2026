package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotVisualizer;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher; 

public class IntakePivotS extends SubsystemBase {
    public class intakeConstants {
    public static final Distance OFFSET_X = Meters.of(0.175);
    public static final Distance OFFSET_Y = Meters.of(0);
    public static final Distance OFFSET_Z = Meters.of(-0.06);

        // 0.175,
        // 0.0,
        // -0.06


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
    public static final double MOI = 0.0855457256;
    public static Angle L1_ANGLE;
  }
    public Pose3d intakePose = new Pose3d(new Translation3d(intakeConstants.OFFSET_X, intakeConstants.OFFSET_Y, intakeConstants.OFFSET_Z), new Rotation3d(0, 0, 90));

  private SmartMotorControllerConfig smcConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          // Feedback Constants (PID Constants)
          .withClosedLoopController(
              50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
          .withSimClosedLoopController(
              50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
          // Feedforward Constants
          .withFeedforward(new ArmFeedforward(0, 0, 0))
          .withSimFeedforward(new ArmFeedforward(0, 0, 0))
          // Telemetry name and verbosity level
          .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH)
          // Gearing from the motor rotor to final shaft.
          // In this example GearBox.fromReductionStages(3,4) is the same as
          // GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your
          // motor.
          // You could also use .withGearing(12) which does the same thing.
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
          // Motor properties to prevent over currenting.
          .withMotorInverted(false)
          .withIdleMode(MotorMode.BRAKE)
          .withStatorCurrentLimit(Amps.of(40))
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25));

  private TalonFX armMotor = new TalonFX(0);

  private SmartMotorController TalonFXSmartMotorController =
      new TalonFXWrapper(armMotor, DCMotor.getNEO(1), smcConfig);
  private final MechanismPositionConfig robotToMechanism = new MechanismPositionConfig()
    .withRelativePosition(new Translation3d(intakeConstants.OFFSET_X, intakeConstants.OFFSET_Y, intakeConstants.OFFSET_Z));
  private ArmConfig armCfg =
      new ArmConfig(TalonFXSmartMotorController)
          .withSoftLimits(Degrees.of(-20), Degrees.of(10))
          .withHardLimit(Degrees.of(-30), Degrees.of(40))
          .withStartingPosition(Degrees.of(-5))
          .withLength(Feet.of(3))
          .withMass(Pounds.of(1))
          .withTelemetry("Arm", TelemetryVerbosity.HIGH)
          .withMechanismPositionConfig(robotToMechanism);

  private Arm arm = new Arm(armCfg);

  // public IntakePivotS() {
  //       NetworkTableInstance.getDefault().getEntry("pivotPose").setValue(pivotPose);
  // }
  // private final StructPublisher<Pose3d> pivotPosePub = NetworkTableInstance.getDefault()
  //       .getStructTopic("pivotPose", Pose3d.struct)
  //       .publish();
  // set arm angle
  public Command setAngle(Angle angle) {
    return arm.setAngle(angle);
  }
  public Angle getAngle() {
    return arm.getAngle();
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

    if (arm != null) {

      arm.updateTelemetry();
      double currentAngleRad = arm.getAngle().in(edu.wpi.first.units.Units.Radians);
      RobotVisualizer.updateIntake(currentAngleRad);

    }
  }

  @Override
  public void simulationPeriodic() {
      if (arm != null) {
          arm.simIterate();
      }
  }
}
