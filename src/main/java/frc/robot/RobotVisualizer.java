package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.IntakePivotS;

public class RobotVisualizer {

  final static double INTAKE_X = Units.inchesToMeters(10.625);
  final static double INTAKE_Z = Units.inchesToMeters(8.875);
  final static Pose3d INTAKE_PIVOT_LOCATION = new Pose3d(INTAKE_X, 0, INTAKE_Z, Rotation3d.kZero);
  private static Pose3d[] components = new Pose3d[] { Pose3d.kZero };
  private static final StructArrayPublisher<Pose3d> layoutPub = NetworkTableInstance.getDefault()
      .getStructArrayTopic("Visualizer/Components", Pose3d.struct)
      .publish();
  public static Pose3d[] getComponents() {return components;}
  public static void updateIntake(double intakeRadians) {
    components[0] = INTAKE_PIVOT_LOCATION.transformBy(new Transform3d(Translation3d.kZero, new Rotation3d(0,-intakeRadians,0)));
    layoutPub.set(components);
  }
}