package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.GameConstants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.Stopwatch;
import frc.robot.util.LimelightHelpers.RawDetection;
import limelight.Limelight;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightResults;
import limelight.networktables.target.pipeline.NeuralDetector;

public class ObjectDetection {

    private final RobotContainer m_robotContainer;
    private final CommandSwerveDrivetrain m_drivetrain;
    private final Limelight m_limeLight;

    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
	private ArrayList<StructPublisher<Pose2d>> publishers = new ArrayList<StructPublisher<Pose2d>>();
	private ArrayList<Fuel> tracker = new ArrayList<Fuel>();
	private Stopwatch mStopwatch = new Stopwatch();
	private final NetworkTable visTable = ntInstance.getTable("SmartDashboard/Detection");
	private final StructPublisher<Pose2d> closestFuelPose =
			visTable.getStructTopic("BestFuelPose", Pose2d.struct).publish();
	private final StructPublisher<Translation2d> closestFuelTranslation = visTable.getStructTopic(
					"BestFuelTranslation", Translation2d.struct)
			.publish();

    public static class VisionConstants {
        public static final String[] LL_IDS = {
            "limelight-frontll"
        };
        public static final Pose3d[] LL_OFFSETS = {
            new Pose3d( // frontLL
                new Translation3d(-0.0254,-0.0254,0.4826),
                new Rotation3d())
        };
        public static final EstimationMode kDefaultMode = EstimationMode.MEGATAG2;

    }

    class Fuel {
		Pose2d fuelPose;
		Translation2d fuelTranslation;
		double detectionTime;

		public Fuel(Pose2d fuelPose, Translation2d fuelTranslation, double detectionTime) {
			this.fuelPose = fuelPose;
			this.fuelTranslation = fuelTranslation;
			this.detectionTime = detectionTime;
		}
	}

    public ObjectDetection(RobotContainer robotContainer, CommandSwerveDrivetrain commandSwerveDrivetrain, Limelight limelight) {
        m_robotContainer = robotContainer;
        m_drivetrain = commandSwerveDrivetrain;
        m_limeLight = limelight;
    }

    public void update() {
		mStopwatch.startIfNotRunning();
			
		Translation2d base = m_drivetrain.state.Pose.getTranslation();
		Translation2d bestTranslation = null;
		Pose2d bestFuelPose = null;
		double now = Timer.getFPGATimestamp();
		tracker.removeIf((coral) -> now - coral.detectionTime > 0.2);

		while (tracker.size() > 20) {
			tracker.remove(0);
		}

        Optional<NeuralDetector[]> detectors = getTargetDetectors();
        try {
            for (NeuralDetector detector : detectors.get()) {
                double tx = detector.tx;
                double ty = detector.ty;                
                double ta = detector.ta;
                //Translation2d fuelTranslation = distToFuelCitrus(tx, ty); // subtract camera offset
                Translation2d fuelTranslation = distToFuel(tx, ty, ta);
                Pose2d FuelPose =                        m_drivetrain.state.Pose.transformBy(new Transform2d(fuelTranslation, new Rotation2d()));
                tracker.add(new Fuel(FuelPose, fuelTranslation, now));
            }
        } catch (Exception e) {
            System.out.println("no detectors");
        }
        System.out.println(tracker.size());
        

		for (Fuel fuel : tracker) {
			if (bestTranslation == null
					|| bestFuelPose.getTranslation().getDistance(base)
							> fuel.fuelPose.getTranslation().getDistance(base)) {
				bestTranslation = fuel.fuelTranslation;
				bestFuelPose = fuel.fuelPose;
			}
		}

		if (bestFuelPose != null) {
			closestFuelPose.set(bestFuelPose);
			closestFuelTranslation.set(bestTranslation);
		}	
		
	}

    public Pose2d getBestFuelPose(Translation2d base) {
		Translation2d bestTranslation = null;
		Pose2d bestFuelPose = null;
		for (Fuel fuel : tracker) {
			if (bestTranslation == null
					|| bestFuelPose.getTranslation().getDistance(base)
							> fuel.fuelPose.getTranslation().getDistance(base)) {
				bestTranslation = fuel.fuelTranslation;
				bestFuelPose = fuel.fuelPose;
			}
		}
		return bestFuelPose; // will return null if no coral
	}

    public Pose2d[] getFuelPoses() {
        ArrayList<Pose2d> poses = new ArrayList<>();
        for (Fuel fuel : tracker) {
            poses.add(fuel.fuelPose);
        }
        return poses.toArray(Pose2d[]::new);
    }

    public Optional<NeuralDetector[]> getTargetDetectors() {
        
        Optional<LimelightResults> results = m_limeLight.getLatestResults();

        return results.isPresent() ? Optional.of(results.get().targets_Detector) : Optional.empty();
        
    }

    public RawDetection[] getRawDetections() {
        RawDetection[] results = LimelightHelpers.getRawDetections(m_limeLight.limelightName);
        if (results != null) {
            return results;
        }
        else return null;
    }

    public Translation2d distToFuelCitrus(double tx, double ty) {

        double totalAngleY = Units.degreesToRadians(-ty); // subtract camera offset rotation
        Distance distAwayY = GameConstants.FUEL_DIAMETER.times(-1).div(Math.tan(totalAngleY));

        Distance distHypotenuseYToGround = BaseUnits.DistanceUnit.of(Math.hypot(
				distAwayY.in(BaseUnits.DistanceUnit),
				//config.robotToCameraOffset
				//		.getMeasureZ()
						GameConstants.FUEL_DIAMETER.times(-1)
						.in(BaseUnits.DistanceUnit)));

		double totalAngleX = Units.degreesToRadians(-tx);
				//+ config.robotToCameraOffset.getRotation().getZ();

        Distance distAwayX = distHypotenuseYToGround.times(Math.tan(totalAngleX)); // robot y

        return new Translation2d(distAwayY, distAwayX);
    }

    public Translation2d distToFuel(double tx, double ty, double ta) {
        // 10.5 inches = 280 px at 27.953 inches away    (280 px*27.953 in) / 10.5 in   F=745.41333333333333
        double focalLength = 745.4133;
        double width = Math.sqrt(ta);
        Distance distFromCam = GameConstants.FUEL_DIAMETER.times(focalLength).div(width);

        Distance distX = distFromCam.times(Math.sin(tx)).times(Math.cos(ty));
        Distance distY = distFromCam.times(Math.sin(tx)).times(Math.sin(ty));

        SmartDashboard.putNumber(m_limeLight.limelightName + "/Distance Away Y", distY.in(Meters));
		SmartDashboard.putNumber(m_limeLight.limelightName + "/Distance Away X", distX.in(Meters));

        return new Translation2d(distX, distY); 
    }


}
