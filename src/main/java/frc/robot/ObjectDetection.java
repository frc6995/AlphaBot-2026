package frc.robot;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.List;
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
    private ArrayList<Fuel> tracker = new ArrayList<Fuel>();
    private Stopwatch mStopwatch = new Stopwatch();
    private final NetworkTable visTable = ntInstance.getTable("SmartDashboard/Detection");
    private final StructPublisher<Pose2d> closestFuelPose = visTable.getStructTopic("BestFuelPose", Pose2d.struct)
            .publish();
    private final StructPublisher<Pose2d> averageFuelPose = visTable.getStructTopic(
            "AverageFuelPose", Pose2d.struct)
            .publish();

    public static class VisionConstants {
        public static final String[] LL_IDS = {
                "limelight-frontll"
        };
        public static final Pose3d[] LL_OFFSETS = {
                new Pose3d( // frontLL
                        new Translation3d(0, 0, 0.46355),
                        Rotation3d.kZero)
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

    public ObjectDetection(RobotContainer robotContainer, CommandSwerveDrivetrain commandSwerveDrivetrain,
            Limelight limelight) {
        m_robotContainer = robotContainer;
        m_drivetrain = commandSwerveDrivetrain;
        m_limeLight = limelight;

    }

    public void update() {
        mStopwatch.startIfNotRunning();

        Translation2d base = m_drivetrain.state.Pose.getTranslation();
        Translation2d bestTranslation = null;
        Translation2d averageTranslation = null;
        Pose2d bestFuelPose = null;
        double now = Timer.getFPGATimestamp();
        tracker.removeIf((coral) -> now - coral.detectionTime > 0.1);

        while (tracker.size() > 20) {
            tracker.remove(0);
        }

        Optional<NeuralDetector[]> detectors = getTargetDetectors();
        try {
            for (NeuralDetector detector : detectors.get()) {
                double tx = detector.tx_nocrosshair;
                double ty = detector.ty_nocrosshair;
                double ta = detector.ta;
                Translation2d fuelTranslation = distToFuelCitrus(tx, ty)
                        .minus(VisionConstants.LL_OFFSETS[0].getTranslation().toTranslation2d());
                Pose2d FuelPose = m_drivetrain.state.Pose
                        .transformBy(new Transform2d(fuelTranslation, new Rotation2d()));
                tracker.add(new Fuel(FuelPose, fuelTranslation, now));
            }
        } catch (Exception e) {

        }

        bestFuelPose = getBestFuelPose(base);

        averageTranslation = getBestAverageFuelPose(m_robotContainer.m_drivetrain.state.Pose.getTranslation()).getTranslation();
        averageFuelPose.set(new Pose2d(averageTranslation, new Rotation2d()));
        if (bestFuelPose != null) {
            bestTranslation = bestFuelPose.getTranslation();
            closestFuelPose.set(bestFuelPose);
        }

    }

    public Pose2d getBestFuelPose(Translation2d base) {
        Translation2d bestTranslation = null;
        Pose2d bestFuelPose = null;
        for (Fuel fuel : tracker) {
            if (bestTranslation == null
                    || bestFuelPose.getTranslation().getDistance(base) > fuel.fuelPose.getTranslation()
                            .getDistance(base)) {
                bestTranslation = fuel.fuelTranslation;
                bestFuelPose = fuel.fuelPose;
            }
        }
        return bestFuelPose; // will return null if no coral
    }

    public Pose2d getAverageFuelPose(Translation2d drivebaseTranslation) {
        Distance ingnoreThreshold = Meters.of(1);
        ArrayList<Translation2d> validFuelTranslations = new ArrayList<Translation2d>();
        for (Fuel fuel : tracker) {
            if (fuel.fuelTranslation.getDistance(drivebaseTranslation) < ingnoreThreshold.magnitude()) {
                validFuelTranslations.add(fuel.fuelTranslation);
            } else
                continue;
        }
        Translation2d sum = new Translation2d();
        for (Translation2d translation2d : validFuelTranslations) {
            sum = sum.plus(translation2d);
        }
        Translation2d averageTranslation = sum.div(validFuelTranslations.size());
        return new Pose2d(averageTranslation, new Rotation2d());
    }

    // TODO: test and fix this method
    public Pose2d getBestAverageFuelPose(Translation2d driveBaseTranslation) {
        Translation2d currentBest = null;

        for (Fuel fuel : tracker) {
            if (currentBest == null)
                currentBest = fuel.fuelTranslation;

            ArrayList<Fuel> valids = new ArrayList<Fuel>();
            for (Fuel f : tracker) {
                if (fuel != f && fuel.fuelTranslation.getDistance(f.fuelTranslation) 
                            <= Meters.of(0.4).magnitude()) {
                    valids.add(f);
                }
            }
            Translation2d summedValids = new Translation2d();
            for (Fuel f : valids) {
                summedValids.plus(f.fuelTranslation);
            }
            Translation2d newAverage = summedValids.div(valids.size());
            if (newAverage.getDistance(driveBaseTranslation) < currentBest.getDistance(driveBaseTranslation)
                    && newAverage.getDistance(driveBaseTranslation) > new Translation2d(0.1,0.1).getDistance(driveBaseTranslation)) {
                currentBest = newAverage;
            }
        }

        return new Pose2d(currentBest, new Rotation2d());
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
        //System.out.println("Has Results: " + results.isPresent());
        return results.isPresent() ? Optional.of(results.get().targets_Detector) : Optional.empty();

    }

    public RawDetection[] getRawDetections() {
        RawDetection[] results = LimelightHelpers.getRawDetections(m_limeLight.limelightName);
        if (results != null) {
            return results;
        } else
            return null;
    }

    public Translation2d distToFuelCitrus(double tx, double ty) {
        double totalAngleY = Units.degreesToRadians(-ty) - VisionConstants.LL_OFFSETS[0].getRotation().getY();
        Distance distAwayY = VisionConstants.LL_OFFSETS[0].getMeasureZ().minus((GameConstants.FUEL_DIAMETER.div(2)).div(Math.tan(totalAngleY)));

        Distance distHypotenuseYToGround = BaseUnits.DistanceUnit.of(Math.hypot(
				distAwayY.in(BaseUnits.DistanceUnit),
				VisionConstants.LL_OFFSETS[0]
						.getMeasureZ()
						.minus(GameConstants.FUEL_DIAMETER.div(2))
						.in(BaseUnits.DistanceUnit)));

        double totalAngleX = Units.degreesToRadians(-tx)
                + VisionConstants.LL_OFFSETS[0].getRotation().getZ();

        Distance distAwayX = distHypotenuseYToGround.times(Math.tan(totalAngleX)); // robot y

        return new Translation2d(distAwayY, distAwayX);
    }

    public Translation2d distToFuel(double tx, double ty) {
        double distX = (VisionConstants.LL_OFFSETS[0].getZ() - GameConstants.FUEL_DIAMETER.in(Meters)) * Math.tan(-ty);
        double distY = distX * Math.tan(tx);
        return new Translation2d(distX, distY);
    }

    // public Translation2d distToFuel(double tx, double ty, double ta) {
    //     // 10.5 inches = 280 px at 27.953 inches away (280 px*27.953 in) / 10.5 in
    //     // F=745.41333333333333
    //     double focalLength = 745.4133;
    //     double width = Math.sqrt(ta);
    //     Distance distFromCam = GameConstants.FUEL_DIAMETER.times(focalLength).div(width);

    //     Distance distX = distFromCam.times(Math.sin(tx)).times(Math.cos(ty));
    //     Distance distY = distFromCam.times(Math.sin(tx)).times(Math.sin(ty));

    //     SmartDashboard.putNumber(m_limeLight.limelightName + "/Distance Away Y", distY.in(Meters));
    //     SmartDashboard.putNumber(m_limeLight.limelightName + "/Distance Away X", distX.in(Meters));

    //     return new Translation2d(distX, distY);
    // }

}
