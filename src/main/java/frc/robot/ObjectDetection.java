package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import limelight.Limelight;
import limelight.networktables.LimelightResults;
import limelight.networktables.target.pipeline.NeuralDetector;

public class ObjectDetection {

    private final RobotContainer m_robotContainer;
    private final CommandSwerveDrivetrain m_drivetrain;
    private final Limelight m_limeLight;

    public ObjectDetection(RobotContainer robotContainer, CommandSwerveDrivetrain commandSwerveDrivetrain, Limelight limelight) {
        m_robotContainer = robotContainer;
        m_drivetrain = commandSwerveDrivetrain;
        m_limeLight = limelight;
    }


    public Optional<NeuralDetector[]> getTargetDetectors() {
        var results = m_limeLight.getLatestResults();

        return results.isPresent() ? Optional.of(results.get().targets_Detector) : Optional.empty();
        
    }

    public Pose2d getFirstFuelPose() {
        var results = getTargetDetectors();
        if (results.isPresent()) {
            NeuralDetector target = results.get()[0];
            double targetX = target.tx;
            double targetY = target.ty;
            return new Pose2d(targetX, targetY, null);
        }
        else {
            return new Pose2d();
        }
        
        
    }

    // public Pose2d getNearestFuelPose() {
    //     if (getTargetDetectors() != null) {
            
    //     }
    //     else {
    //         return new Pose2d();
    //     }
    // }


}
