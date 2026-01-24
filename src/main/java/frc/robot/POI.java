package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.generated.ChoreoVars;
import frc.robot.util.AllianceFlipUtil;

public class POI {
    //public static final Supplier<Pose2d>  = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.);
    // ============= POSES =============
    public static final Supplier<Pose2d> L_Start = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.L_Start);
    public static final Supplier<Pose2d> R_Start = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.R_Start);
    public static final Supplier<Pose2d> L_Sweep = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.L_Sweep);
    public static final Supplier<Pose2d> R_Sweep = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.R_Sweep);
    public static final Supplier<Pose2d> C_ClimbPose = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.C_ClimbPose);
    public static final Supplier<Pose2d> R_ClimbPose = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.R_Climb_Pose);
    public static final Supplier<Pose2d> L_ClimbPose = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.L_ClimbPose);
    //public static final Supplier<Pose2d> testStart = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.testStart);
    //public static final Supplier<Pose2d> testEnd = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.testEnd);
    public static final Supplier<Pose2d> L_Trench = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.L_Trench);
    public static final Supplier<Pose2d> R_Trench = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.R_Trench);
    public static final Supplier<Pose2d> R_Sweep_Flip = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.R_Sweep_Flip);
    public static final Supplier<Pose2d> L_Sweep_Flip = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.L_Sweep_Flip);
    public static final Supplier<Pose2d> Testing2 = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.Testing2);
    public static final Supplier<Pose2d>  StationIntake = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.StationIntake);
    // ============= ROTATIONS =============
    public static final Supplier<Rotation2d> Degree_45 = () -> AllianceFlipUtil
            .flipRotation(new Rotation2d(Degrees.of(45)));
    public static final Supplier<Rotation2d> Degree_90 = () -> AllianceFlipUtil
            .flipRotation(new Rotation2d(Degrees.of(90)));
    public static final Supplier<Rotation2d> FLIP_45 = () -> AllianceFlipUtil
            .flipRotation(new Rotation2d(Degrees.of(-45)));
    public static final Supplier<Rotation2d> FLIP_90 = () -> AllianceFlipUtil
            .flipRotation(new Rotation2d(Degrees.of(-90)));
    public static final Supplier<Rotation2d> Degree_10 = () -> AllianceFlipUtil
            .flipRotation(new Rotation2d(Degrees.of(10)));
    public static final Supplier<Rotation2d> Degree_170 = () -> AllianceFlipUtil
            .flipRotation(new Rotation2d(Degrees.of(170)));   
    
}