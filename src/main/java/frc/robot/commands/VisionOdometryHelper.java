// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class VisionOdometryHelper extends Command {
  private List<PhotonPoseEstimator> photonPoseEstimators;
  private SwerveSubsystem mDrivetrainSubsystem;
  // private InterpolatingTreeMap<Double, Double> interp = new InterpolatingTreeMap<>();
  private boolean errored = false;
  private static final double xyStdDevCoefficient = 0.1;
  /** Creates a new VisionOdometryHelper. */
  public VisionOdometryHelper(SwerveSubsystem drivetrainSubsystem) {
    mDrivetrainSubsystem = drivetrainSubsystem;
    // interp.put(1.0, 0.7);
    // interp.put(7.5, 4.0);

    try {
      photonPoseEstimators = new ArrayList<PhotonPoseEstimator>() {
        {
          add(new PhotonPoseEstimator(
              AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
              PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
              new PhotonCamera("BackAprilTagCamera"),
              new Transform3d(new Translation3d(-0.14605, -0.2286, 0.635), new Rotation3d(0, -0.785398, Math.PI))
            )
          );
          add(new PhotonPoseEstimator(
              AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
              PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
              new PhotonCamera("AprilTagCameraLeft"),
              new Transform3d(new Translation3d(0, 0.254, 0.2794), new Rotation3d(0, 0.523599, Math.PI/2))
            )
          );
          add(new PhotonPoseEstimator(
              AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
              PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
              new PhotonCamera("AprilTagCameraRight"),
              new Transform3d(new Translation3d(0, -0.254, 0.2794), new Rotation3d(0, -0.523599, -Math.PI/2))
            )
          );
        }
      };
      photonPoseEstimators.forEach(x -> {
        x.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
        x.setTagModel(TargetModel.kAprilTag36h11);
      });
    } catch (Exception e) {
      errored = true;
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(errored)
    {
      this.cancel();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(DriverStation.isTeleopEnabled())
    // {
      List<Double> averageDistanceToTargetList = new ArrayList<>();
      List<Double> averageAmbiguityList = new ArrayList<>();
      List<PhotonTrackedTarget> trackedTargets = new ArrayList<>();
      List<Pose2d> visionOdometryList = new ArrayList<>();

      getEstimatedGlobalPoses(mDrivetrainSubsystem.getPose())
        .forEach(y -> 
            y.ifPresentOrElse((EstimatedRobotPose pose) -> {
              // mDrivetrainSubsystem.resetPose(estimateCurrentPoseFromPastPose(pose.estimatedPose.toPose2d(),
              // pose.timestampSeconds, mDrivetrainSubsystem.getPrevTransforms()));
              double averageDistanceToTarget = pose.targetsUsed
                .stream()
                .map(x -> x.getBestCameraToTarget().getTranslation())
                .mapToDouble(Translation3d::getNorm)
                .average()
                .getAsDouble();
              double averageAmbiguity = pose.targetsUsed
                .stream()
                .mapToDouble(x -> x.getPoseAmbiguity())
                .average()
                .getAsDouble();

              averageDistanceToTargetList.add(averageDistanceToTarget);
              averageAmbiguityList.add(averageAmbiguity);
              // Logger.recordOutput("Vision/averageDistanceToTarget", averageDistanceToTarget);
              // Logger.recordOutput("Vision/averageAmbiguity", averageAmbiguity);
              // double poseSTD =
              // interp.get(averageDistanceToTarget)/Math.pow(pose.targetsUsed.size(), 3);
              double poseSTD = (xyStdDevCoefficient * Math.pow(averageDistanceToTarget, 2)) 
                * (1 / pose.targetsUsed.size())
                * (averageAmbiguity * 10);
            
              mDrivetrainSubsystem.addVisionMeasurement(
                pose.estimatedPose.toPose2d(), 
                pose.timestampSeconds,
                VecBuilder.fill(poseSTD, poseSTD, Double.MAX_VALUE)
              );
              visionOdometryList.add(pose.estimatedPose.toPose2d());
              trackedTargets.addAll(pose.targetsUsed);
              // SmartDashboard.putNumber("Pose updated", pose.timestampSeconds);
              // Logger.recordOutput("Vision/VisionOdometry", pose.estimatedPose.toPose2d());
                // .stream()
                // .map(x -> photonPoseEstimators.get(0).getFieldTags().getTagPose(x.getFiducialId()).get())
                // .toList()
                // .toArray(new Pose3d[0])
              // );
            },
            () -> {
              
              averageDistanceToTargetList.add(0.0);
              averageAmbiguityList.add(0.0);
              visionOdometryList.add(new Pose2d());
              // Logger.recordOutput("Vision/VisionOdometry", new Pose2d());
              // Logger.recordOutput("Vision/VisionTargets", new Pose3d[0]);
              // Logger.recordOutput("Vision/averageDistanceToTarget", 0);
              // Logger.recordOutput("Vision/averageAmbiguity", 0);
            }
          )
        );

        Logger.recordOutput("Vision/VisionTargets", trackedTargets
          .stream()
          .distinct()
          .map(x -> photonPoseEstimators.get(0).getFieldTags().getTagPose(x.getFiducialId()).get())
          .toList()
          .toArray(Pose3d[]::new)
        );
        Logger.recordOutput("Vision/averageDistanceToTarget", averageDistanceToTargetList.stream().mapToDouble(x -> x).toArray());
        Logger.recordOutput("Vision/averageAmbiguity", averageAmbiguityList.stream().mapToDouble(x -> x).toArray());
        Logger.recordOutput("Vision/VisionOdometry", visionOdometryList.toArray(Pose2d[]::new));
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private List<Optional<EstimatedRobotPose>> getEstimatedGlobalPoses(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimators.forEach(x -> x.setReferencePose(prevEstimatedRobotPose));
    return photonPoseEstimators.stream().map(x -> x.update()).toList();
  }
}
