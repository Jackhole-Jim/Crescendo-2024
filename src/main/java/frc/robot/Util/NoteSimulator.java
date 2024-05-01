// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class NoteSimulator extends Command {
    private boolean noteInRobot = false;
    private final double NOTE_INTAKE_MAX_DIST = 0.1;
    private Transform2d robotIntakeSpot = new Transform2d(0.375, 0, new Rotation2d());
    private Supplier<Pose2d> mRobotPoseSupplier;

    private List<Pose3d> noteLocations = new ArrayList<>(){
      {
        add(new Pose3d(2.9, 4.105 + 1.45 + 1.45, Units.inchesToMeters(1), new Rotation3d()));
        add(new Pose3d(2.9, 4.105 + 1.45, Units.inchesToMeters(1), new Rotation3d()));
        add(new Pose3d(2.9, 4.105, Units.inchesToMeters(1), new Rotation3d()));

        add(new Pose3d(8.27, 4.105 + 1.68 + 1.68, Units.inchesToMeters(1), new Rotation3d()));
        add(new Pose3d(8.27, 4.105 + 1.68, Units.inchesToMeters(1), new Rotation3d()));
        add(new Pose3d(8.27, 4.105, Units.inchesToMeters(1), new Rotation3d()));
        add(new Pose3d(8.27, 4.105 - 1.68, Units.inchesToMeters(1), new Rotation3d()));
        add(new Pose3d(8.27, 4.105 - 1.68 - 1.68, Units.inchesToMeters(1), new Rotation3d()));

        add(new Pose3d(13.64, 4.105 + 1.45 + 1.45, Units.inchesToMeters(1), new Rotation3d()));
        add(new Pose3d(13.64, 4.105 + 1.45, Units.inchesToMeters(1), new Rotation3d()));
        add(new Pose3d(13.64, 4.105, Units.inchesToMeters(1), new Rotation3d()));
      }
    };

  /** Creates a new NoteSimulator. */
  public NoteSimulator(Supplier<Pose2d> robotPoseSupplier,
    Supplier<Double> intakeSetpointSupplier, 
    Supplier<Double> shooterSetpointSupplier) {
    mRobotPoseSupplier = robotPoseSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose3d intakeSpot = new Pose3d(mRobotPoseSupplier.get().plus(robotIntakeSpot));

    Pose3d closestNote = noteLocations
      .stream()
      .map((pose) -> new Pair<>(pose, intakeSpot))
      .min((posePair1, posePair2) -> { 
        return (int)((posePair1.getFirst().getTranslation().getDistance(posePair1.getSecond().getTranslation())
                - posePair2.getFirst().getTranslation().getDistance(posePair2.getSecond().getTranslation()))
                * 1000); 
      })
      .map((posePair) -> posePair.getFirst())
      .get();

    Logger.recordOutput("NoteSimulation/ClosestNoteToIntakeDist", intakeSpot.getTranslation().getDistance(closestNote.getTranslation()));
    Logger.recordOutput("NoteSimulation/IntakeSpot", intakeSpot);
    Logger.recordOutput("NoteSimulation/Notes", noteLocations.toArray(Pose3d[]::new));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
