// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.Constants;

public class NoteSimulator extends Command {
    private boolean noteInRobot = false;
    private boolean shootingTimerRunning = false;
    private final double NOTE_INTAKE_MAX_DIST = 0.3;
    private final double NOTE_SHOOTING_TIME = 0.5;
    private Transform2d robotIntakeSpot = new Transform2d(0.375, 0, new Rotation2d());

  

    private Supplier<Pose2d> mRobotPoseSupplier;
    private Supplier<Boolean> mNoteIntakingSupplier;
    private Supplier<Boolean> mNoteShootingSupplier;

    private Timer timer = new Timer();

    private List<Pose3d> noteLocations = new ArrayList<>(){
      {
      //   add(Constants.FieldConstants.BLUE_1_NOTE);
      //   add(Constants.FieldConstants.BLUE_2_NOTE);
      //   add(Constants.FieldConstants.BLUE_3_NOTE);

        add(Constants.FieldConstants.CENTER_1_NOTE);
        add(Constants.FieldConstants.CENTER_2_NOTE);
        // add(Constants.FieldConstants.CENTER_3_NOTE);
        // add(Constants.FieldConstants.CENTER_4_NOTE);
        // add(Constants.FieldConstants.CENTER_5_NOTE);

        // add(Constants.FieldConstants.RED_1_NOTE);
        // add(Constants.FieldConstants.RED_2_NOTE);
        // add(Constants.FieldConstants.RED_3_NOTE);
      }
    };

  /** Creates a new NoteSimulator. */
  public NoteSimulator(Supplier<Pose2d> robotPoseSupplier,
    Supplier<Boolean> noteIntakingSupplier, 
    Supplier<Boolean> noteShootingSupplier) {
    mRobotPoseSupplier = robotPoseSupplier;
    mNoteIntakingSupplier = noteIntakingSupplier;
    mNoteShootingSupplier = noteShootingSupplier;
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
      .orElseGet(() -> new Pose3d());

    double closestNoteToIntakeDist = intakeSpot.getTranslation().getDistance(closestNote.getTranslation());


    if(this.noteInRobot)
    {
      if(mNoteShootingSupplier.get())
      {
        if(!this.shootingTimerRunning)
        {
          timer.start();
          this.shootingTimerRunning = true;
        }
        else
        {
          if(timer.hasElapsed(NOTE_SHOOTING_TIME))
          {
            timer.reset();
            this.noteInRobot = false;
            this.shootingTimerRunning = false;
          }
        }
      }
      else
      {
        timer.reset();
        this.shootingTimerRunning = false;
      }
    }
    else
    {
      if(closestNoteToIntakeDist < this.NOTE_INTAKE_MAX_DIST && mNoteIntakingSupplier.get())
      {
        noteLocations.remove(closestNote);
        this.noteInRobot = true;
      }
    }


    Logger.recordOutput("NoteSimulation/ClosestNoteToIntakeDist", closestNoteToIntakeDist);
    Logger.recordOutput("NoteSimulation/NoteInRobot", this.noteInRobot);
    Logger.recordOutput("NoteSimulation/IntakeSpot", intakeSpot);

    if(!noteLocations.isEmpty())
    {
      Logger.recordOutput("NoteSimulation/Notes", noteLocations.toArray(Pose3d[]::new));
    }
    else
    {
      Logger.recordOutput("NoteSimulation/Notes", new Pose3d[]{});
    }
  }

  public Supplier<Boolean> noteBeamBreakSimulation()
  {
    return () -> {
      return noteInRobot;
    };
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public boolean runsWhenDisabled(){
    return true;
  }
olean isFinished() {
    return false;
  }

