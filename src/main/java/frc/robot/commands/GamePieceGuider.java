// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import java.util.List;
import java.util.Set;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.GamePieceVisionSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class GamePieceGuider extends DeferredCommand {
  
  private static double RADIUS_TO_SEARCH = 1.0;
  /** Creates a new GamePieceGuider. */
  public GamePieceGuider(SwerveSubsystem drivetrainSubsystem, IntakeSubsystem intakeSubsystem, LEDSubsystem ledSubsystem, GamePieceVisionSubsystem gamePieceVisionSubsystem, CommandXboxController driverController, Pose3d areaToSearch) {
    this(drivetrainSubsystem, intakeSubsystem, ledSubsystem, gamePieceVisionSubsystem, driverController, areaToSearch.toPose2d());
  }


  public GamePieceGuider(SwerveSubsystem drivetrainSubsystem, IntakeSubsystem intakeSubsystem, LEDSubsystem ledSubsystem, GamePieceVisionSubsystem gamePieceVisionSubsystem, CommandXboxController driverController, Pose2d areaToSearch) {
    super(() -> {
      List<Pose3d> estimatedPose3ds = gamePieceVisionSubsystem.getEstimatedGamePieceLocations();

      if(!estimatedPose3ds.isEmpty())
      {
        Pose2d nearest = areaToSearch.nearest(estimatedPose3ds.stream().map(x -> x.toPose2d()).toList());
  
        Translation2d robotToNoteTranslation = nearest.getTranslation().minus(drivetrainSubsystem.getPose().getTranslation());
  
        if(areaToSearch.relativeTo(nearest).getTranslation().getNorm() < RADIUS_TO_SEARCH && !intakeSubsystem.IsNotePresent())
        {
          return new IntakeNoteCommand(intakeSubsystem, ledSubsystem, drivetrainSubsystem, driverController)
             .raceWith(drivetrainSubsystem.driveToPose(new Pose2d(nearest.getTranslation(), new Rotation2d(robotToNoteTranslation.getX(), robotToNoteTranslation.getY()))))
            .andThen(new StopIntakeCommand(intakeSubsystem, driverController));
        }
        else
        {
          return new InstantCommand();
        }
      }
      else
      {
        return new InstantCommand();
      }

    }, 
    Set.of(drivetrainSubsystem, intakeSubsystem, ledSubsystem, gamePieceVisionSubsystem));
  }
}
