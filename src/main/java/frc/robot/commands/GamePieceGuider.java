// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.GamePieceVisionSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class GamePieceGuider extends DeferredCommand {
  
  private static double RADIUS_TO_SEARCH = 1.0;
  /** Creates a new GamePieceGuider. */
  public GamePieceGuider(SwerveSubsystem drivetrainSubsystem, Command intakeCommand, Command stopIntakeCommand, GamePieceVisionSubsystem gamePieceVisionSubsystem, Pose3d areaToSearch) {
    this(drivetrainSubsystem, intakeCommand, stopIntakeCommand, gamePieceVisionSubsystem, areaToSearch.toPose2d());
  }


  public GamePieceGuider(SwerveSubsystem drivetrainSubsystem, Command intakeCommand, Command stopIntakeCommand, GamePieceVisionSubsystem gamePieceVisionSubsystem, Pose2d areaToSearch) {
    super(() -> {
      Pose2d nearest = areaToSearch.nearest(gamePieceVisionSubsystem.getEstimatedGamePieceLocations().stream().map(x -> x.toPose2d()).toList());

      if(areaToSearch.relativeTo(nearest).getTranslation().getNorm() < RADIUS_TO_SEARCH)
      {
        return (intakeCommand
            .alongWith(drivetrainSubsystem.driveToPose(nearest))
          )
          .andThen(stopIntakeCommand);
      }
      else
      {
        return new InstantCommand();
      }
    }, 
    Set.of(drivetrainSubsystem, gamePieceVisionSubsystem));

  }
}
