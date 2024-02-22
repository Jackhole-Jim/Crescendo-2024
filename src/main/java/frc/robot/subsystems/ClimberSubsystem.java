// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  private final CANSparkMax climberMotor = new CANSparkMax(Constants.climberConstants.CLIMBER_MOTOR_ID, MotorType.kBrushless);
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Climber/ClimberSpeed",climberMotor.get());
    Logger.recordOutput("Climber/ClimberPosition",climberMotor.getEncoder().getPosition());
  }
  public Command setClimberSpeed(DoubleSupplier Double_Supplier){
    return run(() -> {
      climberMotor.set(Double_Supplier.getAsDouble());
    });
  }
}
