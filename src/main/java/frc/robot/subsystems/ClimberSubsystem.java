// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  private final CANSparkMax climberMotor = new CANSparkMax(Constants.ClimberConstants.CLIMBER_MOTOR_ID, MotorType.kBrushless);
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    climberMotor.restoreFactoryDefaults();
    // climberMotor.setInverted(true);
    climberMotor.setIdleMode(IdleMode.kBrake);
    climberMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
    climberMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    climberMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.ClimberConstants.CLIMBER_HEIGHT_LIMIT);
    climberMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    climberMotor.setSmartCurrentLimit(Constants.ClimberConstants.CLIMBER_MOTOR_CURRENT_LIMIT);
    climberMotor.getEncoder().setPositionConversionFactor(Constants.ClimberConstants.CLIMBER_MOTOR_CONVERSION);
    // climberMotor.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Climber/ClimberSpeed", climberMotor.get());
    Logger.recordOutput("Climber/ClimberPosition", climberMotor.getEncoder().getPosition());
    Logger.recordOutput("Climber/ClimberCurrent", climberMotor.getOutputCurrent());
  }

  public Command setClimberSpeed(DoubleSupplier doubleSupplier){
    return run(() -> {
      climberMotor.set(doubleSupplier.getAsDouble());
    });
  }
}
