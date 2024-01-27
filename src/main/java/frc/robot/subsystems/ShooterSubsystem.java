// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private final CANSparkMax shooterMotor = new CANSparkMax(Constants.ShooterConstants.SHOOTER_MOTOR_ID, MotorType.kBrushless) ;
  private final CANSparkMax indexMotor = new CANSparkMax(Constants.ShooterConstants.INDEX_MOTOR_ID, MotorType.kBrushless) ; 
  private int shooterSetpoint = 0;

  public ShooterSubsystem() {
    shooterMotor.restoreFactoryDefaults();
    shooterMotor.getPIDController().setP(0);
    shooterMotor.getPIDController().setI(0);
    shooterMotor.getPIDController().setD(0);
    shooterMotor.getPIDController().setFF(0);
    shooterMotor.getPIDController().setSmartMotionAllowedClosedLoopError(100, 0);
    shooterMotor.getEncoder().setVelocityConversionFactor(Constants.ShooterConstants.SHOOTER_GEARBOX_RATIO);

    indexMotor.setOpenLoopRampRate(Constants.ShooterConstants.INDEX_RAMP_RATE);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  } 

  public Command StartIndexMotor(){
    return runOnce(()-> {
      indexMotor.set(1);
    });
  }

  public Command StopIndexMotor(){
    return runOnce(()-> {
      indexMotor.set(0);
    });
  }

  public Command StartShooter(int rpm)
  {
    return runOnce(()->{
      shooterSetpoint = rpm;
      shooterMotor.getPIDController().setReference(rpm, CANSparkMax.ControlType.kVelocity);
    });
  }

  public Command StopShooter()
  {
    return runOnce(()->{
      shooterSetpoint = 0;
      shooterMotor.getPIDController().setReference(0, CANSparkMax.ControlType.kVoltage);
    });
  }

  public boolean IsAtSetpoint()
  {
    return Math.abs(shooterMotor.getEncoder().getVelocity() - shooterSetpoint) < Constants.ShooterConstants.SHOOTER_SETPOINT_TOLERANCE;
  }
}
