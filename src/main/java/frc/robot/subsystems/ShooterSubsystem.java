// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private final CANSparkMax shooterMotor = new CANSparkMax(Constants.ShooterConstants.SHOOTER_MOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax shooterMotor2 = new CANSparkMax(Constants.ShooterConstants.SHOOTER_MOTOR_ID2, MotorType.kBrushless);
  private final CANSparkMax indexMotor = new CANSparkMax(Constants.ShooterConstants.INDEX_MOTOR_ID, MotorType.kBrushless); 
  private int shooterSetpoint = 0;

  public ShooterSubsystem() {
    shooterMotor.restoreFactoryDefaults();
    shooterMotor.getPIDController().setP(Constants.ShooterConstants.SHOOTER_P);
    shooterMotor.getPIDController().setI(0);
    shooterMotor.getPIDController().setD(Constants.ShooterConstants.SHOOTER_D);
    shooterMotor.getPIDController().setFF(Constants.ShooterConstants.SHOOTER_FF);
    shooterMotor.getPIDController().setSmartMotionAllowedClosedLoopError(100, 0);
    shooterMotor.getEncoder().setVelocityConversionFactor(Constants.ShooterConstants.SHOOTER_GEARBOX_RATIO);
    shooterMotor.burnFlash();

    shooterMotor2.restoreFactoryDefaults();
    shooterMotor2.follow(shooterMotor);
    shooterMotor2.burnFlash();

    indexMotor.restoreFactoryDefaults();
    indexMotor.setInverted (true);
    indexMotor.setOpenLoopRampRate(Constants.ShooterConstants.INDEX_RAMP_RATE);
    indexMotor.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Shooter/ShooterRPM", shooterMotor.getEncoder().getVelocity());
    Logger.recordOutput("Shooter/ShooterCurrent", shooterMotor.getOutputCurrent());
    Logger.recordOutput("Shooter/ShooterFault", shooterMotor.getFaults());
    Logger.recordOutput("Shooter/ShooterStickyFault", shooterMotor.getStickyFaults());
    Logger.recordOutput("Shooter/Shooter2RPM", shooterMotor2.getEncoder().getVelocity());
    Logger.recordOutput("Shooter/Shooter2Current", shooterMotor2.getOutputCurrent());
    Logger.recordOutput("Shooter/Shooter2Fault", shooterMotor2.getFaults());
    Logger.recordOutput("Shooter/Shooter2StickyFault", shooterMotor2.getStickyFaults());
    Logger.recordOutput("Shooter/Setpoint", GetSetpoint());
    Logger.recordOutput("Shooter/IndexMotorSpeed", indexMotor.getEncoder().getVelocity());
    Logger.recordOutput("Shooter/IndexMotorSet", indexMotor.get());
    Logger.recordOutput("Shooter/IndexMotorCurrent", indexMotor.getOutputCurrent());
    Logger.recordOutput("Shooter/IndexMotorFault", indexMotor.getFaults());
    Logger.recordOutput("Shooter/IndexMotorStickyFault", indexMotor.getStickyFaults());
    Logger.recordOutput("Shooter/IsAtSetpoint", IsAtSetpoint());
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

  public void StartShooter(int rpm)
  {
      shooterSetpoint = rpm;
      shooterMotor.getPIDController().setReference(rpm, CANSparkMax.ControlType.kVelocity);
  }

  public Command StartShooterCommand(int rpm)
  {
    return runOnce(()->{
      StartShooter(rpm);
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

  public int GetSetpoint()
  {
    return shooterSetpoint;
  }

  public double GetRPM()
  {
    return shooterMotor.getEncoder().getVelocity();
  }
}
