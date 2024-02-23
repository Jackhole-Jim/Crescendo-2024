// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AmpWhipperSubsystem extends SubsystemBase {

  private final PWM linearActuatorLeft = new PWM(Constants.WhipperConstants.LINEAR_ACTUATOR_LEFT);
  private final PWM linearActuatorRight = new PWM(Constants.WhipperConstants.LINEAR_ACTUATOR_RIGHT);
  private final CANSparkMax whipperMotor = new CANSparkMax(Constants.WhipperConstants.WHIPPER_MOTOR_ID, MotorType.kBrushless);

  /** Creates a new AmpWhipperSubsystem. */
  public AmpWhipperSubsystem() {
    linearActuatorLeft.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
    linearActuatorRight.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean IsWhipperExtended(){
    return linearActuatorLeft.getSpeed() > 0.0;
  }

  public Command extendActuators(){
    return runOnce(() -> {
      linearActuatorLeft.setSpeed(1);
      linearActuatorRight.setSpeed(1);
    });
  }

  public Command retractActuators(){
    return runOnce(() -> {
      linearActuatorLeft.setSpeed(-1);
      linearActuatorRight.setSpeed(-1);
    });
  }
  public Command setWhipperSpeed(DoubleSupplier Double_Supplier){
    return run(() -> {
      whipperMotor.set(Double_Supplier.getAsDouble());
    });
  }
}
