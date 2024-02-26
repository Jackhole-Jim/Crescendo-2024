// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;

  public LEDSubsystem() {
    m_ledBuffer = new AddressableLEDBuffer(Constants.LEDConstants.TOTAL_LEDS);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  @Override
  public void periodic() {
    m_led.setData(m_ledBuffer);
  }

  //0-1
  public void setPercentageLit(double percentage, Color color)
  {
    for(int i = 0; i < percentage * Constants.LEDConstants.TOTAL_LEDS; i++)
    {
      m_ledBuffer.setLED(i, Color.kBlack);
    }

    for(int i = 0; i < percentage * Constants.LEDConstants.MID_POINT_START_LED_INDEX; i++)
    {
      m_ledBuffer.setLED(i, color);
    }

    for(int i = Constants.LEDConstants.TOTAL_LEDS - 1; i > Constants.LEDConstants.TOTAL_LEDS - ((Constants.LEDConstants.TOTAL_LEDS - Constants.LEDConstants.MID_POINT_START_LED_INDEX) * percentage) - 1; i--)
    {
      m_ledBuffer.setLED(i, color);
    }
  }

  public Command setPercentageLitCommand(double percentage, Color color)
  {
    return runOnce(() -> setPercentageLit(percentage, color));
  }

  public Command setPercentageLitCommand(DoubleSupplier percentageSupplier, Color color)
  {
    return run(() -> setPercentageLit(percentageSupplier.getAsDouble(), color));
  }
}
