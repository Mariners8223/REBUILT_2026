// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  AddressableLED led;
  AddressableLEDBuffer buffer;

  LEDPattern currentPattern;
  boolean ledOn = true;

  /** Creates a new LED. */
  public LED() {
    led = new AddressableLED(LEDConstants.LED_PORT);
    buffer = new AddressableLEDBuffer(LEDConstants.LENGTH);

    currentPattern = LEDPattern.kOff;

    led.setLength(buffer.getLength());
    led.setData(buffer);
    led.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (ledOn) LEDPattern.kOff.applyTo(buffer);
    else currentPattern.applyTo(buffer);

    led.setData(buffer);
  }

  public void setIsLedOn(boolean isOn){
    ledOn = isOn;
  }

  public void setColour(Color colour){
    currentPattern = LEDPattern.solid(colour);
  }
  public void setColourCommand(Color colour){
    this.runOnce(() -> setColour(colour));
  }
}
