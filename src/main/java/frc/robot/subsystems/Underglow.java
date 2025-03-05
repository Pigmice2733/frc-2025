// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConfig;

public class Underglow extends SubsystemBase {
  private AddressableLED led;
  private AddressableLEDBuffer led_buffer;
  private int length;

  public Underglow() {
    led = new AddressableLED(LEDConfig.LED_PORT);
    length = LEDConfig.LED_LEN;
    led_buffer = new AddressableLEDBuffer(length);
    led.setLength(led_buffer.getLength());
    led.start();
  }

  public void periodic() {
  }

  public void clear() {
    displaySolidColor(0, 0, 0);
  }

  public void displaySolidColor(int r, int g, int b) {
    for (int i = 0; i < length; i++)
      led_buffer.setRGB(i, r, g, b);
    led.setData(led_buffer);
  }

  public void displayPigmicePurple() {
    displaySolidColor(75, 48, 71);
  }
}
