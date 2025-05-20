// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConfig;

public class Underglow extends SubsystemBase {
  private AddressableLED led;
  private AddressableLEDBuffer led_buffer;
  private int halfLength, splitpoint, counter;
  private Color colorA, colorB;

  public Underglow() {
    led = new AddressableLED(LEDConfig.LED_PORT);
    halfLength = (int) Math.round(LEDConfig.LED_LEN / 2.0);
    led_buffer = new AddressableLEDBuffer(halfLength * 2);
    led.setLength(led_buffer.getLength());
    led.start();

    splitpoint = counter = 0;
    colorA = new Color();
    colorB = new Color();
  }

  public void periodic() {
    if (counter >= 4) {
      displayHalfColor(splitpoint, colorA);
      displayHalfColor(splitpoint + halfLength, colorB);
      splitpoint++;
      counter = 0;
    }
    counter++;
  }

  private void displayHalfColor(int startIndex, Color color) {
    for (int i = 0; i < halfLength; i++)
      led_buffer.setLED((i + startIndex) % (halfLength * 2), color);
    led.setData(led_buffer);
  }

  public void setSolidColor(Color color) {
    colorA = colorB = color;
  }

  public void setTwoColors(Color color1, Color color2) {
    colorA = color1;
    colorB = color2;
  }

  public void setSolidColor(int r, int g, int b) {
    colorA = new Color(r, g, b);
    colorB = new Color(r, g, b);
  }

  public void setTwoColors(int r1, int g1, int b1, int r2, int g2, int b2) {
    colorA = new Color(r1, g1, b1);
    colorB = new Color(r2, g2, b2);
  }
}
