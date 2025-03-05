// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
import java.util.TreeMap;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Underglow extends SubsystemBase {
  private AddressableLED led;
  private AddressableLEDBuffer led_buffer;
  private final int LED_PORT = 0;
  public static final int LED_LEN = 91;
  // private static double brightnessFactor = 0.5;

  public Underglow() {
    led = new AddressableLED(LED_PORT);
    led_buffer = new AddressableLEDBuffer(LED_LEN);
    led.setLength(led_buffer.getLength());
    led.start();
  }

  public void periodic() {

  }

  public void clear() {
    displaySolidColor(0, 0, 0);
  }

  public void displaySolidColor(int r, int g, int b) {
    for (int i = 0; i < LED_LEN; i++)
      led_buffer.setRGB(i, r, g, b);
    led.setData(led_buffer);
  }

  public void displayPigmicePurple() {
    displaySolidColor(75, 48, 71);
  }
}
