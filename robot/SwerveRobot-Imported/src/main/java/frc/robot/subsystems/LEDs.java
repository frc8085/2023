// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
    private static LEDs instance;

    public enum GamePiece {
        CUBE, CONE, NONE
    }

    public GamePiece selectedGamePiece = GamePiece.NONE;

    public static LEDs getInstance() {
        if (instance == null) {
            instance = new LEDs();
        }
        return instance;
    }

    private final AddressableLED m_leds;
    private final AddressableLEDBuffer m_ledsBuffer;

    private LEDs() {
        m_leds = new AddressableLED(9);
        m_ledsBuffer = new AddressableLEDBuffer(60);
        m_leds.setLength(m_ledsBuffer.getLength());

        m_leds.setData(m_ledsBuffer);

    }

    public void setDesiredPiece(GamePiece piece) {
        selectedGamePiece = piece;
    }

    // I think this tells each LED to turn on
    private void solid(Color color) {
        for (int i = 0; i < 60; i++)
            m_ledsBuffer.setLED(i, color);
    }

    public void periodic() {
        // Select LED mode
        // default off

        // if we are Estopped, change to red
        if (DriverStation.isEStopped()) {
            solid(Color.kRed);
        }

        // if we are in the final 10 seconds of match, change lights to blue
        else if (Timer.getMatchTime() < 10) {
            solid(Color.kBlue);
        }

        // if we're in Auto, change to aquamarine
        else if (DriverStation.isAutonomous()) {
            solid(Color.kBlue);
        }

        else if (selectedGamePiece == GamePiece.NONE) {
            solid(Color.kBlack);
        }

        // if operator has selected manual intake, change lights to purple
        else if (selectedGamePiece == GamePiece.CUBE) {
            solid(Color.kPurple);
        }
        // if operator has selected SingleSubstationButton, change lights to yellow
        else if (selectedGamePiece == GamePiece.CONE) {
            solid(Color.kYellow);
        }

    }
}