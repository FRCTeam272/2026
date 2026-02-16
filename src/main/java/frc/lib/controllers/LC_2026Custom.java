// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger; // if anything is an AXIS use this

/** this is a custom printed controller and button names have been made to reflect that */
public class LC_2026Custom {
    public Joystick controller;
    public JoystickButton AgitateButton;
    public JoystickButton ShooterTrimPotUpButton;
    public JoystickButton ShooterTrimPotDownButton;
    public JoystickButton IntakeTrimPotUpButton;
    public JoystickButton IntakeTrimPotDownButton;
    public JoystickButton hoodTrimPotUpButton;
    public JoystickButton hoodTrimPotDownButton;
    public JoystickButton ClimbTrimPotUpButton;
    public JoystickButton ClimbTrimPotDownButton;
    public JoystickButton ClimberRaise;
    public JoystickButton ClimberLower;
    public JoystickButton ClimberZero;
    public LC_2026Custom(int port) {
        controller = new Joystick(port);
        // this.CoralIntake = new JoystickButton(controller, 1);        
        this.AgitateButton = new JoystickButton(controller, 12);
        this.ShooterTrimPotUpButton = new JoystickButton(controller, 1);
        this.ShooterTrimPotDownButton = new JoystickButton(controller, 2);
        this.IntakeTrimPotUpButton = new JoystickButton(controller, 3);
        this.IntakeTrimPotDownButton = new JoystickButton(controller, 4);
        this.hoodTrimPotUpButton = new JoystickButton(controller, 5);
        this.hoodTrimPotDownButton = new JoystickButton(controller, 6);
        this.ClimbTrimPotUpButton = new JoystickButton(controller, 7);
        this.ClimbTrimPotDownButton = new JoystickButton(controller, 8);
        this.ClimberRaise = new JoystickButton(controller, 9);
        this.ClimberLower = new JoystickButton(controller, 10);
        this.ClimberZero = new JoystickButton(controller, 11);
    }
}
