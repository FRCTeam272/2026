// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** this is a custom printed controller and button names have been made to reflect that */
public class LC_2025Custom {
    public Joystick controller;
    public JoystickButton UnloadedConfig;
    public JoystickButton CoralIntake;
    public JoystickButton CoralL1;
    public JoystickButton CoralL2;
    public Trigger CoralL3;
    public Trigger CoralL4;
    public JoystickButton AlgaeIntakeHigh;
    public JoystickButton AlgaeIntakeLow;
    public JoystickButton AlgaeIntakeFloor;
    public JoystickButton NoItemDriving;
    public JoystickButton AlgaeScoreProcessor;
    public JoystickButton AlgaeScoreHigh;
    public JoystickButton EnableClimb;
    public Trigger RaiseTrimPot;
    public Trigger LowerTrimPot;
    public JoystickButton CoralDriving;
    public JoystickButton AlgaeDriving;
    public LC_2025Custom(int port) {
        controller = new Joystick(port);
        this.CoralIntake = new JoystickButton(controller, 1);
        this.AlgaeIntakeHigh = new JoystickButton(controller, 3);
        this.AlgaeIntakeLow = new JoystickButton(controller, 4);
        this.AlgaeIntakeFloor = new JoystickButton(controller, 5);
        this.NoItemDriving = new JoystickButton(controller, 7);
        this.AlgaeScoreProcessor = new JoystickButton(controller, 8);
        this.CoralL2 = new JoystickButton(controller, 12);
        this.CoralL1 = new JoystickButton(controller, 11);
        this.AlgaeScoreHigh = new JoystickButton(controller, 10);
        this.EnableClimb = new JoystickButton(controller, 9);
        this.RaiseTrimPot = new Trigger(
            () -> controller.getRawAxis(1) > .5
        );
        this.LowerTrimPot = new Trigger(
            () -> controller.getRawAxis(1) < -.5
        );
        this.AlgaeDriving = new JoystickButton(controller, 6);
        // new Trigger( // Button 6
        //     () -> controller.getRawAxis(2) > .5
        // );
        this.CoralDriving = new JoystickButton(controller, 2); 
        // new Trigger( // Button 2
        //     () -> controller.getRawAxis(2) < -.5
        // );

        this.CoralL4 = new Trigger(
            () -> controller.getRawAxis(0) > -.5
        ); // is axis 0 neg
        this.CoralL3 = new Trigger(
            () -> controller.getRawAxis(0) > .5
        ); // is axis 0 pos 
        
    }

}
