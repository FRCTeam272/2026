// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.controllers;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger; // if anything is an AXIS use this

/** this is a custom printed controller and button names have been made to reflect that */
public class LC_2026Custom extends SubsystemBase{
    public Joystick controller;
    public JoystickButton ClimberStage1;
    public JoystickButton ClimberStage2;
    public Trigger ClimberStage3;
    public Trigger ClimberRelease;
    public Trigger ForceCloseIntake;
    public JoystickButton Stir;
    public JoystickButton CloseUpFlywheel;
    public JoystickButton MeduimFlywheel;
    public JoystickButton FarFlywheel;
    public JoystickButton UseAutoFlywheel;
    public JoystickButton ShooterTrimPotUp;
    public JoystickButton ShooterTrimPotDown;
    public JoystickButton HoodTrimPotUp;
    public JoystickButton HoodTrimPotDown;


    public LC_2026Custom(int port) {
        controller = new Joystick(port);
        this.ClimberStage1 = new JoystickButton(controller, 11);
        this.ClimberStage2 = new JoystickButton(controller, 12);
        this.ClimberStage3 = new Trigger(() -> controller.getRawAxis(0) > .5);
        this.ClimberRelease = new Trigger(() -> controller.getRawAxis(0) < -.5);
        this.Stir = new JoystickButton(controller, 8);
        this.CloseUpFlywheel = new JoystickButton(controller, 5);
        this.MeduimFlywheel = new JoystickButton(controller, 4);
        this.FarFlywheel = new JoystickButton(controller, 3);
        this.UseAutoFlywheel = new JoystickButton(controller, 7);
        this.ShooterTrimPotUp = new JoystickButton(controller, 1);
        this.ShooterTrimPotDown = new JoystickButton(controller, 2);
        this.HoodTrimPotUp = new JoystickButton(controller, 10);
        this.HoodTrimPotDown = new JoystickButton(controller, 9);
        this.ForceCloseIntake = new Trigger(() -> controller.getRawAxis(1) > .5);
    }

    @Override
    public void periodic(){
        if(DriverStation.isFMSAttached()) return;
        for(var i = 0; i < 12; i++){
            SmartDashboard.putBoolean("Operator/Is " + i + "Pressed", controller.getRawButton(i));
        }
        
    }
}
