// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.controllers;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller.Button;
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
    public JoystickButton ClimberRelease;
    public Trigger ForceCloseIntake;
    public JoystickButton Stir;
    public Trigger CloseUpFlywheel;
    public Trigger MeduimFlywheel;
    public JoystickButton FarFlywheel;
    public Trigger UseAutoFlywheel;
    public JoystickButton ShooterTrimPotUp;
    public Trigger ShooterTrimPotDown;
    public JoystickButton HoodTrimPotUp;
    public JoystickButton HoodTrimPotDown;
    public JoystickButton BonusButton2;
    public JoystickButton BonusButton3;


    public LC_2026Custom(int port) {
        controller = new Joystick(port);
        this.ClimberStage1 = new JoystickButton(controller, 6);
        this.ClimberStage2 = new JoystickButton(controller, 10);
        this.ClimberStage3 =  new JoystickButton(controller, 12);
        this.ClimberRelease = new JoystickButton(controller, 5);
        this.Stir = new JoystickButton(controller, 7);
        this.CloseUpFlywheel = new Trigger(() -> controller.getRawAxis(0) < -.5);
        this.MeduimFlywheel = new Trigger(() -> controller.getRawAxis(0) > .5);
        this.FarFlywheel = new JoystickButton(controller, 11);
        this.UseAutoFlywheel = new Trigger(() -> controller.getRawAxis(1) > .5);
        this.ShooterTrimPotUp = new JoystickButton(controller, 9);
        this.ShooterTrimPotDown = new Trigger(() -> controller.getRawAxis(1) < -.5);;
        this.HoodTrimPotUp = new JoystickButton(controller, 8);
        this.HoodTrimPotDown = new JoystickButton(controller, 4);
        this.ForceCloseIntake = new JoystickButton(controller, 0);
        this.BonusButton2 = new JoystickButton(controller, 1);
        this.BonusButton3 = new JoystickButton(controller, 2);
    }

    // new Trigger(() -> controller.getRawAxis(0) < -.5);

    @Override
    public void periodic(){
        if(DriverStation.isFMSAttached()) return;
        for(var i = 0; i < 12; i++){
            SmartDashboard.putBoolean("Operator/Is " + i + "Pressed", controller.getRawButton(i));
        }
        
    }
}
