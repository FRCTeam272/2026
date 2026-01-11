package frc.lib.controllers;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class LC_FlyStickController {
    public Joystick controller;
    public JoystickButton trigger;
    public JoystickButton thumb;
    public JoystickButton _3;
    public JoystickButton _4;
    public JoystickButton _5;
    public JoystickButton _6;
    public JoystickButton _7;
    public JoystickButton _8;
    public JoystickButton _9;
    public JoystickButton _10;
    public JoystickButton _11;
    public JoystickButton _12;
    public DoubleSupplier Y;
    public DoubleSupplier X;
    public DoubleSupplier Z;
    public DoubleSupplier slider;

    public POVButton pov;

    public LC_FlyStickController(int port) {
        controller = new Joystick(port);
        trigger = new JoystickButton(controller, 1);
        thumb = new JoystickButton(controller, 2);
        _3 = new JoystickButton(controller, 3);
        _4 = new JoystickButton(controller, 4);
        _5 = new JoystickButton(controller, 5);
        _6 = new JoystickButton(controller, 6);
        _7 = new JoystickButton(controller, 7);
        _8 = new JoystickButton(controller, 8);
        _9 = new JoystickButton(controller, 9);
        _10 = new JoystickButton(controller, 10);
        _11 = new JoystickButton(controller, 11);
        _12 = new JoystickButton(controller, 12);

        Y = () -> controller.getRawAxis(1);
        X = () -> controller.getRawAxis(0);
        Z = () -> controller.getRawAxis(2);
        slider = () -> controller.getRawAxis(3);

        pov = new POVButton(controller, 0);
    }
}
