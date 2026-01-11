package frc.lib.controllers;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class LC_XboxController {
    public Joystick controller;
    public JoystickButton A;
    public JoystickButton B;
    public JoystickButton X;
    public JoystickButton Y;
    public JoystickButton LB;
    public JoystickButton RB;
    public JoystickButton BACK;
    public JoystickButton START;
    public DoubleSupplier RightTrigger;
    public DoubleSupplier LeftTrigger;
    public DoubleSupplier RightStickY;
    public DoubleSupplier RightStickX;
    public DoubleSupplier LeftStickX;
    public DoubleSupplier LeftStickY;
    private double trigger_sensitivity;
    public POVButton POV;
    public LC_XboxController(int port) {
        this(port, .2);
    }

    public LC_XboxController(int port, double trigger_sensitivity){
        controller = new Joystick(port);
        A = new JoystickButton(controller, XboxController.Button.kA.value);
        B = new JoystickButton(controller, XboxController.Button.kB.value);
        X = new JoystickButton(controller, XboxController.Button.kX.value);
        Y = new JoystickButton(controller, XboxController.Button.kY.value);
        LB = new JoystickButton(controller, XboxController.Button.kLeftBumper.value);
        RB = new JoystickButton(controller, XboxController.Button.kRightBumper.value);
        BACK = new JoystickButton(controller, XboxController.Button.kBack.value);
        START = new JoystickButton(controller, XboxController.Button.kStart.value);
        RightTrigger = () -> controller.getRawAxis(XboxController.Axis.kRightTrigger.value);
        LeftTrigger =  () -> controller.getRawAxis(XboxController.Axis.kLeftTrigger.value);
        RightStickX = () -> controller.getRawAxis(XboxController.Axis.kRightX.value);
        RightStickY = () -> controller.getRawAxis(XboxController.Axis.kRightY.value);
        LeftStickX = () -> controller.getRawAxis(XboxController.Axis.kLeftX.value);
        LeftStickY = () -> controller.getRawAxis(XboxController.Axis.kLeftY.value);
        this.trigger_sensitivity = trigger_sensitivity;
        POV = new POVButton(controller, 0);
    }

    public Trigger RightTrigger() {
        return new Trigger(() -> Math.abs(RightTrigger.getAsDouble()) > trigger_sensitivity);
    }

    public Trigger LeftTrigger() {
        return new Trigger(() -> Math.abs(LeftTrigger.getAsDouble()) > trigger_sensitivity);
    }
}
