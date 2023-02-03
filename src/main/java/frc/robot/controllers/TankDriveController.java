package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants;

public class TankDriveController {

    private final CommandJoystick leftJoystick;
    private final CommandJoystick rightJoystick;

    public TankDriveController(CommandJoystick leftJoystick, CommandJoystick rightJoystick) {
        this.leftJoystick = leftJoystick;
        this.rightJoystick = rightJoystick;
    }

    public double getLeftPosition() {
        return leftJoystick.getY() * -1.0;
    }

    public double getRightPosition() {
        return rightJoystick.getY() * -1.0;
    }
    
}