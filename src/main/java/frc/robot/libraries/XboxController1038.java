package frc.robot.libraries;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class XboxController1038 extends CommandXboxController {
    private XboxController controller;

    // Enums
    public enum PovPositions {
        Up, Down, Left, Right, None
    }

    /**
     * Creates a new Xbox joystick object
     *
     * @param port USB port the joystick should be in
     */
    public XboxController1038(int port) {
        super(port);
        this.controller = super.getHID();
    }

    public PovPositions getPOV() {
        int povVal = controller.getPOV();
        switch (povVal) {
            case 0:
                return PovPositions.Up;
            case 90:
                return PovPositions.Right;
            case 180:
                return PovPositions.Down;
            case 270:
                return PovPositions.Left;
            default:
                return PovPositions.None;
        }
    }

    /**
     * Returns the joystick axis value or 0 if less than deadband
     *
     * @return value of input axis, after deadband
     */
    public double deadband(double value) {
        return MathUtil.applyDeadband(value, 0.1);
    }

    /**
     * Returns the state of the left joystick on its vertical axis
     *
     * @return value of the left joystick vertical axis, inverted so positive values
     *         are joystick up
     */
    public double getLeftY() {
        return deadband(-super.getLeftY());
    }

    /**
     * Returns the state of the left joystick on its horizontal axis
     *
     * @return value of the left joystick horizontal axis
     */
    public double getLeftX() {
        return deadband(super.getLeftX());
    }

    /**
     * Returns the state of the right joystick on its vertical axis
     *
     * @return value of the right joystick vertical axis, inverted so positive
     *         values are joystick up
     */
    public double getRightY() {
        return deadband(-super.getRightY());
    }

    /**
     * Returns the state of the right joystick on its horizontal axis
     *
     * @return value of the right joystick horizontal axis
     */
    public double getRightX() {
        return deadband(super.getRightX());
    }


    /**
     * Sets the left rumble speed
     *
     * @param speed the rumble speed between 0.0 and 1.0
     * @return the new speed
     */
    public void setLeftRumble(double speed) {
        controller.setRumble(RumbleType.kLeftRumble, speed);
    }

    /**
     * Sets the right rumble speed
     *
     * @param speed the rumble speed between 0.0 and 1.0
     * @return the new speed
     */
    public void setRightRumble(double speed) {
        controller.setRumble(RumbleType.kRightRumble, speed);
    }
}