package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotHardware;

/**
 * Four-bar that sets the height of the intake. The stack positions lower it one pixel at a time
 * so the roller only grabs the top pixels of a stack.
 */
public class IntakeFourBar extends StateController<IntakeFourBar.State> {
    public enum State {
        INIT,
        DRIVE,
        COLLECT,
        COLLECT_STACK,
        AUTO_STACK_5,
        AUTO_STACK_4,
        AUTO_STACK_LOW
    }

    public double initPosition = 1f;
    public double drivePosition = 0.7f;
    public double collectPosition = 0.56f;
    public double collectStackPosition = 0.73f;
    public double autoStack5Position = 0.74f;
    public double autoStack4Position = 0.71f;
    public double autoStackLowPosition = 0.62f;

    private final Servo servo;

    public IntakeFourBar(RobotHardware robot) {
        super(State.INIT);
        servo = robot.intakeFourBarServo;
    }

    @Override
    protected void apply(State s) {
        servo.setPosition(positionFor(s));
    }

    private double positionFor(State s) {
        switch (s) {
            case DRIVE:          return drivePosition;
            case COLLECT:        return collectPosition;
            case COLLECT_STACK:  return collectStackPosition;
            case AUTO_STACK_5:   return autoStack5Position;
            case AUTO_STACK_4:   return autoStack4Position;
            case AUTO_STACK_LOW: return autoStackLowPosition;
            default:             return initPosition;
        }
    }
}
