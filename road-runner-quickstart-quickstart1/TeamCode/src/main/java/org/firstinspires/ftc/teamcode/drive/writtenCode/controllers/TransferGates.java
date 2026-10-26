package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotHardware;

/**
 * Two mirrored servos at the mouth of the intake box. They stay open while collecting,
 * close to hold the pixels while the box flips, and push the pixels into the outtake.
 */
public class TransferGates extends StateController<TransferGates.State> {
    public enum State { OPEN, PUSH, LOCKED }

    public double openLeft = 0.13;
    public double openRight = 1 - openLeft;
    public double pushLeft = 1;
    public double pushRight = 1 - pushLeft;
    public double lockedLeft = 0.05;
    public double lockedRight = 1 - lockedLeft;

    private final Servo left;
    private final Servo right;

    public TransferGates(RobotHardware robot) {
        super(State.OPEN);
        left = robot.leftGateServo;
        right = robot.rightGateServo;
    }

    @Override
    protected void apply(State s) {
        switch (s) {
            case OPEN:
                move(openLeft, openRight);
                break;
            case PUSH:
                move(pushLeft, pushRight);
                break;
            case LOCKED:
                move(lockedLeft, lockedRight);
                break;
        }
    }

    private void move(double leftPosition, double rightPosition) {
        left.setPosition(leftPosition);
        right.setPosition(rightPosition);
    }
}
