package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotHardware;

/** Two continuous-rotation servos that pull the robot up for the end-game hang. They face opposite ways. */
public class Hang extends StateController<Hang.State> {
    public enum State { STOP, UP, DOWN }

    private static final double POWER = 0.9;
    private final CRServo left;
    private final CRServo right;

    public Hang(RobotHardware robot) {
        super(State.STOP);
        left = robot.leftHangServo;
        right = robot.rightHangServo;
    }

    @Override
    protected void apply(State s) {
        switch (s) {
            case STOP:
                right.setPower(0);
                left.setPower(0);
                break;
            case UP:
                right.setPower(POWER);
                left.setPower(-POWER);
                break;
            case DOWN:
                right.setPower(-POWER);
                left.setPower(POWER);
                break;
        }
    }
}
