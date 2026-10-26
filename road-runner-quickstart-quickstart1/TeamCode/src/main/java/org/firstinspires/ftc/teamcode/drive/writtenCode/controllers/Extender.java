package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotHardware;

/**
 * Horizontal slides that push the intake out. Both motors get the same encoder target and
 * the hub's built-in position control drives them there.
 */
public class Extender extends StateController<Extender.State> {
    public enum State {
        RETRACTED,
        /** Slightly extended. The lift switches to this whenever it rises. */
        HOLD,
        EXTENDED
    }

    public int retractedPosition = -5;
    public int holdPosition = 50;
    public int extendedPosition = 965;
    /** Encoder target sent every loop. Changing it moves the slides on the next update. */
    public int targetPosition = -5;

    private final DcMotor left;
    private final DcMotor right;

    public Extender(RobotHardware robot) {
        super(State.RETRACTED);
        left = robot.leftExtension;
        right = robot.rightExtension;
    }

    @Override
    protected void everyLoop() {
        driveTo(left, targetPosition);
        driveTo(right, targetPosition);
    }

    private static void driveTo(DcMotor motor, int target) {
        motor.setTargetPosition(target);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(1);
    }

    @Override
    protected void apply(State s) {
        switch (s) {
            case RETRACTED:
                targetPosition = retractedPosition;
                break;
            case HOLD:
                targetPosition = holdPosition;
                break;
            case EXTENDED:
                targetPosition = extendedPosition;
                break;
        }
    }
}
