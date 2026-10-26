package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotHardware;

/** Turns the outtake box so pixels are placed upright or lying flat (for mosaics). */
public class ClawRotation extends StateController<ClawRotation.State> {
    public enum State { VERTICAL, HORIZONTAL }

    public double verticalPosition = 0.115;
    public double horizontalPosition = 0.395;
    private final Servo servo;

    public ClawRotation(RobotHardware robot) {
        super(State.VERTICAL);
        servo = robot.clawRotationServo;
    }

    @Override
    protected void apply(State s) {
        servo.setPosition(s == State.HORIZONTAL ? horizontalPosition : verticalPosition);
    }
}
