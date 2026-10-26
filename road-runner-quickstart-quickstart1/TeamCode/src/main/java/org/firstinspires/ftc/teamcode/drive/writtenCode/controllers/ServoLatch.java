package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.qualcomm.robotcore.hardware.Servo;

/** A servo with two positions. Used for the outtake lock, the flap and the second-pixel latch. */
public class ServoLatch extends StateController<ServoLatch.State> {
    public enum State { CLOSED, OPEN }

    public double closedPosition;
    public double openPosition;
    private final Servo servo;

    public ServoLatch(Servo servo, double closedPosition, double openPosition) {
        super(State.CLOSED);
        this.servo = servo;
        this.closedPosition = closedPosition;
        this.openPosition = openPosition;
    }

    @Override
    protected void apply(State s) {
        servo.setPosition(s == State.OPEN ? openPosition : closedPosition);
    }
}
