package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Releases pixels from the outtake box. The box holds two pixels: the outtake lock holds
 * the bottom one and the second-pixel latch holds the top one.
 */
public class Scoring extends StateController<Scoring.State> {
    public enum State { IDLE, DROP_BOTH, DROP_ONE, WAIT_FOR_LATCH, DROP_BOTTOM }

    /** Time for the latch to close on the top pixel before the bottom one is released. */
    private final double latchCloseTime = 0.3;
    private final ElapsedTime latchTimer = new ElapsedTime();

    private final ServoLatch secondPixelLatch;
    private final ServoLatch outtakeLock;
    private final ServoLatch flap;
    private final ClawRotation clawRotation;

    public Scoring(ServoLatch secondPixelLatch, ServoLatch outtakeLock, ServoLatch flap, ClawRotation clawRotation) {
        super(State.IDLE);
        this.secondPixelLatch = secondPixelLatch;
        this.outtakeLock = outtakeLock;
        this.flap = flap;
        this.clawRotation = clawRotation;
    }

    @Override
    protected boolean isTimed(State s) {
        return s == State.WAIT_FOR_LATCH;
    }

    @Override
    protected void apply(State s) {
        switch (s) {
            case DROP_ONE:
                secondPixelLatch.state = ServoLatch.State.CLOSED;
                latchTimer.reset();
                state = State.WAIT_FOR_LATCH;
                break;
            case WAIT_FOR_LATCH:
                if (latchTimer.seconds() > latchCloseTime) {
                    state = State.DROP_BOTTOM;
                }
                break;
            case DROP_BOTTOM:
                outtakeLock.state = ServoLatch.State.OPEN;
                secondPixelLatch.state = ServoLatch.State.CLOSED;
                state = State.IDLE;
                break;
            case DROP_BOTH:
                outtakeLock.state = ServoLatch.State.OPEN;
                secondPixelLatch.state = ServoLatch.State.OPEN;
                // Lying flat, the pixels also need the flap out of the way.
                if (clawRotation.state == ClawRotation.State.HORIZONTAL) {
                    flap.state = ServoLatch.State.OPEN;
                }
                state = State.IDLE;
                break;
            default:
                break;
        }
    }
}
