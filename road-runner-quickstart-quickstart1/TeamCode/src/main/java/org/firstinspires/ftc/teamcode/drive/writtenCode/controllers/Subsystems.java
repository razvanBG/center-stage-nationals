package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotHardware;

/** Builds every subsystem once and updates them all in a fixed order each loop. */
public class Subsystems {
    public final RobotHardware robot;

    public final Hang hang;
    public final DroneLauncher droneLauncher;
    public final Intake intake;
    public final IntakeFourBar intakeFourBar;
    public final TransferGates transferGates;
    public final ServoLatch secondPixelLatch;
    public final ServoLatch flap;
    public final ServoLatch outtakeLock;
    public final OuttakeFourBar outtakeFourBar;
    public final ClawRotation clawRotation;
    public final Extender extender;
    public final Lift lift;
    public final Transfer transfer;
    public final Scoring scoring;

    public Subsystems(RobotHardware robot) {
        this.robot = robot;
        hang = new Hang(robot);
        droneLauncher = new DroneLauncher(robot);
        intake = new Intake(robot);
        intakeFourBar = new IntakeFourBar(robot);
        transferGates = new TransferGates(robot);
        secondPixelLatch = new ServoLatch(robot.secondPixelServo, 0.6, 0);
        flap = new ServoLatch(robot.flapServo, 0.5, 0.1);
        outtakeLock = new ServoLatch(robot.outtakeLockServo, 0.5, 0);
        outtakeFourBar = new OuttakeFourBar(robot);
        clawRotation = new ClawRotation(robot);
        extender = new Extender(robot);
        lift = new Lift(outtakeFourBar, extender, robot);
        transfer = new Transfer(intake, transferGates, outtakeLock, robot);
        scoring = new Scoring(secondPixelLatch, outtakeLock, flap, clawRotation);
    }

    /** Puts the intake box in its start position and sends every subsystem to its initial state. */
    public void initialize() {
        robot.intakeBoxServo.setPosition(Transfer.startPosition);
        update();
    }

    public void update() {
        hang.update();
        droneLauncher.update();
        extender.update();
        clawRotation.update();
        outtakeFourBar.update();
        lift.update();
        intake.update();
        intakeFourBar.update();
        transferGates.update();
        transfer.update();
        secondPixelLatch.update();
        flap.update();
        outtakeLock.update();
        scoring.update();
    }

    /**
     * Prepares the outtake and starts a transfer. The second-pixel latch opens so it does not
     * block the incoming pixels; the flap is closed unless the caller already knows it is.
     */
    public void startTransfer(boolean extenderReturning, boolean closeFlap) {
        if (closeFlap) {
            flap.state = ServoLatch.State.CLOSED;
        }
        secondPixelLatch.state = ServoLatch.State.OPEN;
        transfer.start(extenderReturning);
    }

    /** Both break beams are blocked, so both pixels are in the intake box. */
    public boolean hasTwoPixels() {
        return !robot.frontBeam.getState() && !robot.backBeam.getState();
    }
}
