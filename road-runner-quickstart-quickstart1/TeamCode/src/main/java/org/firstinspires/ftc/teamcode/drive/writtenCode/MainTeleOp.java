package org.firstinspires.ftc.teamcode.drive.writtenCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.ClawRotation;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.DroneLauncher;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.Extender;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.Hang;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.Intake;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.IntakeFourBar;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.Lift;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.OuttakeFourBar;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.Scoring;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.ServoLatch;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.Subsystems;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.Transfer;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.TransferGates;

/**
 * Driver-controlled period.
 *
 * Gamepad 1 drives, works the extender, toggles the break-beam auto-transfer and launches the drone.
 * Gamepad 2 runs the intake, the transfer, the lift, scoring and the hang.
 * Inputs only request states; the subsystems do the work in {@link Subsystems#update()}.
 */
@TeleOp(name = "Main TeleOp", group = "Linear OpMode")
public class MainTeleOp extends LinearOpMode {
    private Subsystems s;

    // Snapshots of both gamepads from this loop and the previous one, used to detect a fresh press.
    private final Gamepad driver = new Gamepad();
    private final Gamepad operator = new Gamepad();
    private final Gamepad lastDriver = new Gamepad();
    private final Gamepad lastOperator = new Gamepad();

    private boolean beamTransferEnabled = true;
    private int liftPosition;
    private int extenderPosition;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        s = new Subsystems(new RobotHardware(hardwareMap));
        s.initialize();
        DriveMotors drive = new DriveMotors(hardwareMap);

        waitForStart();
        s.intakeFourBar.state = IntakeFourBar.State.DRIVE;
        s.intakeFourBar.update();

        while (opModeIsActive()) {
            if (isStopRequested()) return;

            liftPosition = s.robot.liftMotor.getCurrentPosition();
            extenderPosition = s.robot.leftExtension.getCurrentPosition();

            drive.driveRobotCentric(gamepad1);
            takeGamepadSnapshots();

            handleIntake();
            handleManualTransfer();
            handleLift();
            handleScoring();
            handleExtender();
            handleBeamTransfer();
            handleEndgame();
            handleStackIntake();

            s.update();
        }
    }

    private void takeGamepadSnapshots() {
        lastDriver.copy(driver);
        lastOperator.copy(operator);
        driver.copy(gamepad1);
        operator.copy(gamepad2);
    }

    private static boolean pressed(boolean now, boolean before) {
        return now && !before;
    }

    /** Operator dpad left toggles the intake, dpad right reverses it. */
    private void handleIntake() {
        if (pressed(operator.dpad_left, lastOperator.dpad_left)) {
            if (s.intake.state == Intake.State.STOP) {
                s.transferGates.state = TransferGates.State.OPEN;
                s.intake.state = Intake.State.COLLECT;
                if (s.intakeFourBar.state == IntakeFourBar.State.DRIVE) {
                    s.intakeFourBar.state = IntakeFourBar.State.COLLECT;
                }
            } else {
                s.transferGates.state = TransferGates.State.LOCKED;
                s.intake.state = Intake.State.STOP;
                s.intakeFourBar.state = IntakeFourBar.State.DRIVE;
            }
        }
        if (pressed(operator.dpad_right, lastOperator.dpad_right)) {
            s.intake.state = Intake.State.REVERSE;
        }
    }

    /** Operator B starts a transfer, but only when everything is back in its transfer position. */
    private void handleManualTransfer() {
        if (pressed(operator.b, lastOperator.b)
                && s.transfer.state == Transfer.State.IDLE
                && liftPosition >= 0
                && s.outtakeFourBar.state == OuttakeFourBar.State.RECEIVE
                && extenderPosition <= 0) {
            s.startTransfer(false, true);
        }
    }

    /** Operator A: low position. Operator X: mosaic position. Left stick: fine adjustment. */
    private void handleLift() {
        if (pressed(operator.a, lastOperator.a)) {
            if (s.lift.state == Lift.State.REST) {
                s.lift.state = Lift.State.LOW;
            } else {
                lowerLiftOrStraightenClaw();
            }
        }

        // The lift counts as down once it is close enough to the bottom.
        if (liftPosition >= -120 && s.lift.state == Lift.State.GOING_DOWN) {
            s.lift.state = Lift.State.REST;
        }

        if (gamepad2.left_stick_y > 0) {
            nudgeLift(10);
        } else if (gamepad2.left_stick_y < 0) {
            nudgeLift(-10);
        }

        if (pressed(operator.x, lastOperator.x)) {
            if (s.lift.state == Lift.State.REST) {
                s.lift.state = Lift.State.MOSAIC;
                s.clawRotation.state = ClawRotation.State.HORIZONTAL;
            } else {
                lowerLiftOrStraightenClaw();
            }
        }
    }

    /** A second press first turns a flat claw back upright, and only then brings the lift down. */
    private void lowerLiftOrStraightenClaw() {
        if (s.clawRotation.state == ClawRotation.State.VERTICAL) {
            s.lift.state = Lift.State.GOING_DOWN;
            s.flap.state = ServoLatch.State.CLOSED;
        } else if (s.clawRotation.state == ClawRotation.State.HORIZONTAL) {
            s.clawRotation.state = ClawRotation.State.VERTICAL;
        }
    }

    private void nudgeLift(int counts) {
        int target = Math.min(s.lift.targetPosition + counts, s.lift.manualLimit);
        s.lift.targetPosition = Math.max(Lift.highPosition, target);
        s.extender.state = Extender.State.HOLD;
    }

    /** Left bumper drops one pixel (claw upright only), right bumper drops both, Y rotates the claw. */
    private void handleScoring() {
        if (pressed(operator.left_bumper, lastOperator.left_bumper)
                && s.scoring.state == Scoring.State.IDLE
                && s.clawRotation.state == ClawRotation.State.VERTICAL) {
            s.scoring.state = Scoring.State.DROP_ONE;
        }
        if (pressed(operator.right_bumper, lastOperator.right_bumper)
                && s.scoring.state == Scoring.State.IDLE) {
            s.scoring.state = Scoring.State.DROP_BOTH;
        }
        if (pressed(operator.y, lastOperator.y)
                && s.outtakeFourBar.state == OuttakeFourBar.State.SCORE) {
            s.clawRotation.state = s.clawRotation.state == ClawRotation.State.HORIZONTAL
                    ? ClawRotation.State.VERTICAL
                    : ClawRotation.State.HORIZONTAL;
        }
    }

    /** Driver right bumper sends the intake out on the slides, and a second press brings it back and transfers. */
    private void handleExtender() {
        if (!pressed(driver.right_bumper, lastDriver.right_bumper)) return;

        if (s.extender.state == Extender.State.RETRACTED) {
            s.extender.state = Extender.State.EXTENDED;
            s.transferGates.state = TransferGates.State.OPEN;
            s.intake.state = Intake.State.COLLECT;
            s.intakeFourBar.state = IntakeFourBar.State.COLLECT;
        } else {
            s.extender.state = Extender.State.RETRACTED;
            s.startTransfer(true, true);
        }
    }

    /** With both pixels detected by the break beams, the transfer starts on its own. Driver X turns this off. */
    private void handleBeamTransfer() {
        if (pressed(driver.x, lastDriver.x)) {
            beamTransferEnabled = !beamTransferEnabled;
        }
        if (!beamTransferEnabled || !s.hasTwoPixels() || s.transfer.state != Transfer.State.IDLE) return;

        if (s.extender.state == Extender.State.RETRACTED) {
            s.startTransfer(false, true);
        } else {
            // The flap is already closed whenever the lift is down.
            s.startTransfer(true, false);
            s.extender.state = Extender.State.RETRACTED;
        }
    }

    /** Driver Y toggles the drone launcher. Operator dpad down/up run the hang while held. */
    private void handleEndgame() {
        if (pressed(driver.y, lastDriver.y)) {
            s.droneLauncher.state = s.droneLauncher.state == DroneLauncher.State.HOLD
                    ? DroneLauncher.State.LAUNCH
                    : DroneLauncher.State.HOLD;
        }

        if (operator.dpad_down) {
            if (s.hang.state == Hang.State.STOP) {
                s.hang.state = Hang.State.DOWN;
            }
        } else {
            s.hang.state = Hang.State.STOP;
        }
        if (operator.dpad_up && s.hang.state == Hang.State.STOP) {
            s.hang.state = Hang.State.UP;
        }
    }

    /** Holding the operator's right stick button collects from a stack; otherwise the four-bar sits at collect height. */
    private void handleStackIntake() {
        if (gamepad2.right_stick_button) {
            s.intake.state = Intake.State.STACK;
            s.transferGates.state = TransferGates.State.OPEN;
            s.intakeFourBar.state = IntakeFourBar.State.COLLECT_STACK;
        } else {
            s.intakeFourBar.state = IntakeFourBar.State.COLLECT;
        }
    }
}
