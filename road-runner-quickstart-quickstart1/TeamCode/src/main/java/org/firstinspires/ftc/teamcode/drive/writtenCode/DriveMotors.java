package org.firstinspires.ftc.teamcode.drive.writtenCode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** The four mecanum drive motors, driven robot-centric in TeleOp. */
public class DriveMotors {
    private final DcMotor leftFront;
    private final DcMotor leftBack;
    private final DcMotor rightFront;
    private final DcMotor rightBack;

    public DriveMotors(HardwareMap map) {
        rightFront = map.get(DcMotor.class, "rightFront");
        leftFront = map.get(DcMotor.class, "leftFront");
        rightBack = map.get(DcMotor.class, "rightBack");
        leftBack = map.get(DcMotor.class, "leftBack");

        // Open-loop power is faster than the SDK's velocity control and keeps the loop short.
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // The left side is mounted mirrored.
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        // Brake so other robots cannot push us around easily.
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void setMode(DcMotor.RunMode mode) {
        leftFront.setMode(mode);
        rightFront.setMode(mode);
        leftBack.setMode(mode);
        rightBack.setMode(mode);
    }

    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFront.setZeroPowerBehavior(behavior);
        rightFront.setZeroPowerBehavior(behavior);
        leftBack.setZeroPowerBehavior(behavior);
        rightBack.setZeroPowerBehavior(behavior);
    }

    /**
     * Left stick Y drives forward and back, the triggers strafe, right stick X turns.
     * Powers are scaled down together if any of them would exceed 1.
     */
    public void driveRobotCentric(Gamepad gamepad) {
        double forward = -gamepad.left_stick_y;
        double strafe = (-gamepad.left_trigger + gamepad.right_trigger) * 1.05; // mecanum strafes slower than it drives
        double turn = gamepad.right_stick_x;

        double scale = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1);
        leftFront.setPower((forward + strafe + turn) / scale);
        leftBack.setPower((forward - strafe + turn) / scale);
        rightFront.setPower((forward - strafe - turn) / scale);
        rightBack.setPower((forward + strafe - turn) / scale);
    }
}
