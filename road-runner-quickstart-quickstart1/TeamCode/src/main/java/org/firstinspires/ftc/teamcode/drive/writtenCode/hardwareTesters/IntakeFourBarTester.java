package org.firstinspires.ftc.teamcode.drive.writtenCode.hardwareTesters;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.Intake;

/**
 * Steps the intake four-bar with the driver's dpad (0.01 per press) to find positions.
 * A runs the intake at stack power, B stops it.
 */
@TeleOp(name = "Intake Four-Bar Tester", group = "Linear OpMode")
public class IntakeFourBarTester extends LinearOpMode {
    private static final double STEP = 0.01f;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        RobotHardware robot = new RobotHardware(hardwareMap);
        Intake intake = new Intake(robot);
        intake.update();

        Gamepad now = new Gamepad();
        Gamepad before = new Gamepad();
        Servo fourBar = robot.intakeFourBarServo;
        fourBar.setPosition(1f);

        waitForStart();
        while (opModeIsActive()) {
            if (isStopRequested()) return;
            before.copy(now);
            now.copy(gamepad1);

            if (now.dpad_up && !before.dpad_up) fourBar.setPosition(fourBar.getPosition() + STEP);
            if (now.dpad_down && !before.dpad_down) fourBar.setPosition(fourBar.getPosition() - STEP);
            if (now.a) intake.state = Intake.State.STACK;
            if (now.b) intake.state = Intake.State.STOP;

            telemetry.addData("position", fourBar.getPosition());
            telemetry.update();
            intake.update();
        }
    }
}
