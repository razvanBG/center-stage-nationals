package org.firstinspires.ftc.teamcode.drive.writtenCode.hardwareTesters;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.Intake;

/** Runs the intake while the driver holds dpad left. */
@TeleOp(name = "Intake Tester", group = "Linear OpMode")
public class IntakeTester extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        Intake intake = new Intake(new RobotHardware(hardwareMap));
        waitForStart();
        while (opModeIsActive()) {
            if (isStopRequested()) return;
            intake.state = gamepad1.dpad_left ? Intake.State.COLLECT : Intake.State.STOP;
            intake.update();
        }
    }
}
