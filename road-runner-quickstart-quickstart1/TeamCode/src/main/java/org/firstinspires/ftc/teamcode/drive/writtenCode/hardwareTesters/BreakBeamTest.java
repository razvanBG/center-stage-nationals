package org.firstinspires.ftc.teamcode.drive.writtenCode.hardwareTesters;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

/** Shows the raw state of both break-beam sensors. */
@TeleOp(name = "Break Beam Test", group = "Linear Opmode")
public class BreakBeamTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        DigitalChannel front = hardwareMap.digitalChannel.get("frontBeam");
        DigitalChannel back = hardwareMap.digitalChannel.get("backBeam");
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("time", "elapsed time: " + time);
            telemetry.addData("front beam", front.getState());
            telemetry.addData("back beam", back.getState());
            telemetry.update();
        }
    }
}
