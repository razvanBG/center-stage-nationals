package org.firstinspires.ftc.teamcode.drive.writtenCode.hardwareTesters;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotHardware;

@TeleOp(name = "Extender Tester", group = "Linear OpMode")
public class ExtenderTester extends EncoderReadout {
    @Override
    protected DcMotor[] motors(RobotHardware robot) {
        return new DcMotor[]{robot.leftExtension, robot.rightExtension};
    }

    @Override
    protected String[] labels() {
        return new String[]{"Left extension position", "Right extension position"};
    }
}
