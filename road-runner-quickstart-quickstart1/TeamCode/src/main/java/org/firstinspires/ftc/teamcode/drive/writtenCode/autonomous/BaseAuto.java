package org.firstinspires.ftc.teamcode.drive.writtenCode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.writtenCode.TeamPropDetector;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.ServoLatch;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.Subsystems;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/** Setup shared by every autonomous: subsystems, camera, start pose and prop detection during init. */
public abstract class BaseAuto extends LinearOpMode {
    protected SampleMecanumDrive drive;
    protected Subsystems s;
    protected TeamPropDetector detector;
    /** Spike mark with the team prop: 1, 2 or 3. */
    protected int propCase = 1;

    protected void setUp(String alliance, Pose2d startPose) {
        drive = new SampleMecanumDrive(hardwareMap);
        s = new Subsystems(new RobotHardware(hardwareMap));
        // In autonomous the second-pixel latch starts open.
        s.secondPixelLatch.state = ServoLatch.State.OPEN;
        s.initialize();

        detector = new TeamPropDetector(hardwareMap, alliance);
        detector.startCamera();
        drive.setPoseEstimate(startPose);
    }

    /** Keeps reading the camera until the match starts. The last reading is the one used. */
    protected void detectPropUntilStart() {
        while (!isStarted() && !isStopRequested()) {
            propCase = detector.getCase();
            telemetry.addData("detected", propCase);
            telemetry.addLine("Init Complete");
            telemetry.update();
            sleep(50);
        }
        waitForStart();
    }

    /** Starts the trajectory that matches the detected prop position. */
    protected void followForCase(TrajectorySequence case1, TrajectorySequence case2, TrajectorySequence case3) {
        if (propCase == 1) {
            drive.followTrajectorySequenceAsync(case1);
        } else if (propCase == 2) {
            drive.followTrajectorySequenceAsync(case2);
        } else if (propCase == 3) {
            drive.followTrajectorySequenceAsync(case3);
        }
    }

    protected void updateRobot() {
        s.update();
        drive.update();
    }

    protected static void pause(int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException ignored) {
        }
    }
}
