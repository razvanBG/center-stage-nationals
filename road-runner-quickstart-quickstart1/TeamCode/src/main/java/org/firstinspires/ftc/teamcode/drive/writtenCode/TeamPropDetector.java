package org.firstinspires.ftc.teamcode.drive.writtenCode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

/**
 * Finds which spike mark holds our team prop.
 *
 * The camera is mounted sideways, so the frame is split into three bands along its long side,
 * one per spike mark. The frame is converted to YCrCb and the chroma channel of our alliance
 * color is thresholded. The band with the most bright pixels is where the prop is,
 * reported as case 1, 2 or 3.
 */
public class TeamPropDetector {
    public static int resolutionX = 1280;
    public static int resolutionY = 720;
    public int binaryThreshold = 170;

    public OpenCvWebcam webcam;
    /** Last result computed by the pipeline. */
    public int detectedCase = 0;

    private final HardwareMap hardwareMap;
    private final String alliance;

    private Point band1Start, band1End, band2Start, band2End, band3Start, band3End;
    private int band1Score = 0, band2Score = 0, band3Score = 0;

    /** @param alliance "red" or "blue" */
    public TeamPropDetector(HardwareMap hardwareMap, String alliance) {
        this.hardwareMap = hardwareMap;
        this.alliance = alliance;
    }

    /** Opens the webcam and starts streaming frames into the pipeline. */
    public void startCamera() {
        int third = resolutionX / 3;
        band1Start = new Point(0, 0);
        band1End = new Point(resolutionY, third);
        band2Start = new Point(0, third);
        band2End = new Point(resolutionY, third * 2);
        band3Start = new Point(0, third * 2);
        band3End = new Point(resolutionY, resolutionX);

        int monitorId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), monitorId);
        webcam.setPipeline(new PropPipeline());
        webcam.openCameraDevice();
        webcam.startStreaming(resolutionX, resolutionY, OpenCvCameraRotation.SIDEWAYS_RIGHT);
    }

    /** The band with the highest score wins. Ties favour the lower band number. */
    public int getCase() {
        if (band3Score > Math.max(band1Score, band2Score)) {
            return 3;
        }
        return band2Score > band1Score ? 2 : 1;
    }

    private class PropPipeline extends OpenCvPipeline {
        private final Mat ycrcb = new Mat();
        private final Mat chroma = new Mat();
        private boolean viewportPaused;

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, ycrcb, Imgproc.COLOR_RGB2YCrCb);
            // Cb (index 2) responds to blue, Cr (index 1) to red.
            if ("blue".equals(alliance)) {
                Core.extractChannel(ycrcb, chroma, 2);
            } else if ("red".equals(alliance)) {
                Core.extractChannel(ycrcb, chroma, 1);
            }

            Mat mask = new Mat(chroma.rows(), chroma.cols(), chroma.type(), new Scalar(0));
            Imgproc.threshold(chroma, mask, binaryThreshold, 255, Imgproc.THRESH_BINARY);

            Mat band1 = mask.submat(new Rect(band1Start, band1End));
            Mat band2 = mask.submat(new Rect(band2Start, band2End));
            Mat band3 = mask.submat(new Rect(band3Start, band3End));
            int score1 = (int) Core.mean(band1).val[0];
            int score2 = (int) Core.mean(band2).val[0];
            int score3 = (int) Core.mean(band3).val[0];

            // Draw the bands on the camera preview.
            Imgproc.rectangle(input, band1Start, band1End, new Scalar(0, 0, 255), 3);
            Imgproc.rectangle(input, band2Start, band2End, new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(input, band3Start, band3End, new Scalar(255, 0, 0), 3);

            band1Score = score1;
            band2Score = score2;
            band3Score = score3;
            detectedCase = getCase();
            return mask;
        }

        @Override
        public void onViewportTapped() {
            viewportPaused = !viewportPaused;
            if (viewportPaused) {
                webcam.pauseViewport();
            } else {
                webcam.resumeViewport();
            }
        }
    }
}
