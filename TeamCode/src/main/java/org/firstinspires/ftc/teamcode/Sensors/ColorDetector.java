package org.firstinspires.ftc.teamcode.Sensors;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;

import java.util.List;

public class ColorDetector {
    private ColorBlobLocatorProcessor colorLocator;
    private VisionPortal portal;
    private Telemetry telemetry;

    public ColorDetector(HardwareMap hardwareMap, Telemetry telemetry, ColorRange colorRange) {
        this.telemetry = telemetry;

        colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(colorRange)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.7))  // search central 1/4 of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                .build();
        portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, Constants.Ids.webcamName))
                .build();
    }

    public void colorDetectionsData() {

        // Read the current list
        List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();

        ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, blobs);  // filter out very small blobs.

        this.telemetry.addLine(" Area Density Aspect  Center");

        // Display the size (area) and center location for each Blob.
        for (ColorBlobLocatorProcessor.Blob b : blobs) {
            RotatedRect boxFit = b.getBoxFit();
            this.telemetry.addLine(String.format("%5d  %4.2f   %5.2f  (%3d,%3d)",
                    b.getContourArea(), b.getDensity(), b.getAspectRatio(), (int) boxFit.center.x, (int) boxFit.center.y));

            // Test
            this.telemetry.addData("Box Fit", boxFit.angle);
        }

        this.telemetry.update();
        try {
            Thread.sleep(50);
        } catch (Exception e) {
        }

    }

    public Point getClosestDetectionPoint() {
        // Get detections
        List<ColorBlobLocatorProcessor.Blob> detections = colorLocator.getBlobs();
        ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, detections);  // filter out very small blobs.

        if (detections == null || detections.isEmpty()) {
            return null;
        }

        Point closestDetectionPoint = null;
        double minimunDist = 0.0;

        // Camera Center point
        Point piecePositionCenter = new Point(160, 120);

        for (ColorBlobLocatorProcessor.Blob detection : detections) {
            RotatedRect boxFit = detection.getBoxFit();
            // calculate euclidean distance according to the camera center
            double distance = Math.sqrt(Math.pow((int) boxFit.center.x - piecePositionCenter.x, 2) + Math.pow((int) boxFit.center.y - piecePositionCenter.y, 2));

            // Verificar si esta detección es la más cercana hasta el momento
            if (closestDetectionPoint == null) {
                closestDetectionPoint = new Point(boxFit.center.x, boxFit.center.y);
                minimunDist = distance;
            } else if (distance < minimunDist) {
                minimunDist = distance;
                closestDetectionPoint = new Point(boxFit.center.x, boxFit.center.y);
            }
        }

        return closestDetectionPoint;  // Return the closest Detection Point
    }

    // Prueba para conocer orientación
    public ColorBlobLocatorProcessor.Blob getClosestDetection() {
        // Get detections
        List<ColorBlobLocatorProcessor.Blob> detections = colorLocator.getBlobs();
        ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, detections);  // filter out very small blobs.

        if (detections == null || detections.isEmpty()) {
            return null;
        }

        ColorBlobLocatorProcessor.Blob closestDetection = null;
        double minimunDist = 0.0;

        // Camera Center point
        Point cameraCenter = new Point(160, 120);

        for (ColorBlobLocatorProcessor.Blob detection : detections) {
            RotatedRect boxFit = detection.getBoxFit();
            // calculate euclidean distance according to the camera center
            double distance = Math.sqrt(Math.pow((int) boxFit.center.x - cameraCenter.x, 2) + Math.pow((int) boxFit.center.y - cameraCenter.y, 2));

            // Verificar si esta detección es la más cercana hasta el momento
            if (closestDetection == null) {
                minimunDist = distance;
                closestDetection = detection;

            } else if (distance < minimunDist) {
                minimunDist = distance;
                closestDetection = detection;

            }
        }

        return closestDetection;  // Return the closest Detection Point
    }

    public String getOrientationOfPiece(double angle, double aspectRatio) {

        String orientation;

        // Clasificar la orientación según el ángulo
        if ((angle >= -10 && angle <= 10) || (angle >= 170 && angle <= 180)) {
            orientation = "Horizontal";
        } else if ((angle >= 80 && angle <= 100) || (angle >= -100 && angle <= -80)) {
            orientation = "Vertical";
        } else {
            orientation = "Diagonal";
        }

        // Opcional: Validar con la relación de aspecto si es necesario
        if (aspectRatio < 1.2) {
            // Podrías considerar añadir una regla para piezas casi cuadradas
            orientation = "Ambigua (posiblemente cuadrada)";
        }

        return "La pieza está orientada: " + orientation;
    }
}

