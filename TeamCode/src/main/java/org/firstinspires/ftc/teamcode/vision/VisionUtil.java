package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.vision.CameraConstants.cameraHeight;
import static org.firstinspires.ftc.teamcode.vision.CameraConstants.cameraWidth;
import static org.firstinspires.ftc.teamcode.vision.CameraConstants.pixelsPerHorizontalDegree;
import static org.firstinspires.ftc.teamcode.vision.CameraConstants.pixelsPerVerticalDegree;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.List;


// Class to reduce boilerplate in vision pipelines by providing methods for common operations
public class VisionUtil {
    private Mat hierarchy = new Mat();
    private int colorSpace; // YCrCb is 37 and HSV is 41
    // Constructor
    public VisionUtil(int colorSpace){
        this.colorSpace = colorSpace;
    }

    // Color constants
    public Scalar
            red = new Scalar(255,0,0),
            orange = new Scalar(255,130,0),
            yellow = new Scalar(255,255,0),
            green = new Scalar(0,255,0),
            cyan = new Scalar(0,255,255),
            blue = new Scalar(0,0,255),
            magenta = new Scalar(255,0,255),
            black = new Scalar(0,0,0),
            white = new Scalar(255,255,255);

    // Blur a little and convert to YCrCb
    public Mat prepareFrame(Mat inputMat){
        Imgproc.blur(inputMat, inputMat, new Size(2,2));
        Imgproc.cvtColor(inputMat, inputMat, colorSpace);
        return inputMat;
    }

    public List<MatOfPoint> getContours(Mat inputmat, List<MatOfPoint> contours){
        Imgproc.findContours(inputmat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        return contours;
    }

    // Filters countours that are too small or too big
    public List<MatOfPoint> getFilteredContours(List<MatOfPoint> contours, int minLength, int maxLength){
        for (int i = 0; i < contours.size(); i++) {
            // If the length is less than the min or greater than the max, delete that contour
            if (contours.get(i).rows() < minLength || contours.get(i).rows() > maxLength){
                contours.remove(i);
                i--; // This line is hacky but I think it should work
            }
        }
        return contours;
    }

    // Scoring methods
    // In the scoring system, 0 is a perfect score and all scores are positive
    // The higher the score, the worse it is
    public double getAreaScore(RotatedRect rect, MatOfPoint contour, double idealScore){
        return Math.abs(idealScore - ((rect.size.width*rect.size.height) / Imgproc.contourArea(contour)));
    }
    // Eccentricity is the length longest side divided by the length of the shortest side
    public double getEccentricityScore(RotatedRect rect, double idealScore){
        return Math.abs(idealScore - (Math.max(rect.size.height / rect.size.width, rect.size.width / rect.size.height)));
    }

    // Why is drawing rotated rects so hard
    public void drawRotatedRect(RotatedRect rect, Mat outputMat, Scalar color, int thickness){
        Point[] rectPoints = new Point[4];
        rect.points(rectPoints);
        for (int i = 0; i < 4; i++) {
            Imgproc.line(outputMat, rectPoints[i], rectPoints[(i+1) % 4], color, thickness);
        }
    }

    // Why isn't this function in OpenCV in the first place
    public Point getRectCenter(Rect rect){
        return new Point(rect.tl().x + (rect.width/2.0),rect.tl().y + (rect.height/2.0));
    }

    // Position related methods
    // Height is weird because positive y points down
    public boolean isInHeightWindow(RotatedRect rect, int maxHeight, int minHeight){
        return rect.center.y > maxHeight && rect.center.y < minHeight;
    }
    public boolean isInWidthWindow(RotatedRect rect, int leftBound, int rightBound){
        return rect.center.x > leftBound && rect.center.x < rightBound;
    }

    // "corrected" means that the origin is in the center instead of the top left
    public double getCorrectedX(int x){
        return (x - (cameraWidth /2.0));
    }
    public double getCorrectedY(int y){
        return (y - (cameraHeight/2.0));
    }

    // Convert pixel positions to angles
    public double getHorizontalAngle(int xCoordinate){
        return (getCorrectedX(xCoordinate) * pixelsPerHorizontalDegree);
    }
    public double getVerticalAngle(int yCoordinate){
        return (getCorrectedY(yCoordinate) * pixelsPerVerticalDegree);
    }
}
