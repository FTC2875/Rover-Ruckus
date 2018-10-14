package vision;

import org.opencv.core.Mat;
import org.opencv.core.Point;

/**
 * Stores the result of the Yellow Block Processing
 * Results are held in a 3 long array of Mats
 *
 * 1st Mat: HSV-thresholded image
 * 2nd Mat: Normal camera image with with point on center yellow block if it finds it
 * 3rd Mat: Image of just the contours that OpenCV detects from the thresholded image
 *
 */
public class YellowBlockResult {
    private Mat[] results;
    private double blockArea;
    private boolean foundBlock;
    private Point point;

    public YellowBlockResult(Mat thresholdedImage, Mat imageWithPoint, Mat matOnlyContours, double blockArea, Point center) {
        results = new Mat[3];

        results[0] = thresholdedImage;
        results[1] = imageWithPoint;
        results[2] = matOnlyContours;

        this.blockArea = blockArea;
        this.foundBlock = true;
        this.point = center;
    }

    public Point getPoint() {
        return point;
    }

    public YellowBlockResult(Mat thresholdedImage) {
        this.foundBlock = false;
        results = new Mat[1];
        results[0] = thresholdedImage;
    }

    public YellowBlockResult() {
        results = new Mat[0];
        this.foundBlock = false;
    }

    public Mat[] getResults() {
        return results;
    }

    public double getBlockArea() {
        return blockArea;
    }

    public boolean isFoundBlock() {
        return foundBlock;
    }

    public void releaseResults() {
        for (Mat mat : results) {
            mat.release();
        }
    }
}
