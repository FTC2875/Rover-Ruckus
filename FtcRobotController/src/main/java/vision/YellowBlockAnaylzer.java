package vision;

import android.util.Log;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;

public class YellowBlockAnaylzer {
    private final String TAG = "Yellow Block Processor";
    private Mat image;
    private final static Scalar lowerBounds = new Scalar(18, 134, 45);
    private final static Scalar upperBounds = new Scalar(30, 255, 255);

    // outdated values
//    private final static Scalar lowerBounds = new Scalar(22, 81, 0);
//    private final static Scalar upperBounds = new Scalar(30, 255, 255);

    // low (18, 134, 45)
    // upper (30, 255, 255)

    public YellowBlockAnaylzer(Mat image) {
        this.image = image;
    }

    public YellowBlockResult process() {
        Log.i(TAG, "Starting to image process ball");
        Mat hsvImage = image.clone();
        Mat hierachy = image.clone();
        Mat thresholded = image.clone();

        List<MatOfPoint> contours = new ArrayList<>();


        // erode and dilate to remove noise
        Mat kernel = Mat.ones(5, 5, CvType.CV_8U);
        Mat morphedImage = new Mat(image.size(), image.type());
        Imgproc.morphologyEx(image, morphedImage, Imgproc.MORPH_OPEN, kernel);

        Imgproc.cvtColor(morphedImage, hsvImage, Imgproc.COLOR_RGB2HSV);

        // thresholding
        Core.inRange(hsvImage, lowerBounds, upperBounds, thresholded);

        //Log.d(TAG, "HSV: " + hsv.toString());

        Imgproc.findContours(thresholded, contours, hierachy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Mat matOnlyContours = new Mat(thresholded.size(), thresholded.type());
        Imgproc.drawContours(matOnlyContours, contours, -1, new Scalar(100, 100, 255));

        if (contours.size() > 0) {
            Log.i(TAG, "Contours: " + contours.size());
            MatOfPoint largestContour = findLargestContour(contours);

            double area = Imgproc.contourArea(largestContour);
            Moments contourMoments = Imgproc.moments(largestContour);

            int centerX = (int) (contourMoments.get_m10() / contourMoments.get_m00());
            int centerY = (int) (contourMoments.get_m01() / contourMoments.get_m00());

            Point center = new Point(centerX, centerY);
            Imgproc.circle(image, center, 5, new Scalar(150, 8, 206), 15);

            return new YellowBlockResult(thresholded, image, matOnlyContours, area, center);
        } else {
            Log.i(TAG, "No contours found");

            hierachy.release();
            hsvImage.release();
            matOnlyContours.release();

            return new YellowBlockResult(thresholded);
        }
    }

    private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
        MatOfPoint largest = contours.get(0);

        for (MatOfPoint contour : contours) {
            if (contour.size().area() > largest.size().area()) {
                largest = contour;
            }
        }

        return largest;
    }




}
