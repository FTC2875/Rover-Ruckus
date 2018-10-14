package vision;

import android.graphics.Bitmap;
import android.util.Log;
import android.view.View;
import android.widget.ImageView;

import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

/**
 * This class provides OpModes and object to interact with vision tools
 *
 */
public class VisionController {
    private final static String TAG = "VisionController";

    private VuforiaLocalizer vuforia;
    private YellowBlockResult result;
    private int resultMatIndex;

    private ImageView demoView;

    public VisionController(VuforiaLocalizer vuforia, ImageView demoView) {
        this.vuforia = vuforia;
        this.demoView = demoView;
        result = new YellowBlockResult(); // intializes empty result with empty mat array

        resultMatIndex = 0;
    }


    private Mat vuforiatoCV() throws InterruptedException {
        Image rgb = null;
        VuforiaLocalizer.CloseableFrame frame;


        frame = vuforia.getFrameQueue().take();
        //takes the frame at the head of the queue
        long numImages = frame.getNumImages();

        for (int i = 0; i < numImages; i++) {
            if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                rgb = frame.getImage(i);
                break;
            }
        }

        /*rgb is now the Image object that weve used in the video*/
        final Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
        bm.copyPixelsFromBuffer(rgb.getPixels());

        //put the image into a MAT for OpenCV
        Mat tmp = new Mat(rgb.getWidth(), rgb.getHeight(), CvType.CV_8UC4);
        Utils.bitmapToMat(bm, tmp);

//            saveMat(bm);
//            setDemoImage(bm);

        return tmp;

    }

    private Bitmap matToBitmap(Mat mat) {
        final Bitmap bm = Bitmap.createBitmap(mat.width(), mat.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(mat, bm);

        return bm;
    }

    // changes the demo view to the given bitmap
    public void setDemoImage(Bitmap bm) {
        final ImageView demoView = FtcRobotControllerActivity.getDemoView();
        final Bitmap bmFinal = bm;

        demoView.post(new Runnable() {
            @Override
            public void run() {
                demoView.setImageBitmap(bmFinal);
            }
        });
    }

    public void toggleDemoView() {
        final ImageView demoView = FtcRobotControllerActivity.getDemoView();

        if (demoView.getVisibility() == View.VISIBLE) {
            demoView.post(new Runnable() {
                @Override
                public void run() {
                    demoView.setVisibility(View.INVISIBLE);
                }
            });
        } else {
            demoView.post(new Runnable() {
                @Override
                public void run() {
                    demoView.setVisibility(View.VISIBLE);
                }
            });
        }
    }

    public void processFrame() {
        try {
            result.releaseResults(); // clear up Mat memory

            Mat mat = vuforiatoCV();
            YellowBlockAnaylzer processor = new YellowBlockAnaylzer(mat);
            result = processor.process();

            updateDemo();
        } catch (InterruptedException ie) {
            Log.e(TAG, "runOpMode: " + ie.getMessage());
        }
    }

    public void swapDemos() {

        if (resultMatIndex == 2)
            resultMatIndex = 0;
        else
            resultMatIndex++;

        updateDemo();

    }

    private void updateDemo() {
        if (result.isFoundBlock()) {

            Mat resultImage = result.getResults()[resultMatIndex];
            if (resultImage.width() > 0 && resultImage.height() > 0) {
                Bitmap bitmapResultImage = matToBitmap(resultImage);
                setDemoImage(bitmapResultImage);
            }


        } else {                              // no block found, show openCV image
            if (result.getResults().length > 0) {
                Mat resultImage = result.getResults()[0];

                if (resultImage.width() > 0 && resultImage.height() > 0) {
                    Bitmap bitmapResultImage = matToBitmap(resultImage);
                    setDemoImage(bitmapResultImage);
                }

            }
        }
    }

    public YellowBlockResult getResult() {
        return result;
    }
}
