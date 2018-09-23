package vision;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Mat;

/**
 * This class just takes in a frame and outputs it to the RC app
 */
public class SimpleFrameOutputter implements CameraBridgeViewBase.CvCameraViewListener2 {
    @Override
    public void onCameraViewStarted(int width, int height) {

    }

    @Override
    public void onCameraViewStopped() {

    }

    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        return inputFrame.rgba();
    }
}
