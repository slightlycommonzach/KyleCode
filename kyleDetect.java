package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class kyleDetect extends OpenCvPipeline {
    /*
     * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
     * highly recommended to declare them here as instance variables and re-use them for
     * each invocation of processFrame(), rather than declaring them as new local variables
     * each time through processFrame(). This removes the danger of causing a memory leak
     * by forgetting to call mat.release(), and it also reduces memory pressure by not
     * constantly allocating and freeing large chunks of memory.
     */

    @Override
    public Mat processFrame(Mat input) {

        Scalar LOWER_LIMIT_HSV = new Scalar(5, 125, 125);
        Scalar UPPER_LIMIT_HSV = new Scalar(50, 255, 255); //Create a upper and lower hsv value limit to specify a color
        Mat mask = new Mat(); // create a mask that only shows the color requested
        Mat hsv = new Mat();
        Imgproc.GaussianBlur(input, hsv, new Size(35, 35), 0); // Blur the original image to have less minor contour lines
        Imgproc.cvtColor(hsv, hsv, Imgproc.COLOR_RGB2HSV);// transfer received rgb value and convert to hsv
        Core.inRange(hsv, LOWER_LIMIT_HSV, UPPER_LIMIT_HSV, mask);
        ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Mat hirarchy = new Mat();
        Imgproc.findContours(mask, contours, hirarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        //use mask to find object location
        double biggest = 0;
        Mat output = new Mat();
        int bigPos = 0;
        for (int contourIdx = 0; contourIdx < contours.size(); contourIdx++) {
            if (contours.get(contourIdx).size().area() > biggest)  // Minimum size allowed for consideration
            {
                bigPos = contourIdx;
                biggest = contours.get(contourIdx).size().area();
            }
        }
        Imgproc.drawContours(input, contours, bigPos, new Scalar(0, 255, 255), 3);
        if (contours.size() > 0) {
            Moments p = Imgproc.moments(contours.get(bigPos), false);// Retrieve the x,y coordinate from the center of the contoured object
            double x = (int) (p.get_m10() / p.get_m00());
            double y = (int) (p.get_m01() / p.get_m00());
            double cameraPixelCenterX = 160;//get the pixel x resolution from phone
            double cameraPixelCenterY = 120;//get the pixel y resolution from phone
            double camAngle = Math.atan(((x-cameraPixelCenterX)/y-cameraPixelCenterY));//get the angle of that difference coordinate
            double planarZ = Math.hypot((((x-cameraPixelCenterX))),((y-cameraPixelCenterY))); //get the distance from the coordinate to the phone center
            double finalDistance = (planarZ/camAngle);// get raw distance using opposite over yaw angle
        }
        return input;

    }


}
