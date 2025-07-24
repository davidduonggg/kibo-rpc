package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.graphics.Bitmap;

import org.opencv.android.Utils;
import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.tensorflow.lite.DataType;
import org.tensorflow.lite.Interpreter;
import org.tensorflow.lite.support.common.FileUtil;
import org.tensorflow.lite.support.common.ops.NormalizeOp;
import org.tensorflow.lite.support.image.ImageProcessor;
import org.tensorflow.lite.support.image.TensorImage;
import org.tensorflow.lite.support.image.ops.ResizeOp;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.MappedByteBuffer;
import java.util.ArrayList;
import java.util.List;

import gov.nasa.arc.astrobee.Result;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee.
 */

public class YourService extends KiboRpcService {
    private final String[] TEMPLATE_NAME = {
            "coin",
            "compass",
            "coral",
            "crystal",
            "diamond",
            "emerald",
            "fossil",
            "key",
            "letter",
            "shell",
            "treasure_box",
    };
    private final int LOOP_MAX = 5;
    @Override
    protected void runPlan1() {

        // The mission starts.
        api.startMission();

        // instantiate image handler class
        ImageHandler imgHandler = new ImageHandler(this);

        // Move to area 1
        moveToWrapper(Area.AREA_1, Area.AREA_1_QUATERNION);
        inferenceWrapper(imgHandler, 1, Area.AREA_1, Area.AREA_1_QUATERNION);

        moveToWrapper(Area.AREA_2, Area.AREA_2_QUATERNION);
        inferenceWrapper(imgHandler, 2, Area.AREA_2, Area.AREA_2_QUATERNION);

        moveToWrapper(Area.AREA_3, Area.AREA_3_QUATERNION);
        inferenceWrapper(imgHandler, 3, Area.AREA_3, Area.AREA_3_QUATERNION);

        moveToWrapper(Area.AREA_4, Area.AREA_4_QUATERNION);
        inferenceWrapper(imgHandler, 4, Area.AREA_4, Area.AREA_4_QUATERNION);


        // When you move to the front of the astronaut, report the rounding completion.
        api.moveTo(Area.ASTRONAUT_POINT, Area.ASTRONAUT_ORIENTATION, false);
        api.reportRoundingCompletion();
        Area.TreasureLocation treasureLocation = astronautItem(imgHandler, Area.ASTRONAUT_ORIENTATION);
        if (treasureLocation != null) {
            api.notifyRecognitionItem();
            moveToWrapper(treasureLocation.treasurePoint, treasureLocation.treasureQuaternion);
        }
        else {
            api.notifyRecognitionItem();
        }

        // Take a snapshot of the target item.
        api.takeTargetItemSnapshot();
    }

    @Override
    protected void runPlan2() {
        // write your plan 2 here.
    }

    @Override
    protected void runPlan3() {
        // write your plan 3 here.
    }

    private void moveToWrapper(Point p, Quaternion q) {
        int i = 0;
        Result result = api.moveTo(p, q, false);
        while (!result.hasSucceeded() && i < LOOP_MAX) {
            // retry
            result = api.moveTo(p, q, false);
            ++i;
        }
    }

    public class ImageHandler { // remove when finished testing
        private Interpreter tflite;
        private final float[][][] outputBuffer = new float[1][300][6];
        private final TensorImage tensorImage = new TensorImage(DataType.FLOAT32);
        private final ImageProcessor processor = new ImageProcessor.Builder()
                // can chain TensorOperators and ImageOperators, used to resize and normalize bitmap
                .add(new ResizeOp(640, 640, ResizeOp.ResizeMethod.BILINEAR))
                .add(new NormalizeOp(0f, 255f))
                .build();

        public ImageHandler(android.content.Context activity) {
            try {
                MappedByteBuffer modelBuffer = FileUtil.loadMappedFile(activity,
                        "best_int8.tflite");
                tflite = new Interpreter(modelBuffer);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        private int photos = 0;
        private float markerLength = 4.5f; // 4.5 cm
        private float posterLength = 27.0f;
        private float posterHeight = 15.0f;
        public processRes process(Mat image, double[][] intrinsics, Quaternion areaQ) {
            // ArUco detection
            Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
            List<Mat> corners = new ArrayList<>();
            Mat markerIds = new Mat();

            // undistort
            Mat cameraMatrix = new Mat(3, 3, CvType.CV_64F);
            Mat cameraCoefficients = new Mat(1, 5, CvType.CV_64F);
            cameraMatrix.put(0, 0, intrinsics[0]);
            cameraCoefficients.put(0, 0, intrinsics[1]);

            // Undistort image:
            Mat undistortImg = new Mat();
            Calib3d.undistort(image, undistortImg, cameraMatrix, cameraCoefficients);
            api.saveMatImage(undistortImg, "photo" + photos + ".png");

            // detect aruco marker
            Aruco.detectMarkers(undistortImg, dictionary, corners, markerIds); // returns pixels

            // rotate images
            Mat finalImg = undistortImg;
            if (!corners.isEmpty()) {
                // use the first corner detected
                Mat corner = corners.get(0);
                org.opencv.core.Point[] pts = new MatOfPoint2f(corner).toArray();

                org.opencv.core.Point tl = pts[0], tr = pts[1], br = pts[2], bl = pts[3];

                // compute rotation angle
                double angle = Math.toDegrees(Math.atan2(tr.y - tl.y, tr.x - tl.x));

                // compute center of marker for rotation
                org.opencv.core.Point markerCenter = new org.opencv.core.Point(
                        (tl.x + br.x) / 2.0,
                        (tl.y + br.y) / 2.0
                );

                // rotate image
                Mat rotatedImg = new Mat();
                Mat M = Imgproc.getRotationMatrix2D(markerCenter, angle, 1.0);
                Imgproc.warpAffine(undistortImg, rotatedImg, M, undistortImg.size());
                api.saveMatImage(rotatedImg, "rotated" + photos + ".png");

                // crop logic
                double markerPxLength = Math.hypot(
                        tr.x - tl.x,
                        tr.y - tl.y
                );
                double cmToPx = markerPxLength / markerLength; // cm -> pixel

                // compute poster size in pixels
                int posterHeightPx = (int)(posterHeight * cmToPx);
                int posterLengthPx = (int)(posterLength * cmToPx);

                // determine offset
                org.opencv.core.Point offset = new org.opencv.core.Point(
                        // x+ -> right
                        // y+ -> down
                        -23.75f * cmToPx, // cm
                        -3.75f * cmToPx
                );
                // use top left of the poster as the anchor
                org.opencv.core.Point anchor = new org.opencv.core.Point(
                        markerCenter.x + offset.x,
                        markerCenter.y + offset.y
                );

                // create the crop
                Rect crop = new Rect( (int) anchor.x, (int) anchor.y, posterLengthPx, posterHeightPx);

                // make sure points don't go out of bounds
                int x = Math.max(crop.x, 0);
                int y = Math.max(crop.y, 0);

                int width = Math.min(crop.width, rotatedImg.cols() - x);
                int height = Math.min(crop.height, rotatedImg.rows() - y);

                if (width > 0 && height > 0) {
                    finalImg = new Mat(rotatedImg, new Rect(x, y, width, height));
                }
                else {
                    finalImg = rotatedImg;
                }
            }

            // orient towards marker for final process
            Quaternion newQ = areaQ;
            if (!corners.isEmpty()) {
                Mat tvecs = new Mat();
                Mat rvecs = new Mat();

                // fill in tvecs and rvecs
                Aruco.estimatePoseSingleMarkers(corners, markerLength, cameraMatrix, cameraCoefficients, rvecs, tvecs);
                double[] tvec = tvecs.get(0, 0);

                double dx = tvec[0];
                double dy = tvec[1];
                double dz = tvec[2];

                // compute yaw and pitch
                double yaw   = Math.atan2(dx, dz);
                double pitch = -Math.atan2(dy, Math.sqrt(dx * dx + dz * dz));

                // convert to quaternion
                Quaternion qYaw   = Utilities.eulerToQuaternion(0, yaw, 0);
                Quaternion qPitch = Utilities.eulerToQuaternion(pitch, 0, 0);
                Quaternion qFinal = Utilities.multiplyQuaternions(qYaw, qPitch);

                // get final quaternion
                newQ = Utilities.multiplyQuaternions(areaQ, qFinal);
            }

            // convert to bitmap
            Mat rgbaImg = new Mat();
            Imgproc.cvtColor(finalImg, rgbaImg, Imgproc.COLOR_BGR2RGBA);

            Bitmap bitmapImg = Bitmap.createBitmap(finalImg.cols(), finalImg.rows(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(rgbaImg, bitmapImg);

            // remove when finished testing
            api.saveBitmapImage(bitmapImg, "cropped" + photos++ + ".png");

            // run inference
            tensorImage.load(bitmapImg);
            ByteBuffer buffer = processor.process(tensorImage).getBuffer();
            tflite.run(buffer, outputBuffer);

            processRes res = new processRes(newQ);

            for (int i = 0; i < 300; i++) {
                float conf = outputBuffer[0][i][4];
                // if low confidence stop inferring
                // nms sorts confidence by descending order
                if (conf < 0.25f) break;

                // logic for detection
                int classId = (int) outputBuffer[0][i][5];
                // error handling
                if (classId < 0 || classId >= res.counts.length) continue;

                res.counts[classId]++;
            }
            return res;
        }
    }
    public class processRes {
        int[] counts;
        Quaternion q;

        public processRes(Quaternion quaternion) {
            counts = new int[11];
            q = quaternion;
        }
    }

    public void inferenceWrapper(ImageHandler imgHandler, int area_num, Point area, Quaternion quaternion) {
        api.flashlightControlFront(0.01f);
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        Mat image = api.getMatNavCam();
        double[][] intrinsics = api.getNavCamIntrinsics();

        // run inference
        processRes res = imgHandler.process(image, intrinsics, quaternion);
        for (int i = 0; i < res.counts.length; i++) {
            if (res.counts[i] > 0) {
                if (i >= 3 && i <= 5) {
                    Area.treasureLocations.put(TEMPLATE_NAME[i], new Area.TreasureLocation(area, res.q));
                }
                else {
                    api.setAreaInfo(area_num, TEMPLATE_NAME[i], res.counts[i]);

                }
            }
        }
    }

    private Area.TreasureLocation astronautItem(ImageHandler imgHandler, Quaternion quaternion) {
        api.flashlightControlFront(0.01f);
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        Mat image = api.getMatNavCam();
        double[][] intrinsics = api.getNavCamIntrinsics();

        // run inference
        processRes res = imgHandler.process(image, intrinsics, quaternion);
        for (int i = 3; i < 6; i++) {
            if (res.counts[i] > 0) {
                return Area.treasureLocations.get(TEMPLATE_NAME[i]);
            }
        }
        return null;
    }
}