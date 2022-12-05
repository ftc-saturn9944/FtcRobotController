package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.robotcore.internal.camera.WebcamExample.TAG;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.graphics.ImageFormat;
import android.os.Handler;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureRequest;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSequenceId;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraException;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.collections.EvictingBlockingQueue;
import org.firstinspires.ftc.robotcore.internal.network.CallbackLooper;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.system.ContinuationSynchronizer;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Locale;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.TimeUnit;

import com.arcrobotics.ftclib.command.SubsystemBase;


    public class CameraSubsystem extends SubsystemBase {

        private static final int secondsPermissionTimeout = Integer.MAX_VALUE;

        /** State regarding our interaction with the camera */
        private CameraManager cameraManager;
        private WebcamName cameraName;
        private Camera camera;
        private CameraCaptureSession cameraCaptureSession;

        private Bitmap bmp;
        /** The queue into which all frames from the camera are placed as they become available.
         * Frames which are not processed by the OpMode are automatically discarded. */
        private EvictingBlockingQueue<Bitmap> frameQueue;

        /** State regarding where and how to save frames when the 'A' button is pressed. */
        private int captureCounter = 0;
        private File captureDirectory = AppUtil.ROBOT_DATA_DIR;

        /** A utility object that indicates where the asynchronous callbacks from the camera
         * infrastructure are to run. In this OpMode, that's all hidden from you (but see {@link #startCamera}
         * if you're curious): no knowledge of multi-threading is needed here. */
        private Handler callbackHandler;


        public CameraSubsystem (HardwareMap hMap, String webcam){

            callbackHandler = CallbackLooper.getDefault().getHandler();

            cameraManager = ClassFactory.getInstance().getCameraManager();
            cameraName = hMap.get(WebcamName.class, webcam);

            initializeFrameQueue(2);
            AppUtil.getInstance().ensureDirectoryExists(captureDirectory);

        }
    public boolean isFinished(){
        return true;
    }

        public void initializeCamera() {
            openCamera();
            startCamera();
        }
        public Bitmap capture(){

            openCamera();
            startCamera();
            Bitmap bmp = frameQueue.poll();

            closeCamera();

            return bmp;
        }

        public int[] getRGBData(){
            startCamera();
            bmp = frameQueue.poll();
            if (bmp != null) {
                int y = 275;
                for (int x = 270; x < 280; x += 2) {
                    int pixel = bmp.getPixel(x, y);

                    int r = Color.red(pixel);
                    int g = Color.green(pixel);
                    int b = Color.blue(pixel);
                    return new int[]{r, g, b};
                }
                onNewFrame(bmp);
            }
            return new int[]{-1, -1, -1};
        }

        public String getRGBDataString() {
            int[] rgb = this.getRGBData();
            return rgb[0] + ", " + rgb[1] + ", " + rgb[2];
        }

        public String detectColor (){
            int[] rgb = this.getRGBData();
            int r = rgb[0];
            int g = rgb[1];
            int b = rgb[2];

            if ((r > 75 && r < 85) && (g > 110 && g < 130) && (b > 80 && b < 100)) {
                return "Green";
            } else if ((r > 210 && r < 230) && (g > 200 && g < 225) && (b > 100 && b < 120)) {
                return "Yellow";
            } else if ((r > 85 && r < 105) && (g > 54 && g < 70) && (b > 85 && b < 100)) {
                return "Purple";
            } else {
                return "Unknown";
            }
        }

        private void onNewFrame(Bitmap frame) {
            //saveBitmap(frame);
            frame.recycle(); // not strictly necessary, but helpful
        }
        public int getParking(Bitmap bmp){

            int level = 3;

            if (bmp != null) {

                        /*if(80 < r && r < 95 && 110 < g && g < 130 && 60 < b && b <80 && y < 240){
                            telemetry.addData("TSE:", x + " " + y);
                            telemetry.update();
                            if( x < 215){
                                level = 1;
                            } else if (x > 300 && x < 505){
                                level = 2;
                            }
                        }*/


                        //telemetry.addData("RGB", r + ", " + g + ", " + b);
                        //telemetry.update();

                        //sleep(10000);
                        //telemetry.addData(">", "Not found");
                        //telemetry.update();

                //onNewFrame(bmp);
            }


            return level;


        }

        private void initializeFrameQueue(int capacity) {
            /** The frame queue will automatically throw away bitmap frames if they are not processed
             * quickly by the OpMode. This avoids a buildup of frames in memory */
            frameQueue = new EvictingBlockingQueue<Bitmap>(new ArrayBlockingQueue<Bitmap>(capacity));
            frameQueue.setEvictAction(new Consumer<Bitmap>() {
                @Override public void accept(Bitmap frame) {
                    // RobotLog.ii(TAG, "frame recycled w/o processing");
                    frame.recycle(); // not strictly necessary, but helpful
                }
            });
        }

        private void openCamera() {
            if (camera != null) return; // be idempotent

            Deadline deadline = new Deadline(secondsPermissionTimeout, TimeUnit.SECONDS);
            camera = cameraManager.requestPermissionAndOpenCamera(deadline, cameraName, null);
            if (camera == null) {
                error("camera not found or permission to use not granted: %s", cameraName);
            }
        }

        private void startCamera() {
            if (cameraCaptureSession != null) return; // be idempotent

            /** YUY2 is supported by all Webcams, per the USB Webcam standard: See "USB Device Class Definition
             * for Video Devices: Uncompressed Payload, Table 2-1". Further, often this is the *only*
             * image format supported by a camera */
            final int imageFormat = ImageFormat.YUY2;

            /** Verify that the image is supported, and fetch size and desired frame rate if so */
            CameraCharacteristics cameraCharacteristics = cameraName.getCameraCharacteristics();
            if (!contains(cameraCharacteristics.getAndroidFormats(), imageFormat)) {
                error("image format not supported");
                return;
            }
            final Size size = cameraCharacteristics.getDefaultSize(imageFormat);
            final int fps = cameraCharacteristics.getMaxFramesPerSecond(imageFormat, size);

            /** Some of the logic below runs asynchronously on other threads. Use of the synchronizer
             * here allows us to wait in this method until all that asynchrony completes before returning. */
            final ContinuationSynchronizer<CameraCaptureSession> synchronizer = new ContinuationSynchronizer<>();
            try {
                /** Create a session in which requests to capture frames can be made */
                camera.createCaptureSession(Continuation.create(callbackHandler, new CameraCaptureSession.StateCallbackDefault() {
                    @Override public void onConfigured(@NonNull CameraCaptureSession session) {
                        try {
                            /** The session is ready to go. Start requesting frames */
                            final CameraCaptureRequest captureRequest = camera.createCaptureRequest(imageFormat, size, fps);
                            session.startCapture(captureRequest,
                                    new CameraCaptureSession.CaptureCallback() {
                                        @Override public void onNewFrame(@NonNull CameraCaptureSession session, @NonNull CameraCaptureRequest request, @NonNull CameraFrame cameraFrame) {
                                            /** A new frame is available. The frame data has <em>not</em> been copied for us, and we can only access it
                                             * for the duration of the callback. So we copy here manually. */
                                            Bitmap bmp = captureRequest.createEmptyBitmap();
                                            cameraFrame.copyToBitmap(bmp);
                                            frameQueue.offer(bmp);
                                        }
                                    },
                                    Continuation.create(callbackHandler, new CameraCaptureSession.StatusCallback() {
                                        @Override public void onCaptureSequenceCompleted(@NonNull CameraCaptureSession session, CameraCaptureSequenceId cameraCaptureSequenceId, long lastFrameNumber) {
                                            RobotLog.ii(TAG, "capture sequence %s reports completed: lastFrame=%d", cameraCaptureSequenceId, lastFrameNumber);
                                        }
                                    })
                            );
                            synchronizer.finish(session);
                        } catch (CameraException|RuntimeException e) {
                            RobotLog.ee(TAG, e, "exception starting capture");
                            error("exception starting capture");
                            session.close();
                            synchronizer.finish(null);
                        }
                    }
                }));
            } catch (CameraException|RuntimeException e) {
                RobotLog.ee(TAG, e, "exception starting camera");
                error("exception starting camera");
                synchronizer.finish(null);
            }

            /** Wait for all the asynchrony to complete */
            try {
                synchronizer.await();
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }

            /** Retrieve the created session. This will be null on error. */
            cameraCaptureSession = synchronizer.getValue();
        }

        private void stopCamera() {
            if (cameraCaptureSession != null) {
                cameraCaptureSession.stopCapture();
                cameraCaptureSession.close();
                cameraCaptureSession = null;
            }
        }

        private void closeCamera() {
            stopCamera();
            if (camera != null) {
                camera.close();
                camera = null;
            }
        }

        //----------------------------------------------------------------------------------------------
        // Utilities
        //----------------------------------------------------------------------------------------------

        private void error(String msg) {
//            telemetry.log().add(msg);
//            telemetry.update();
        }
        private void error(String format, Object...args) {
//            telemetry.log().add(format, args);
//            telemetry.update();
        }

        private boolean contains(int[] array, int value) {
            for (int i : array) {
                if (i == value) return true;
            }
            return false;
        }

        private void saveBitmap(Bitmap bitmap) {
            File file = new File(captureDirectory, String.format(Locale.getDefault(), "webcam-frame-%d.jpg", captureCounter++));
            try {
                try (FileOutputStream outputStream = new FileOutputStream(file)) {
                    bitmap.compress(Bitmap.CompressFormat.JPEG, 100, outputStream);
                    telemetry.log().add("captured %s", file.getName());
                }
            } catch (IOException e) {
                RobotLog.ee(TAG, e, "exception in saveBitmap()");
                error("exception saving %s", file.getName());
            }
        }
    }
