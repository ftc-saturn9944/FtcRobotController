
package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.graphics.ImageFormat;
import android.os.Handler;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
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
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
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


@Autonomous(name="Color Park", group="Linear Opmode")
//@Disabled
public class ColorPark extends LinearOpMode {

    //Camera stuff
    private static final String TAG = "Webcam Sample";
    private static final int secondsPermissionTimeout = Integer.MAX_VALUE;
    private CameraManager cameraManager;
    private WebcamName cameraName;
    private Camera camera;
    private CameraCaptureSession cameraCaptureSession;
    private EvictingBlockingQueue<Bitmap> frameQueue;
    private int captureCounter = 0;
    private File captureDirectory = AppUtil.ROBOT_DATA_DIR;
    private Handler callbackHandler;
    Orientation angles;
    String color = "Purple";

    //robot setup
    public DcMotor leftFront = null;
    public DcMotor leftRear = null;
    public DcMotor rightFront = null;
    public DcMotor rightRear = null;
    public DcMotorEx lift = null;
    public Servo gripper = null;
    public Servo wrist = null;



    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // Camera intializations
        callbackHandler = CallbackLooper.getDefault().getHandler();

        cameraManager = ClassFactory.getInstance().getCameraManager();
        cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        initializeFrameQueue(2);
        AppUtil.getInstance().ensureDirectoryExists(captureDirectory);

        // Wait for the game to start (driver presses PLAY)
        //waitForStart();
        //runtime.reset();

        //setup robot
        leftFront = hardwareMap.get(DcMotor.class,"LEFTFRONT");
        leftRear = hardwareMap.get(DcMotor.class, "LEFTREAR");
        rightFront = hardwareMap.get(DcMotor.class, "RIGHTFRONT");
        rightRear = hardwareMap.get(DcMotor.class, "RIGHTREAR" );
        lift = hardwareMap.get(DcMotorEx.class, "LIFTMOTOR");
        gripper = hardwareMap.get(Servo.class, "GRIPPER");
        wrist = hardwareMap.get(Servo.class, "WRIST");


        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        gripper.setPosition(0.8);

        try {
            openCamera();
            if (camera == null) return;

            startCamera();
            if (cameraCaptureSession == null) return;

            telemetry.addData(">", "Press Play to start");
            telemetry.update();
            waitForStart();
            telemetry.clear();


            //wrist.setPosition(0.05);

            //int level = 3;

            Bitmap bmp = frameQueue.poll();
            if (bmp != null) {
                //onNewFrame(bmp);
                int y = 285;
                for (int x = 355; x < 390; x+=2) {//390-440
                    int pixel = bmp.getPixel(x, y);

                    int r = Color.red(pixel);
                    int g = Color.green(pixel);
                    int b = Color.blue(pixel);

//                                if(170 < r && r < 195 && 220 < g && g < 251 && 135 < b && b <179){
//                                    telemetry.addData("TSE:", x + " " + y);
//                                    break;
//                                }
                        /*if ((r > 60 && r < 125) && (g > 115 && g < 165) && (b > 45 && b < 115)) {
                            color = "Green";
                        } else if ((r > 175 && r < 255) && (g > 180 && g < 255) && (b > 0 && b < 110)) {
                            color = "Yellow";
                        } else if ((r > 95 && r < 145) && (g > 50 && g < 105) && (b > 100 && b < 155)) {
                            color = "Purple";
                        }

                         */

                    if ((r > 60 && r < 125) && (g > 120 && g < 195) && (b > 45 && b < 165)) {
                        color = "Green";
                    } else if ((r > 175 && r < 255) && (g > 180 && g < 255) && (b > 0 && b < 110)) {
                        color = "Yellow";
                    } else if ((r > 80 && r < 145) && (g > 45 && g < 115) && (b > 95 && b < 170)) {
                        color = "Purple";
                    }
                    bmp.setPixel(x, y,89);

                    //telemetry.addData("RGB", r + ", " + g + ", " + b);
                    //telemetry.update();

                    //sleep(10000);
                    //telemetry.addData(">", "Not found");
                    //telemetry.update();
                }

                onNewFrame(bmp);
            }


            telemetry.addData("color seen: ", color);
            telemetry.update();

        } finally {
            closeCamera();
        }

        //sleep(2000);
        /*rightFront.setTargetPosition(1000);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setPower(0.5);
        sleep(2000);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

         */
        drive("right", 300);
        drive("forward", 1400);
        sleep(2000);
        drive("back", 300);
        if(color.equals("Yellow")) {
            drive("left", 1100);
        } else if(color.equals("Purple")) {
            sleep(2000);
        } else if(color.equals("Green")) {
            drive("right", 1600);
        }


    }

    //drive
    public void drive (String direction, int encoderDist) {
        if (direction.equals("right")) {
            leftFront.setTargetPosition(-encoderDist);
            leftRear.setTargetPosition(encoderDist);
            rightFront.setTargetPosition(encoderDist);
            rightRear.setTargetPosition(-encoderDist);

            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftFront.setPower(0.5);
            leftRear.setPower(0.5);
            rightFront.setPower(0.5);
            rightRear.setPower(0.5);


            while (opModeIsActive() & rightFront.isBusy() & leftFront.isBusy() & leftRear.isBusy() & rightRear.isBusy() ) {
                idle();
            }
        } else if (direction.equals("left")) {
            leftFront.setTargetPosition(encoderDist+500);
            leftRear.setTargetPosition(-encoderDist);
            rightFront.setTargetPosition(-encoderDist);
            rightRear.setTargetPosition(encoderDist+500);

            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            leftFront.setPower(0.7);//0.6
            leftRear.setPower(0.5);//0.5
            rightFront.setPower(0.5);
            rightRear.setPower(0.7);

            while (opModeIsActive() & rightFront.isBusy() & leftFront.isBusy() & leftRear.isBusy() & rightRear.isBusy() ) {
                idle();
            }

        } else if (direction.equals("lift")) {
            lift.setTargetPosition(encoderDist);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(4.0);
            while (opModeIsActive() & lift.isBusy()) {
                idle();
            }

        } else if (direction.equals("lowerlift")) {
            lift.setTargetPosition(-encoderDist);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(2.0);
            while (opModeIsActive() & lift.isBusy()) {
                idle();
            }

        } else if (direction.equals("forward")) {
            leftFront.setTargetPosition(encoderDist);
            leftRear.setTargetPosition(encoderDist);
            rightFront.setTargetPosition(encoderDist);
            rightRear.setTargetPosition(encoderDist);

            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            leftFront.setPower(0.3);
            leftRear.setPower(0.3);
            rightFront.setPower(0.3);
            rightRear.setPower(0.3);

            while (opModeIsActive() & rightFront.isBusy() & leftFront.isBusy() & leftRear.isBusy() & rightRear.isBusy() ) {
                idle();
            }
        } else if (direction.equals("back")) {
            leftFront.setTargetPosition(-encoderDist);
            leftRear.setTargetPosition(-encoderDist);
            rightFront.setTargetPosition(-encoderDist);
            rightRear.setTargetPosition(-encoderDist);
            //lift.setTargetPosition(-encoderDist);

            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftFront.setPower(0.3);
            leftRear.setPower(0.3);
            rightFront.setPower(0.3);
            rightRear.setPower(0.3);
            //lift.setPower(0.4);

            while (opModeIsActive() & rightFront.isBusy() & leftFront.isBusy() & leftRear.isBusy() & rightRear.isBusy()) {
                idle();
            }
        }
        else if (direction.equals("rotation")) {
            leftFront.setTargetPosition(-encoderDist);
            leftRear.setTargetPosition(-encoderDist);
            rightFront.setTargetPosition(encoderDist);
            rightRear.setTargetPosition(encoderDist);
            //lift.setTargetPosition(-encoderDist);

            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftFront.setPower(0.4);
            leftRear.setPower(0.4);
            rightFront.setPower(0.4);
            rightRear.setPower(0.4);
            //lift.setPower(0.4);

            while (opModeIsActive() & rightFront.isBusy() & leftFront.isBusy() & leftRear.isBusy() & rightRear.isBusy()) {
                idle();
            }
        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        lift.setPower(0);


        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }

    ///////////////////////// Camera Operation Functions

    /** Do something with the frame */
    private void onNewFrame (Bitmap frame){
        saveBitmap(frame);
        //frame.recycle(); // not strictly necessary, but helpful
    }

    public void initializeFrameQueue ( int capacity){
        /** The frame queue will automatically throw away bitmap frames if they are not processed
         * quickly by the OpMode. This avoids a buildup of frames in memory */
        frameQueue = new EvictingBlockingQueue<Bitmap>(new ArrayBlockingQueue<Bitmap>(capacity));
        frameQueue.setEvictAction(new Consumer<Bitmap>() {
            @Override
            public void accept(Bitmap frame) {
                // RobotLog.ii(TAG, "frame recycled w/o processing");
                frame.recycle(); // not strictly necessary, but helpful
            }
        });
    }

    public void openCamera () {
        if (camera != null) return; // be idempotent

        Deadline deadline = new Deadline(secondsPermissionTimeout, TimeUnit.SECONDS);
        camera = cameraManager.requestPermissionAndOpenCamera(deadline, cameraName, null);
        if (camera == null)
            error("camera not found or permission to use not granted: %s", cameraName);
    }


    public void startCamera () {
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
                @Override
                public void onConfigured(@NonNull CameraCaptureSession session) {
                    try {
                        /** The session is ready to go. Start requesting frames */
                        final CameraCaptureRequest captureRequest = camera.createCaptureRequest(imageFormat, size, fps);
                        session.startCapture(captureRequest,
                                new CameraCaptureSession.CaptureCallback() {
                                    @Override
                                    public void onNewFrame(@NonNull CameraCaptureSession session, @NonNull CameraCaptureRequest request, @NonNull CameraFrame cameraFrame) {
                                        /** A new frame is available. The frame data has <em>not</em> been copied for us, and we can only access it
                                         * for the duration of the callback. So we copy here manually. */
                                        Bitmap bmp = captureRequest.createEmptyBitmap();
                                        cameraFrame.copyToBitmap(bmp);
                                        frameQueue.offer(bmp);
                                    }
                                },
                                Continuation.create(callbackHandler, new CameraCaptureSession.StatusCallback() {
                                    @Override
                                    public void onCaptureSequenceCompleted(@NonNull CameraCaptureSession session, CameraCaptureSequenceId cameraCaptureSequenceId, long lastFrameNumber) {
                                        RobotLog.ii(TAG, "capture sequence %s reports completed: lastFrame=%d", cameraCaptureSequenceId, lastFrameNumber);
                                    }
                                })
                        );
                        synchronizer.finish(session);
                    } catch (CameraException | RuntimeException e) {
                        RobotLog.ee(TAG, e, "exception starting capture");
                        error("exception starting capture");
                        session.close();
                        synchronizer.finish(null);
                    }
                }
            }));
        } catch (CameraException | RuntimeException e) {
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

    public void stopCamera () {
        if (cameraCaptureSession != null) {
            cameraCaptureSession.stopCapture();
            cameraCaptureSession.close();
            cameraCaptureSession = null;
        }
    }

    private void closeCamera () {
        stopCamera();
        if (camera != null) {
            camera.close();
            camera = null;
        }
    }

    //----------------------------------------------------------------------------------------------
    // Utilities
    //----------------------------------------------------------------------------------------------

    public void error (String msg){
        telemetry.log().add(msg);
        telemetry.update();
    }
    public void error (String format, Object...args){
        telemetry.log().add(format, args);
        telemetry.update();
    }

    public boolean contains ( int[] array, int value){
        for (int i : array) {
            if (i == value) return true;
        }
        return false;
    }

    public void saveBitmap (Bitmap bitmap){
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



