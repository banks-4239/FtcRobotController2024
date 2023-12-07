package org.firstinspires.ftc.teamcode;



import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous
public class BasicAuto extends LinearOpMode {

    public ColorDetection.BlueDeterminationPipeline pipelineBlue;
    public ColorDetection.RedDeterminationPipeline pipelineRed;
    private ElapsedTime runtime = new ElapsedTime();
    Object placement;
    OpenCvCamera camera;
    @Override
    public void runOpMode() {

     //   MecanumDrive robot = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // if statements for the element being in the right zone, change placment's value.

        //if(isRight==true){
        //placement==2;
        //}
        //else if(isLeft==true){
        //placement==1;
        //}

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,"Webcam 1"), cameraMonitorViewId);
        pipelineBlue = new ColorDetection.BlueDeterminationPipeline();
        pipelineRed = new ColorDetection.RedDeterminationPipeline();
        camera.setPipeline(pipelineBlue);

//        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
//        // out when the RC activity is in portrait. We do our actual image processing assuming
//        // landscape orientation, though.
//        camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280,960, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        // Wait for the game to start (driver presses PLAY)
       // waitForStart();
        while (!isStarted() &&!isStopRequested())
        {
            placement = pipelineBlue.getAnalysis();
            telemetry.addData("Analysis", placement);
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive forward for 3 seconds
//        robot.leftBack.setPower  (0.3);
//        robot.rightBack.setPower (0.3);
//        robot.leftFront.setPower (0.3);
//        robot.rightFront.setPower(0.3);
//        runtime.reset();
//
//        while (opModeIsActive() && !isStopRequested() && (runtime.seconds() < 2.0)) {
//            telemetry.addData("Running", true);
//            telemetry.update();
//        }
//
//
//        // Step 4:  Stop
//        robot.leftBack.setPower(0);
//        robot.rightBack.setPower(0);
//        robot.leftFront.setPower(0);
//        robot.rightFront.setPower(0);
//        robot.scoringServo.setPosition(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);

    }
}
