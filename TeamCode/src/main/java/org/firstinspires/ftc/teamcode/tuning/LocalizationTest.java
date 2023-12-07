package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Localizer;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;

public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();


        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            ThreeDeadWheelLocalizer localizer = new ThreeDeadWheelLocalizer(hardwareMap,0.0009412861137);
            waitForStart();

            while (opModeIsActive()) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
                        ),
                        -gamepad1.right_stick_x
                ));

                drive.updatePoseEstimate();

                //dashboardTelemetry.addData("x", drive.pose.position.x);
                dashboardTelemetry.addData("perp", localizer.perp.getPositionAndVelocity().position);
                dashboardTelemetry.addData("par0", localizer.par0.getPositionAndVelocity().position);
                dashboardTelemetry.addData("par1", localizer.par1.getPositionAndVelocity().position);
                dashboardTelemetry.addData("x", drive.pose.position.x);
                dashboardTelemetry.addData("y", drive.pose.position.y);
                double r = drive.pose.heading.real;
                double i = drive.pose.heading.imag;
                double head = Math.toDegrees(Math.atan(i/r));
                if(r<0) {
                    dashboardTelemetry.addData("heading", head+180);
                } else if(r>=0) {
                    dashboardTelemetry.addData("heading", head);
                }
                dashboardTelemetry.addData("heading", Math.toDegrees(Math.atan(drive.pose.heading.imag/drive.pose.heading.real)));
                dashboardTelemetry.update();
                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading", drive.pose.heading);
                telemetry.update();
            }
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while (opModeIsActive()) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                0.0
                        ),
                        -gamepad1.right_stick_x
                ));

                drive.updatePoseEstimate();

                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading", drive.pose.heading);
                telemetry.update();
            }
        } else {
            throw new AssertionError();
        }
    }
}
