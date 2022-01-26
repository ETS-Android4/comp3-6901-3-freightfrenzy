
package org.firstinspires.ftc.teamcode.auto.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.RoadRunnerImprovedTankDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequenceimproved.TrajectorySequence;

@Config
@Autonomous(name="newRedWareHousePark", group="Roadrunner Paths")
 public class RedPathPark extends LinearOpMode {
    @Override
    public void runOpMode() {
        RoadRunnerImprovedTankDrive drive = new RoadRunnerImprovedTankDrive(this);

        // On start

        waitForStart();
        if(isStopRequested()) return;


        Pose2d startPose = new Pose2d(-8, -60, Math.toRadians(-90));
        ElapsedTime timer = new ElapsedTime();

        drive.setPoseEstimate(startPose);

        TrajectorySequence Trajectory1 = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineTo(new Vector2d(-11, -40), Math.toRadians(90))
                .waitSeconds(2)
                .setReversed(false)
                .splineTo(new Vector2d( 33, -65), Math.toRadians(0))
                .forward(5)
                .waitSeconds(1)
//                                .splineTo(new Vector2d(56,56), Math. toRadians(0))


                .waitSeconds(1)
                .build();




        // Run trajectory 1
        drive.followTrajectorySequence((Trajectory1));

    }
}
