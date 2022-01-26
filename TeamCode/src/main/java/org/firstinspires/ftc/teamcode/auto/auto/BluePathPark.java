//package org.firstinspires.ftc.teamcode.auto.auto;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.drive.Robot;
//import org.firstinspires.ftc.teamcode.trajectorysequenceimproved.TrajectorySequence;
//
//public class BluePathPark extends LinearOpMode {
//    Pose2d startPose = new Pose2d(-11, 67, Math.toRadians(90));
//
//
//    @Override
//    public void runOpMode() {
//        Robot drive = new Robot(this, startPose);
//
//
//        TrajectorySequence myTrajectory = drive.trajectorySequenceBuilder(startPose)
//                .setReversed(true)
//                .splineTo(new Vector2d(-10, 40), Math.toRadians(-90))
//                .waitSeconds(2)
//                .setReversed(false)
//                .splineTo(new Vector2d( 33, 65), Math.toRadians(0))
//                .forward(5)
//                .waitSeconds(1)
//                .build();
//
//        waitForStart();
//
//        if(isStopRequested()) return;
//
//        drive.followTrajectorySequence(myTrajectory);
//    }
//}
