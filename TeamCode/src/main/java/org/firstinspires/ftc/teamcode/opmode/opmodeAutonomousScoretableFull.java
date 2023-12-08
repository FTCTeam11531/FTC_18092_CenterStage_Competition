package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.system.sysDrivetrainOdometryMecanum;
import org.firstinspires.ftc.teamcode.system.sysIntakeArm;
import org.firstinspires.ftc.teamcode.system.sysLighting;
import org.firstinspires.ftc.teamcode.system.sysVision;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utility.utilRobotConstants;

@Config
@Autonomous(name = "Autonomous - Scoretable - Full", group = "_auto", preselectTeleOp = "Teleop Main")
//@Disabled
public class opmodeAutonomousScoretableFull extends LinearOpMode {
    // ------------------------------------------------------------
    // System(s) - Define system and create instance of each system
    // ------------------------------------------------------------
    // -- Robot Initializtion

    // -- Lighting System
    sysLighting sysLighting = new sysLighting(this);

    // -- Drivetrain System
    // defined below

    // Vision System
    sysVision sysVision = new sysVision(this);

    // Intake / Arm
    sysIntakeArm sysIntakeArm = new sysIntakeArm(this);

    // ------------------------------------------------------------
    // Misc
    // ------------------------------------------------------------
    // -- Command Runtime
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        // ------------------------------------------------------------
        // Configure Telemetry
        // ------------------------------------------------------------
        // Set telemetry mode to append
        telemetry.setAutoClear(false);
        telemetry.clearAll();

        // Road Runner -- need to implement in system
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        // ------------------------------------------------------------
        // Initialize System(s) - set different light mode between each system init
        // ------------------------------------------------------------

        // Initialize System(s)
        sysLighting.init(utilRobotConstants.Configuration.ENABLE_LIGHTING);
        sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_SYSTEM_INIT_LIGHTING);

//        sysDrivetrain.init();
        sysDrivetrainOdometryMecanum sysDrivetrain = new sysDrivetrainOdometryMecanum(hardwareMap);
        sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_SYSTEM_INIT_DRIVETRAIN);

        sysVision.init();
        sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_SYSTEM_INIT_VISION);

        sysIntakeArm.init();
        sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_SYSTEM_INIT_INTAKEARM);

        // ------------------------------------------------------------
        // Configure drivetrain for Autonomous Mode
        // ------------------------------------------------------------

        // ------------------------------------------------------------
        // Variables for OpMode
        // ------------------------------------------------------------
        boolean isImageFound = false;
        int targetZone = 0, movementAmount = 0;

        Pose2d startPose, poseEstimate;

        HuskyLens.Block[] targetObjectList = null;
        HuskyLens.Block targetObject = null;

        // ------------------------------------------------------------
        // Send telemetry message to signify robot completed initialization and waiting to start;
        // ------------------------------------------------------------
        telemetry.addData(">", "------------------------------------");
        telemetry.addData(">", "All Systems Ready - Waiting to Start");
        telemetry.update();

        // Reset runtime clock
        runtime.reset();

        // System Initialization Complete
        sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_SYSTEM_INIT_COMPLETE);

        // Repeat while in initialize state (disable if using waitForStart)

        // Set AI Camera Mode
        sysVision.setAICameraMode(utilRobotConstants.Vision.AI_Camera.AI_CAMERA_MODE_OBJECT_TRACKING);

        // ------------------------------------------------------------
        // Configure Telemetry
        // ------------------------------------------------------------
        // Set telemetry mode to auto-clear
//        telemetry.setAutoClear(true);
        telemetry.clearAll();

        while(opModeInInit() && !isStopRequested()) {

            // Look for Team Element position
            targetObjectList = sysVision.getCameraObjectList();

            if(targetObjectList.length > 0) {

                targetObject = sysVision.getCameraObject(targetObjectList, 1);

                if(targetObject != null) {

                    if (targetObject.x < utilRobotConstants.Vision.RANDOM_TARGET_ZONE_ONE_X) {

                        targetZone = 1; // left
                        sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_ONE);

                    } else if (targetObject.x > utilRobotConstants.Vision.RANDOM_TARGET_ZONE_ONE_X && targetObject.x < utilRobotConstants.Vision.RANDOM_TARGET_ZONE_TWO_X) {
                        targetZone = 2; // center
                        sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_TWO);

                    } else if (targetObject.x > utilRobotConstants.Vision.RANDOM_TARGET_ZONE_TWO_X) {

                        targetZone = 3; // right
                        sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_THREE);

                    } else {

                        targetZone = 3;
                        sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_THREE);

                    }
                }
            }
            else {

                targetZone = 3;
                sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_THREE);
            }

            // ------------------------------------------------------------
            // Driver Hub Feedback
            // ------------------------------------------------------------
            telemetry.addData("Init Time", runtime.toString());
            // ------------------------------------------------------------
            // - Vision telemetry
            // ------------------------------------------------------------
            telemetry.addData("-", "------------------------------");
            telemetry.addData("-", "-- Vision");
            telemetry.addData("-", "------------------------------");
            telemetry.addData("Alliance Color", sysVision.getDetectedAllianceTagColor());
            telemetry.addData("-", "------------------------------");
            telemetry.addData("Camera Block Count", sysVision.getCameraObjectList().length);
            telemetry.addData("-", "------------------------------");
            telemetry.addData("Randomized Zone", targetZone);
            if (targetObject != null) {
                telemetry.addData("-", "------------------------------");
                telemetry.addData("-", "-- Target Object");
                telemetry.addData("-", "------------------------------");
                telemetry.addData("Target x:", targetObject.x);
                telemetry.addData("Target y:", targetObject.y);
                telemetry.addData("Target width:", targetObject.width);
                telemetry.addData("Target height:", targetObject.height);
                telemetry.addData("Target top:", targetObject.top);
                telemetry.addData("Target left:", targetObject.left);
            }

            // ------------------------------------------------------------
            // - send telemetry to driver hub
            // ------------------------------------------------------------
            telemetry.update();
        }

        // Set Trajectory Sequence based on Alliance
        if(sysVision.getDetectedAllianceTagColor() == "blue") {

            // Set Starting Pose for Blue Station Two
            startPose = new Pose2d(-64, 9, Math.toRadians(0));
        }
        else {

            // Set Starting Pose for Red Station Two
            startPose = new Pose2d(64, 9, Math.toRadians(180));
        }

        sysDrivetrain.setPoseEstimate(startPose);

//        telemetry.addData("Start Zone Settings", "");
//        telemetry.addData("Start Zone - X", startPose.getX());
//        telemetry.addData("Start Zone - Y", startPose.getY());
//        telemetry.addData("Start Zone - Heading", startPose.getHeading());
//        telemetry.update();

        // ----------------------------------------------------
        // Defined Trajectories
        // ----------------------------------------------------
        // -- start pose: Blue  - Pose2d(-64, -33, Math.toRadians(0))
        // -- start pose: Red   - Pose2d(64, -33, Math.toRadians(180))
        // ----------------------------------------------------

        // ----------------------------------------------------
        // Trajectory Sequence for - Blue Alliance - Audience Side - Zone One
        // ----------------------------------------------------
        TrajectorySequence trajSeqBlueAudienceZoneOne = sysDrivetrain.trajectorySequenceBuilder(startPose)
                // Move to Random Zone One
                .lineToSplineHeading(new Pose2d(-36, 30, Math.toRadians(270)))

                // Place Purple Pixel
                .addTemporalMarker(() -> {
                    sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_ONE, utilRobotConstants.IntakeArm.SERVO_SLOTONE_SETPOINT_OPEN);
                })
                .waitSeconds(2)

                // Back from zone
                .lineTo(new Vector2d(-36,38))

                // Move to Cycle Lane
//                .lineToSplineHeading(new Pose2d(-14, 39, Math.toRadians(85)))

                // Move Across Field - to other side
//                .lineToSplineHeading(new Pose2d(-14, 48, Math.toRadians(85)))

                // TODO: Check proper position
                // Move to board - Position One
                .lineToSplineHeading(new Pose2d(-40, 42, Math.toRadians(90)))

                // Action - Raise Arm
                .addTemporalMarker(() -> {

                    // Raise Arm
                    sysIntakeArm.moveIntakeToTarget(utilRobotConstants.IntakeArm.INTAKE_ENCODER_SETPOINT_AUTOPIXEL, utilRobotConstants.IntakeArm.INTAKE_MOTOR_OUTPUT_POWER_MAX);
                    sysIntakeArm.moveArmToTarget(utilRobotConstants.IntakeArm.ARM_ENCODER_SETPOINT_AUTOPIXEL, utilRobotConstants.IntakeArm.ARM_MOTOR_OUTPUT_POWER_MAX);

                })
                .waitSeconds(2)

                // Move a little closer to board
                .splineTo(
                        new Vector2d(-40, 46), Math.toRadians(90),
                        sysDrivetrain.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        sysDrivetrain.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )

                // Place Yellow Pixel
                .addTemporalMarker(() -> {
                    sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_TWO, utilRobotConstants.IntakeArm.SERVO_SLOTTWO_SETPOINT_OPEN);
                })
                .waitSeconds(2)

                // Lower Arm
                .addTemporalMarker(() -> {
                    sysIntakeArm.moveIntakeToTarget(utilRobotConstants.IntakeArm.INTAKE_ENCODER_SETPOINT_HOME, utilRobotConstants.IntakeArm.INTAKE_MOTOR_OUTPUT_POWER_MIN);
                    sysIntakeArm.moveArmToTarget(utilRobotConstants.IntakeArm.ARM_ENCODER_SETPOINT_HOME, utilRobotConstants.IntakeArm.ARM_MOTOR_OUTPUT_POWER_MIN);
                })
                .waitSeconds(2)

                // Back Away from the board
                .lineTo(new Vector2d(-40, 42))

                // Drive to park location
                .lineToSplineHeading(new Pose2d(-64, 42, Math.toRadians(0)))

                // Move a little closer to park position
                .strafeTo(new Vector2d(-64, 54))

                .build();

        // ----------------------------------------------------
        // Trajectory Sequence for - Blue Alliance - Audience Side - Zone Two
        // ----------------------------------------------------
        TrajectorySequence trajSeqBlueAudienceZoneTwo = sysDrivetrain.trajectorySequenceBuilder(startPose)
                // Move to Random Zone Two
                .lineToSplineHeading(new Pose2d(-26, 20, Math.toRadians(270)))

                // Place Purple Pixel
                .addTemporalMarker(() -> {
                    sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_ONE, utilRobotConstants.IntakeArm.SERVO_SLOTONE_SETPOINT_OPEN);
                })
                .waitSeconds(2)

                // Move away from Random Zone Two
                .lineTo(new Vector2d(-26, 38))

                // Move to Cycle Lane
//                .lineToSplineHeading(new Pose2d(-14, 39, Math.toRadians(87)))

                // Move Across Field - to other side
//                .lineToSplineHeading(new Pose2d(-14, 48, Math.toRadians(90)))

                // TODO: Set proper position
                // Move to board - Position Two
                .lineToSplineHeading(new Pose2d(-38, 42, Math.toRadians(90)))

                // Action - Raise Arm
                .addTemporalMarker(() -> {

                    // Raise Arm
                    sysIntakeArm.moveIntakeToTarget(utilRobotConstants.IntakeArm.INTAKE_ENCODER_SETPOINT_AUTOPIXEL, utilRobotConstants.IntakeArm.INTAKE_MOTOR_OUTPUT_POWER_MAX);
                    sysIntakeArm.moveArmToTarget(utilRobotConstants.IntakeArm.ARM_ENCODER_SETPOINT_AUTOPIXEL, utilRobotConstants.IntakeArm.ARM_MOTOR_OUTPUT_POWER_MAX);

                })
                .waitSeconds(2)

                // Move a little closer to board
                .splineTo(
                        new Vector2d(-38, 46), Math.toRadians(90),
                        sysDrivetrain.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        sysDrivetrain.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )

                // Place Yellow Pixel
                .addTemporalMarker(() -> {
                    sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_TWO, utilRobotConstants.IntakeArm.SERVO_SLOTTWO_SETPOINT_OPEN);
                })
                .waitSeconds(2)

                // Lower Arm
                .addTemporalMarker(() -> {
                    sysIntakeArm.moveIntakeToTarget(utilRobotConstants.IntakeArm.INTAKE_ENCODER_SETPOINT_HOME, utilRobotConstants.IntakeArm.INTAKE_MOTOR_OUTPUT_POWER_MIN);
                    sysIntakeArm.moveArmToTarget(utilRobotConstants.IntakeArm.ARM_ENCODER_SETPOINT_HOME, utilRobotConstants.IntakeArm.ARM_MOTOR_OUTPUT_POWER_MIN);
                })
                .waitSeconds(2)

                // Back Away from the board
                .lineTo(new Vector2d(-38, 42))

                // Drive to park location
                .lineToSplineHeading(new Pose2d(-64, 42, Math.toRadians(0)))

                // Move a little closer to park position
                .strafeTo(new Vector2d(-64, 54))

                .build();

        // ----------------------------------------------------
        // Trajectory Sequence for - Blue Alliance - Audience Side - Zone Three
        // ----------------------------------------------------
        TrajectorySequence trajSeqBlueAudienceZoneThree = sysDrivetrain.trajectorySequenceBuilder(startPose)
                // Move to Random Zone Three
                .lineToSplineHeading(new Pose2d(-36, 6, Math.toRadians(270)))

                // Place Purple Pixel
                .addTemporalMarker(() -> {
                    sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_ONE, utilRobotConstants.IntakeArm.SERVO_SLOTONE_SETPOINT_OPEN);
                })
                .waitSeconds(2)

                // Move away from Random Zone Three
                .lineTo(new Vector2d(-36, 38))

                // Move to Cycle Lane
//                .lineToSplineHeading(new Pose2d(-14, 39, Math.toRadians(87)))

                // Move Across Field - to other side
//                .lineToSplineHeading(new Pose2d(-14, 48, Math.toRadians(90)))

                // TODO: Set proper position
                // Move to board - Position Three
                .lineToSplineHeading(new Pose2d(-34, 42, Math.toRadians(90)))

                // Action - Raise Arm
                .addTemporalMarker(() -> {

                    // Raise Arm
                    sysIntakeArm.moveIntakeToTarget(utilRobotConstants.IntakeArm.INTAKE_ENCODER_SETPOINT_AUTOPIXEL, utilRobotConstants.IntakeArm.INTAKE_MOTOR_OUTPUT_POWER_MAX);
                    sysIntakeArm.moveArmToTarget(utilRobotConstants.IntakeArm.ARM_ENCODER_SETPOINT_AUTOPIXEL, utilRobotConstants.IntakeArm.ARM_MOTOR_OUTPUT_POWER_MAX);

                })
                .waitSeconds(2)

                // Move a little closer to board
                .splineTo(
                        new Vector2d(-34, 46), Math.toRadians(90),
                        sysDrivetrain.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        sysDrivetrain.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )

                // Place Yellow Pixel
                .addTemporalMarker(() -> {
                    sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_TWO, utilRobotConstants.IntakeArm.SERVO_SLOTTWO_SETPOINT_OPEN);
                })
                .waitSeconds(2)

                // Lower Arm
                .addTemporalMarker(() -> {
                    sysIntakeArm.moveIntakeToTarget(utilRobotConstants.IntakeArm.INTAKE_ENCODER_SETPOINT_HOME, utilRobotConstants.IntakeArm.INTAKE_MOTOR_OUTPUT_POWER_MIN);
                    sysIntakeArm.moveArmToTarget(utilRobotConstants.IntakeArm.ARM_ENCODER_SETPOINT_HOME, utilRobotConstants.IntakeArm.ARM_MOTOR_OUTPUT_POWER_MIN);
                })
                .waitSeconds(2)

                // Back Away from the board
                .lineTo(new Vector2d(-34, 42))

                // Drive to park location
                .lineToSplineHeading(new Pose2d(-64, 42, Math.toRadians(0)))

                // Move a little closer to park position
                .strafeTo(new Vector2d(-64, 54))

                .build();


        // ----------------------------------------------------
        // Trajectory Sequence for - Blue Alliance - Audience Side - Default (when detection goes wrong)
        // ----------------------------------------------------
        TrajectorySequence trajSeqBlueAudienceZoneDefault = trajSeqBlueAudienceZoneThree;


        // ----------------------------------------------------
        // Trajectory Sequence for - Red Alliance - Audience Side - Zone Three
        // ----------------------------------------------------
        TrajectorySequence trajSeqRedAudienceZoneThree = sysDrivetrain.trajectorySequenceBuilder(startPose)
                // Move to Random Zone One
                .lineToSplineHeading(new Pose2d(36, 30, Math.toRadians(270)))

                // Place Purple Pixel
                .addTemporalMarker(() -> {
                    sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_ONE, utilRobotConstants.IntakeArm.SERVO_SLOTONE_SETPOINT_OPEN);
                })
                .waitSeconds(2)

                // Move to Cycle Lane
//                .lineToSplineHeading(new Pose2d(14, 39, Math.toRadians(87)))

                // Move Across Field - to other side
//                .lineToSplineHeading(new Pose2d(14, 48, Math.toRadians(90)))

                // TODO: Set proper position
                // Move to board - Position Three
                .lineToSplineHeading(new Pose2d(40, 42, Math.toRadians(90)))

                // Action - Raise Arm
                .addTemporalMarker(() -> {

                    // Raise Arm
                    sysIntakeArm.moveIntakeToTarget(utilRobotConstants.IntakeArm.INTAKE_ENCODER_SETPOINT_AUTOPIXEL, utilRobotConstants.IntakeArm.INTAKE_MOTOR_OUTPUT_POWER_MAX);
                    sysIntakeArm.moveArmToTarget(utilRobotConstants.IntakeArm.ARM_ENCODER_SETPOINT_AUTOPIXEL, utilRobotConstants.IntakeArm.ARM_MOTOR_OUTPUT_POWER_MAX);

                })
                .waitSeconds(2)

                // Move a little closer to board
                .splineTo(
                        new Vector2d(40, 46), Math.toRadians(90),
                        sysDrivetrain.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        sysDrivetrain.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )

                // Place Yellow Pixel
                .addTemporalMarker(() -> {
                    sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_TWO, utilRobotConstants.IntakeArm.SERVO_SLOTTWO_SETPOINT_OPEN);
                })
                .waitSeconds(2)

                // Lower Arm
                .addTemporalMarker(() -> {
                    sysIntakeArm.moveIntakeToTarget(utilRobotConstants.IntakeArm.INTAKE_ENCODER_SETPOINT_HOME, utilRobotConstants.IntakeArm.INTAKE_MOTOR_OUTPUT_POWER_MIN);
                    sysIntakeArm.moveArmToTarget(utilRobotConstants.IntakeArm.ARM_ENCODER_SETPOINT_HOME, utilRobotConstants.IntakeArm.ARM_MOTOR_OUTPUT_POWER_MIN);
                })
                .waitSeconds(2)

                // Back Away from the board
                .lineTo(new Vector2d(40, 42))

                // Drive to park location
                .lineToSplineHeading(new Pose2d(64, 42, Math.toRadians(180)))

                // Move a little closer to park position
                .strafeTo(new Vector2d(64, 54))

                .build();

        // ----------------------------------------------------
        // Trajectory Sequence for - Red Alliance - Audience Side - Zone Two
        // ----------------------------------------------------
        TrajectorySequence trajSeqRedAudienceZoneTwo = sysDrivetrain.trajectorySequenceBuilder(startPose)
                // Move to Random Zone Two
                .lineToSplineHeading(new Pose2d(26, 20, Math.toRadians(270)))

                // Place Purple Pixel
                .addTemporalMarker(() -> {
                    sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_ONE, utilRobotConstants.IntakeArm.SERVO_SLOTONE_SETPOINT_OPEN);
                })
                .waitSeconds(2)

                // Move away from Random Zone Two
                .lineTo(new Vector2d(26, 38))

                // Move to Cycle Lane
//                .lineToSplineHeading(new Pose2d(-14, 39, Math.toRadians(87)))

                // Move Across Field - to other side
//                .lineToSplineHeading(new Pose2d(-14, 48, Math.toRadians(90)))

                // TODO: Set proper position
                // Move to board - Position Two
                .lineToSplineHeading(new Pose2d(38, 42, Math.toRadians(90)))

                // Action - Raise Arm
                .addTemporalMarker(() -> {

                    // Raise Arm
                    sysIntakeArm.moveIntakeToTarget(utilRobotConstants.IntakeArm.INTAKE_ENCODER_SETPOINT_AUTOPIXEL, utilRobotConstants.IntakeArm.INTAKE_MOTOR_OUTPUT_POWER_MAX);
                    sysIntakeArm.moveArmToTarget(utilRobotConstants.IntakeArm.ARM_ENCODER_SETPOINT_AUTOPIXEL, utilRobotConstants.IntakeArm.ARM_MOTOR_OUTPUT_POWER_MAX);

                })
                .waitSeconds(2)

                // Move a little closer to board
                .splineTo(
                        new Vector2d(38, 46), Math.toRadians(90),
                        sysDrivetrain.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        sysDrivetrain.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )

                // Place Yellow Pixel
                .addTemporalMarker(() -> {
                    sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_TWO, utilRobotConstants.IntakeArm.SERVO_SLOTTWO_SETPOINT_OPEN);
                })
                .waitSeconds(2)

                // Lower Arm
                .addTemporalMarker(() -> {
                    sysIntakeArm.moveIntakeToTarget(utilRobotConstants.IntakeArm.INTAKE_ENCODER_SETPOINT_HOME, utilRobotConstants.IntakeArm.INTAKE_MOTOR_OUTPUT_POWER_MIN);
                    sysIntakeArm.moveArmToTarget(utilRobotConstants.IntakeArm.ARM_ENCODER_SETPOINT_HOME, utilRobotConstants.IntakeArm.ARM_MOTOR_OUTPUT_POWER_MIN);
                })
                .waitSeconds(2)

                // Back Away from the board
                .lineTo(new Vector2d(38, 42))

                // Drive to park location
                .lineToSplineHeading(new Pose2d(64, 42, Math.toRadians(180)))

                // Move a little closer to park position
                .strafeTo(new Vector2d(64, 54))

                .build();

        // ----------------------------------------------------
        // Trajectory Sequence for - Red Alliance - Audience Side - Zone One
        // ----------------------------------------------------
        TrajectorySequence trajSeqRedAudienceZoneOne = sysDrivetrain.trajectorySequenceBuilder(startPose)
                // Move to Random Zone Three
                .lineToSplineHeading(new Pose2d(36, 6, Math.toRadians(270)))

                // Place Purple Pixel
                .addTemporalMarker(() -> {
                    sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_ONE, utilRobotConstants.IntakeArm.SERVO_SLOTONE_SETPOINT_OPEN);
                })
                .waitSeconds(2)

                // Move away from Random Zone Three
                .lineTo(new Vector2d(36, 38))

                // Move to Cycle Lane
//                .lineToSplineHeading(new Pose2d(14, 39, Math.toRadians(87)))

                // Move Across Field - to other side
//                .lineToSplineHeading(new Pose2d(14, 48, Math.toRadians(90)))

                // TODO: Set proper position
                // Move to board - Position One
                .lineToSplineHeading(new Pose2d(34, 42, Math.toRadians(90)))

                // Action - Raise Arm
                .addTemporalMarker(() -> {

                    // Raise Arm
                    sysIntakeArm.moveIntakeToTarget(utilRobotConstants.IntakeArm.INTAKE_ENCODER_SETPOINT_AUTOPIXEL, utilRobotConstants.IntakeArm.INTAKE_MOTOR_OUTPUT_POWER_MAX);
                    sysIntakeArm.moveArmToTarget(utilRobotConstants.IntakeArm.ARM_ENCODER_SETPOINT_AUTOPIXEL, utilRobotConstants.IntakeArm.ARM_MOTOR_OUTPUT_POWER_MAX);

                })
                .waitSeconds(2)

                // Move a little closer to board
                .splineTo(
                        new Vector2d(34, 46), Math.toRadians(90),
                        sysDrivetrain.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        sysDrivetrain.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )

                // Place Yellow Pixel
                .addTemporalMarker(() -> {
                    sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_TWO, utilRobotConstants.IntakeArm.SERVO_SLOTTWO_SETPOINT_OPEN);
                })
                .waitSeconds(2)

                // Lower Arm
                .addTemporalMarker(() -> {
                    sysIntakeArm.moveIntakeToTarget(utilRobotConstants.IntakeArm.INTAKE_ENCODER_SETPOINT_HOME, utilRobotConstants.IntakeArm.INTAKE_MOTOR_OUTPUT_POWER_MIN);
                    sysIntakeArm.moveArmToTarget(utilRobotConstants.IntakeArm.ARM_ENCODER_SETPOINT_HOME, utilRobotConstants.IntakeArm.ARM_MOTOR_OUTPUT_POWER_MIN);
                })
                .waitSeconds(2)

                // Back Away from the board
                .lineTo(new Vector2d(34, 42))

                // Drive to park location
                .lineToSplineHeading(new Pose2d(64, 42, Math.toRadians(180)))

                // Move a little closer to park position
                .strafeTo(new Vector2d(64, 54))

                .build();


        // ----------------------------------------------------
        // Trajectory Sequence for - Red Alliance - Audience Side - Default (when detection goes wrong)
        // ----------------------------------------------------
        TrajectorySequence trajSeqRedAudienceZoneDefault = trajSeqRedAudienceZoneOne;


        // Wait for Start state (disable if using opModeInInit)
        waitForStart();

        if (isStopRequested()) return;

        // ----------------------------------------------------
        // Movement Action(s)
        // ----------------------------------------------------
        // Determine Zone Trajectory Sequence
        // ----------------------------------------------------

        // Determine Trajectory Sequence based on Random Zone
        // ----------------------------------------------------
        switch (targetZone){
            case 1:
                // Zone One
                // ----------------------

                // Set Trajectory Sequence based on Alliance
                if(sysVision.getDetectedAllianceTagColor() == "blue") {

                    // Blue - Audience - Zone One
                    sysDrivetrain.followTrajectorySequence(trajSeqBlueAudienceZoneOne);
                }
                else {

                    // Red - Audience - Zone One
                    sysDrivetrain.followTrajectorySequence(trajSeqRedAudienceZoneOne);
                }

                break;
            case 2:
                // Zone Two
                // ----------------------

                // Set Trajectory Sequence based on Alliance
                if(sysVision.getDetectedAllianceTagColor() == "blue") {

                    // Blue - Audience - Zone One
                    sysDrivetrain.followTrajectorySequence(trajSeqBlueAudienceZoneTwo);
                }
                else {

                    // Red - Audience - Zone One
                    sysDrivetrain.followTrajectorySequence(trajSeqRedAudienceZoneTwo);
                }

                break;
            case 3:
                // Zone Three
                // ----------------------

                // Set Trajectory Sequence based on Alliance
                if(sysVision.getDetectedAllianceTagColor() == "blue") {

                    // Blue - Audience - Zone One
                    sysDrivetrain.followTrajectorySequence(trajSeqBlueAudienceZoneThree);
                }
                else {

                    // Red - Audience - Zone One
                    sysDrivetrain.followTrajectorySequence(trajSeqRedAudienceZoneThree);
                }

                break;
            default:
                // Default
                // ----------------------

                // Set Trajectory Sequence based on Alliance
                if(sysVision.getDetectedAllianceTagColor() == "blue") {

                    // Blue - Audience - Zone One
                    sysDrivetrain.followTrajectorySequence(trajSeqBlueAudienceZoneDefault);
                }
                else {

                    // Red - Audience - Zone One
                    sysDrivetrain.followTrajectorySequence(trajSeqRedAudienceZoneDefault);
                }

                break;
        }

        // Wait for end of Autonomous
        // ----------------------
        while (!isStopRequested() && opModeIsActive());

    }


}
