package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import static dev.nextftc.bindings.Bindings.button;

import android.util.Size;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.hoodSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.intakeSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.multiFunctionSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.outtakeSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.transferSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.turretSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Scalar;

import java.util.List;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name="Red Far Non Ideal", group="Red ILT")
public class ILTRedFarNonIdeal extends NextFTCOpMode {

    // conditions + hardware

    double velocityDiff = 0;

    boolean autoShoot = false;
    MotorEx turret = new MotorEx("turret").zeroed().reversed();
    private ControlSystem controller;

    MotorEx frontLeftMotor;
    MotorEx frontRightMotor;
    MotorEx backLeftMotor;
    MotorEx backRightMotor;

    Limelight3A limelight;
    IMU imu;
    Follower clanka;

    Rev2mDistanceSensor proxySens;
    RevColorSensorV3 colorSens;
    Servo backlight;
    boolean hasArtifact;

    boolean shooting = false;

    // turret vars
    boolean turretTracking = true;

    double fx = 626.909;
    double fy = 626.909;

    double cx = 426.007;
    double cy = 236.834;

    double robotYaw = 0;

    double globalCameraYaw = 0;

    Boolean blobDetected;

    //define Color Ranges
    ColorRange greenRange = new ColorRange(
            ColorSpace.YCrCb,
            new Scalar(15, 0, 0),     // Min (Y, Cr, Cb)
            new Scalar(255, 127, 170)   // Max (Y, Cr, Cb)
    );

    //custom PURPLE Range (YCrCb), these values are NOT final
    //purple is generally high in Red (Cr) and high in Blue (Cb)
    ColorRange purpleRange = new ColorRange(
            ColorSpace.YCrCb,
            new Scalar(32, 130, 140),  // Min (Y, Cr, Cb)
            new Scalar(255, 170, 200)   // Max (Y, Cr, Cb)
    );

    double clankerX, clankerY, clankerR, heading;
    boolean running;
    double targetVel;
    double targetHood = 0;
    double yaw;
    double distance;
    double delay;
    double blueX = 0;
    double blueY = 144;
    double offset = 8.8;
    double goalX = 144-3.1;
    double goalY = 144;
    double visionX = 0;
    double visionY = 0;
    double theta;
    double ticks;
    double ticksToDegrees = (130.0 / 36.0) * (384.5 / 360.0);
    double robotHeadingDegrees = 0;
    double leftMax = 691;
    double rightMax = -809;
    double xDiff = 0;
    double yDiff = 0;
    double limelightX = 0;
    double limelightY = 0;
    double angleOffset = 260;
    boolean limelightTracking = false;
    boolean hasCorrectedLL = false;
    double finalTargetTicks;
    boolean far = false;
    boolean readyShoot = false;

    double robotVelocityMag = 0;
    double robotVelocityXComp = 0;
    double robotVelocityYComp = 0;

    CRServoEx liftL;
    CRServoEx liftR;

    // visionmaxxing init
    ColorBlobLocatorProcessor purpleProcessor;
    ColorBlobLocatorProcessor greenProcessor;
    VisionPortal portal;

    public ILTRedFarNonIdeal() {
        addComponents(
                BindingsComponent.INSTANCE,
                new SubsystemComponent(
                        multiFunctionSubsystem.INSTANCE,
                        outtakeSubsystem.INSTANCE,
                        intakeSubsystem.INSTANCE,
                        transferSubsystem.INSTANCE,
                        hoodSubsystem.INSTANCE,
                        turretSubsystem.INSTANCE // used for kinematic turret logic
                ),
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    // velo/hood calcs

//    public double getDistance(double ta) {
//        return 75.16142 * Math.pow(ta, -0.47932);
//    }
//    public double getHoodAngle(double dist) {
//        return 0.00578881 * dist - 0.00802588;
//    }
//
//    public double getVelocity(double dist) {
//        return 10.99866 * dist + 1084.95409;
//    }

    //large triangle curves
    public double getHood(double distance) {
        return (distance >= 148.01468) ? -0.0210202 * distance + 3.48553 : 71.71201 * Math.pow(distance, -1.17652);
    }

    public double getVel(double distance) {
        return (distance >= 148.01468) ? 19.17589 * distance - 582.51652 : 7.65268 * distance + 1123.06996;
    }



    // turret calcs
    public double normalize(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    public double atan2(double y, double x) {
        return Math.atan2(y, x);
    }

    public double calcDigger() {
        return atan2(goalY - (7 + clankerY), goalX - clankerX);
    }

    public double H2(double digger) {
        return normalize(digger - heading) * 180 / Math.PI;
    }

    public double tickAdjustment(double digger) {
        double calcTicks = H2(digger) * (130.0 / 36.0) * (384.5 / 360.0);
        double ticksPerRev = 384.5 * (130.0 / 36.0);

        if (leftMax != 0 && calcTicks > leftMax) calcTicks -= ticksPerRev;
        else if (rightMax != 0 && calcTicks < rightMax) calcTicks += ticksPerRev;

        return calcTicks;
    }

    @Override
    public void onInit() {
        liftL = new CRServoEx("liftLeft");
        liftR = new CRServoEx("liftRight");

        imu = hardwareMap.get(IMU.class, "imu");

        proxySens = hardwareMap.get(Rev2mDistanceSensor.class, "proxySensor");
        colorSens = hardwareMap.get(RevColorSensorV3.class, "color sensor");
        backlight = hardwareMap.get(Servo.class, "backlight");

        RevHubOrientationOnRobot revOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );

        imu.initialize(new IMU.Parameters(revOrientation));
        imu.resetYaw();

        hoodSubsystem.INSTANCE.goon(1);
        purpleProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(purpleRange)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(false) //set to false for comp for faster processing
                .setBlurSize(6)
                .build();

        greenProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(greenRange)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(false)
                .setBlurSize(6)
                .build();

        portal = new VisionPortal.Builder()
                .addProcessor(purpleProcessor)
                .addProcessor(greenProcessor)
                .setCameraResolution(new Size(864, 480)) // Your specified resolution
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCamera(hardwareMap.get(WebcamName.class, "camera"))
                .build();

        frontLeftMotor = new MotorEx("leftFront");
        frontRightMotor = new MotorEx("rightFront");
        backLeftMotor = new MotorEx("leftRear");
        backRightMotor = new MotorEx("rightRear");

        frontRightMotor.setDirection(-1);
        backRightMotor.setDirection(-1);
        frontLeftMotor.setDirection(-1);
        backLeftMotor.setDirection(-1);

        clanka = PedroComponent.follower();
        clanka.setStartingPose(new Pose(72, 72+18, 90).mirror());

        transferSubsystem.INSTANCE.toNeutral.schedule();

        turret.setCurrentPosition(0);
        turret.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        controller = ControlSystem.builder()
                .posPid(0.01, 0.0, 0.0001)
                .basicFF(0, 0, 0.0025)
                .build();
        controller.setGoal(new KineticState(0.0));

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        telemetry.setMsTransmissionInterval(1);

        targetVel = 0;
    }

    @Override
    public void onStartButtonPressed() {
        limelight.start();

        button(() -> gamepad1.y).whenBecomesTrue(multiFunctionSubsystem.INSTANCE.transpherSequencNiga());

    }

    @Override
    public void onUpdate() {
        clanka.update();
        BindingManager.update();

        blobDetected = checkForBlobs(portal, purpleProcessor, greenProcessor);
        // robot pose(X, Y, R)
        heading = clanka.getHeading();
        clankerX = clanka.getPose().getX();
        clankerY = clanka.getPose().getY();

        robotVelocityMag = clanka.getVelocity().getMagnitude();
        robotVelocityXComp = clanka.getVelocity().getXComponent();
        robotVelocityYComp = clanka.getVelocity().getMagnitude();

        readyShoot = Math.abs(robotVelocityMag) < 7.5;

        // ll digga
        LLResult result = limelight.getLatestResult();
        boolean tagFound = (result != null && result.isValid());


        distance = Math.sqrt(Math.pow((blueX - clankerX), 2) + Math.pow((blueY - clankerY), 2)) + offset;
        velocityDiff = Math.abs(outtakeSubsystem.INSTANCE.getJawn() - getVel(distance));

        if(readyShoot && insideShootingTriangle()) {
            if (!hasCorrectedLL) {
                if (tagFound) {
                    yaw = result.getTx();
                    double relativeTickOffset = yaw * ticksToDegrees;
                    finalTargetTicks = turret.getCurrentPosition() + relativeTickOffset;
                    hasCorrectedLL = true;
                }
                else {
                    theta = H2(calcDigger());
                    ticks = tickAdjustment(calcDigger());
                    finalTargetTicks = -ticks;
                }
            }
        }
        else {
            theta = H2(calcDigger());
            ticks = tickAdjustment(calcDigger());
            finalTargetTicks = -ticks;
            hasCorrectedLL = false;
        }

        // subsystem automation velo+hood
        if (gamepad1.right_trigger > 0.1) {
            outtakeSubsystem.INSTANCE.setVel(getVel(distance)).schedule();
        }
        else if(robotVelocityMag > 5){
            outtakeSubsystem.INSTANCE.off().schedule();
        }
        else {
            outtakeSubsystem.INSTANCE.setVel(0).schedule();
        }

        if (gamepad1.a) {
            turretTracking = !turretTracking;
        }

        if (turretTracking) {
            controller.setGoal(new KineticState(finalTargetTicks));
        }
        else {
            //on opposite toggle, set pos to 0
            controller.setGoal(new KineticState(0));
        }

        KineticState currentState = new KineticState(turret.getCurrentPosition(), turret.getVelocity());
        double turretPower = controller.calculate(currentState);

        hoodSubsystem.INSTANCE.goon(getHood(distance)).schedule();

        if (!turretTracking && Math.abs(turret.getCurrentPosition()) < 5) {
            turret.setPower(0);
        } else {
            turret.setPower(turretPower);
        }

        //auto shoot algorithm
        //first, check if turret has corrected already(wraps turret error, robot velocity)
//        if(hasCorrectedLL){
//            autoShoot = true;
//            if(hasArtifact && velocityDiff < 20) {
//                intakeSubsystem.INSTANCE.eat.schedule();
//                multiFunctionSubsystem.INSTANCE.transpherSequencNiga().schedule();
//            }
//            else {
//                intakeSubsystem.INSTANCE.sleep.schedule();
//            }
//        }

        // intake digger
        if (gamepad1.left_bumper) {
            intakeSubsystem.INSTANCE.slowSpit.schedule();
        } else if (gamepad1.right_bumper) {
            intakeSubsystem.INSTANCE.eat.schedule();
        }
//        else if (blobDetected != null) {
//            if (blobDetected) {
//                intakeSubsystem.INSTANCE.eat.schedule();
//            }
//            else {
//                intakeSubsystem.INSTANCE.sleep.schedule();
//            }
//        }
        else {
            intakeSubsystem.INSTANCE.sleep.schedule();
        }


        if (gamepad1.dpad_up || gamepad2.dpad_up) {
            liftL.setPower(1);
            liftR.setPower(-1);
        } else if (gamepad1.dpad_down || gamepad2.dpad_down) {
            liftL.setPower(-1);
            liftR.setPower(1);
        } else {
            liftL.setPower(0);
            liftR.setPower(0);
        }

        if (gamepad1.dpad_up) {
            limelightTracking = !limelightTracking;
        }
        if (gamepad1.dpad_left) {
            clanka.setPose(new Pose(7.75, 7.5, Math.toRadians(90)));
        }
        if (gamepad1.dpad_right) {
            clanka.setPose(new Pose(144 - 7.75, 7.5, Math.toRadians(90)));
        }

        if ((proxySens.getDistance(DistanceUnit.INCH) < 5)) {
            hasArtifact = true;
            backlight.setPosition(0.333);
            if (insideShootingTriangle()){
                backlight.setPosition(0.388);
                if (hasCorrectedLL){
                    backlight.setPosition(0.444);
                    if (velocityDiff < 20) {
                        backlight.setPosition(0.555);
                    }
                }
            }
        }
        else {
            hasArtifact = false;
            backlight.setPosition(0.28);
        }

        double y = gamepad1.left_stick_y;
        double rx = -gamepad1.right_stick_x * 1.1;
        double x = -gamepad1.left_stick_x;

        double y2 = gamepad2.left_stick_y;
        double rx2 = -gamepad2.right_stick_x * 1.1;
        double x2 = -gamepad2.left_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        double denominator2 = Math.max(Math.abs(y2) + Math.abs(x2) + Math.abs(rx2), 1);
        double frontLeftPower2 = (y2 + x2 + rx2) / denominator2;
        double backLeftPower2 = (y2 - x2 + rx2) / denominator2;
        double frontRightPower2 = (y2 - x2 - rx2) / denominator2;
        double backRightPower2 = (y2 + x2 - rx2) / denominator2;

        if (gamepad1.left_stick_y >= 0.1 || gamepad1.left_stick_x >= 0.1 || gamepad1.left_stick_y <= -0.1 || gamepad1.left_stick_x <= -0.1 || gamepad1.right_stick_y >= 0.2 || gamepad1.right_stick_x >= 0.2 || gamepad1.right_stick_y <= -0.2 || gamepad1.right_stick_x <= -0.2) {
            frontLeftMotor.setPower(frontLeftPower * 1);
            backLeftMotor.setPower(backLeftPower * 1);
            frontRightMotor.setPower(frontRightPower * 1);
            backRightMotor.setPower(backRightPower * 1);
        } else if (gamepad2.left_stick_y >= 0.1 || gamepad2.left_stick_x >= 0.1 || gamepad2.left_stick_y <= -0.1 || gamepad2.left_stick_x <= -0.1 || gamepad2.right_stick_y >= 0.1 || gamepad2.right_stick_x >= 0.1 || gamepad2.right_stick_y <= -0.1 || gamepad2.right_stick_x <= -0.1) {
            frontLeftMotor.setPower(frontLeftPower2 * 0.3);
            backLeftMotor.setPower(backLeftPower2 * 0.3);
            frontRightMotor.setPower(frontRightPower2 * 0.3);
            backRightMotor.setPower(backRightPower2 * 0.3);
        } else {
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);
        }

        telemetry.addData("Distance", distance);
        telemetry.addData("Tag Found", tagFound);
        telemetry.addData("Turret Pos", turret.getCurrentPosition());
        telemetry.addData("Degree Offset (Limelight)", yaw);
        telemetry.addData("Degree Offset (Kinematics)", theta);
        telemetry.addData("Target Ticks", -ticks);
        telemetry.addData("Blob Detected", blobDetected);
        telemetry.addData("Distance", distance);
        telemetry.addData("Ideal Velocity", getVel(distance));
        telemetry.addData("Ideal Hood", getHood(distance));
        telemetry.addData("Actual Hood", hoodSubsystem.INSTANCE.getDaddy());
        telemetry.addData("Velocity Calc", 10.81866 * distance + 1084.95409);
        telemetry.addData("Limelight Corrected", hasCorrectedLL);
        telemetry.addData("Robot Velocity Total Magnitude", robotVelocityMag);


        // List<ColorBlobLocatorProcessor.Blob> purpleBlobs = purpleProcessor.getBlobs();
        // List<ColorBlobLocatorProcessor.Blob> greenBlobs = greenProcessor.getBlobs();

        // Apply filter just for display count (logic is handled in function below)
        // ColorBlobLocatorProcessor.Util.filterByArea(30, 20000, purpleBlobs);
        // ColorBlobLocatorProcessor.Util.filterByArea(30, 20000, greenBlobs);

        // telemetry.addData("Purple Count", purpleBlobs.size());
        // telemetry.addData("Green Count", greenBlobs.size());
        // telemetry.addData("Balls Found:", blobDetected);
        telemetry.update();
    }

    public Boolean checkForBlobs(VisionPortal portal,
                                 ColorBlobLocatorProcessor purpleProc,
                                 ColorBlobLocatorProcessor greenProc) {

        // 1. Verify Camera is actually STREAMING.
        // If it's OPENING, STARTING_STREAM, or ERROR, we cannot claim "False" yet.
        if (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return null;
        }

        // 2. Get Blobs
        List<ColorBlobLocatorProcessor.Blob> purpleBlobs = purpleProc.getBlobs();
        List<ColorBlobLocatorProcessor.Blob> greenBlobs = greenProc.getBlobs();

        // 3. Filter out noise (Blobs smaller than 50 pixels area)
        ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, purpleBlobs);
        ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, greenBlobs);

        // 4. Check counts
        boolean seePurple = purpleBlobs.size() > 1;
        boolean seeGreen = greenBlobs.size() > 3;

        // 5. Return logic
        return seePurple ||seeGreen;
    }
//        public YawPitchRollAngles getIMU(AngleUnit angleUnit){
//            return imu.getRobotYawPitchRollAngles();
//        }

    public boolean insideShootingTriangle() {
        if (clankerY > 72) {
            return (clankerX < 72) ? clankerY > -clankerX + 144 : clankerY > clankerX;
        }
        else {
            return (clankerX < 72) ? clankerY + 60 < clankerX : clankerY - 84 < -clankerX;
        }
    }
}