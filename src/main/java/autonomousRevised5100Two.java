

/**
 * Created by Wei Wu on 11/28/17.
 */

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import android.graphics.Color;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

import java.util.Locale;

@Autonomous(name = "autonomous5100Two", group = "5100")

public class autonomousRevised5100Two extends LinearOpMode {
    private DcMotor LeftFront, RightFront, LeftBack, RightBack;
    private DcMotor Lifting;

    private Servo leftservo, rightservo;
    private Servo bottom, top;

    private ColorSensor sensorColor;

    private double initHeading;

    private BNO055IMU imu; // gyro

    private Orientation angles;

    // Vuforia Code

    private boolean isRed = false;

    private int key = 4;

    public static final String TAG = "Vuforia VuMark Sample";

    static final String LICENSE = "AcizW6z/////AAAAGUZPTu7RHE/mj7BX78dIYvhHSBQgkIRk8DbyqqV39fgEgeCyLk/inLOzcYFxgjw" +
            "EJ38p0pLEtfKpvNYHGnE8e/sXPl8HKPyiCxTaUeMEuxW//EmlG5KTcGDODGcpO55Kvsn0tARBkSC" +
            "YPUqTmpMMszDP9xgfcRUj/EjX+3eqqLgrLs9DylSDdtdmp6pgInu+oZdUHZuWyvIxl+w9PVxj9cdkBZ" +
            "XRBygWFtD8sduMGSn+mPK6Bm+Dw+PVnLQbDyPyP9M2FFO9NC1h76ADa4E8JmRhBSmottVfzlFpx4/laBYLqAyAha" +
            "TjW+CLmSOItA3e/rJieIleGfmBLNOsp40TJJt4Y7Gz2qpn64qpDH+nQ+xb";

    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        initialize();

        //Color sensor
        float hsvValues[] = {0F, 0F, 0F};

        final float values[] = hsvValues;

        final double SCALE_FACTOR = 255;

        //Vuforia
        int cameraMonitorViewId =
                hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameterV = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameterV.vuforiaLicenseKey = LICENSE;

        parameterV.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameterV);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        //Gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        composeTelemetry();

        // End of gyro

        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        relicTrackables.activate();
        runtime.reset();

//      Start
        while (opModeIsActive()) {

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                telemetry.addData("VuMark", "%s visible", vuMark);
//                telemetry.addData("Test: ", vuMark.toString());

                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();

                telemetry.addData("Pose", format(pose));

//                if (pose != null) {
//                    VectorF trans = pose.getTranslation();
//                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
//                }
            } else
                telemetry.addData("VuMark", "not visible");

            telemetry.update();

            if (vuMark.toString() == "LEFT"){ key = 0; break;}

            else if (vuMark.toString() == "CENTER"){ key = 1; break;}

            else if (vuMark.toString() == "RIGHT") { key = 2; break;}

            if(runtime.seconds() > 3.0) break;
        }

        if(opModeIsActive()){

            //grab the glyph
            leftservo.setPosition(1);
            rightservo.setPosition(0);
            sleep(300);

            top.setPosition(0.95);
            sleep(300);

            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);
            sleep(1000);

            while(opModeIsActive()) {

                double red = sensorColor.red();
                double blue = sensorColor.blue();

                if ( red < blue || Math.abs(red - blue) <= 5) { isRed = false; break;}

                else if (red - blue > 5) { isRed = true; break;}
            }

            colorR(true);

            top.setPosition(0);
            sleep(500);

            //Motion
            initHeading = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
            composeTelemetry();
            telemetry.update();

            runMeta(2);

        }
    }



    void initialize(){

        LeftFront = hardwareMap.dcMotor.get("LF");
        RightFront = hardwareMap.dcMotor.get("RF");
        LeftBack = hardwareMap.dcMotor.get("LB");
        RightBack = hardwareMap.dcMotor.get("RB");

        Lifting = hardwareMap.dcMotor.get("LS");

        leftservo = hardwareMap.servo.get("servo1");
        rightservo = hardwareMap.servo.get("servo2");

        bottom = hardwareMap.servo.get("servo3");
        top = hardwareMap.servo.get("servo4");

        sensorColor = hardwareMap.get(ColorSensor.class, "sensorcd");
    }

    void colorR(boolean red){
        if(red == true){
            if (isRed == true) {
                bottom.setPosition(0.6);
                sleep(300);
                bottom.setPosition(0.75);
                sleep(200);

            } else {
                bottom.setPosition(1);
                sleep(300);
                bottom.setPosition(0.85);
                sleep(200);
            }
        } else {
            if (isRed == false) {
                bottom.setPosition(0.6);
                sleep(300);
                bottom.setPosition(0.75);
                sleep(200);

            } else {
                bottom.setPosition(1);
                sleep(300);
                bottom.setPosition(0.85);
                sleep(200);
            }
        }
    }

    void turnCheck(double degree){
        double temp = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
        double sub = temp;
        if(degree > 0) {
            while (Math.abs(sub - initHeading - degree) > 15) {
                LeftFront.setPower(-0.5);
                LeftBack.setPower(-0.5);
                RightFront.setPower(-0.5);
                RightBack.setPower(-0.5);
                sleep(200);
                LeftFront.setPower(0);
                LeftBack.setPower(0);
                RightFront.setPower(0);
                RightBack.setPower(0);
                composeTelemetry();
                telemetry.update();
                sleep(100);
                temp = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
                sub = temp;
                if(temp < 0)
                    sub = 360 + temp;
            }
        }
        else if (degree < 0) {
            while (Math.abs(sub - initHeading - degree) > 15) {
                LeftFront.setPower(0.5);
                LeftBack.setPower(0.5);
                RightFront.setPower(0.5);
                RightBack.setPower(0.5);
                sleep(200);
                LeftFront.setPower(0);
                LeftBack.setPower(0);
                RightFront.setPower(0);
                RightBack.setPower(0);
                composeTelemetry();
                telemetry.update();
                sleep(100);
                temp = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
                sub = temp;
                if(temp > 0)
                    sub = temp - 360;
            }
        }

        initHeading += degree;

        if(sub - degree > 0) {
            while (Math.abs(sub - degree) > 3 && degree < sub) {
                LeftFront.setPower(0.3);
                LeftBack.setPower(0.3);
                RightFront.setPower(0.3);
                RightBack.setPower(0.3);
                sleep(100);
                LeftFront.setPower(0);
                LeftBack.setPower(0);
                RightFront.setPower(0);
                RightBack.setPower(0);
                composeTelemetry();
                telemetry.update();
                sleep(100);
                temp = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
                sub = temp;

                if(temp > 0 && degree < 0)
                    sub = temp - 360;
                if(temp < 0 && degree > 0)
                    sub = 360 + temp;
            }
        }
        else if (sub - degree < 0) {
            while (Math.abs(sub - degree) > 3 && degree > sub) {
                LeftFront.setPower(-0.3);
                LeftBack.setPower(-0.3);
                RightFront.setPower(-0.3);
                RightBack.setPower(-0.3);
                sleep(100);
                LeftFront.setPower(0);
                LeftBack.setPower(0);
                RightFront.setPower(0);
                RightBack.setPower(0);
                composeTelemetry();
                telemetry.update();
                sleep(100);
                temp = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
                sub = temp;
                if(temp > 0 && degree < 0)
                    sub = temp - 360;
                if(temp < 0 && degree > 0)
                    sub = 360 + temp;
            }
        }
    }

    void composeTelemetry() {

        telemetry.addAction(new Runnable() { @Override public void run()
        {
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//            gravity  = imu.getGravity();
        }
        });
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    void moveForward(double pwr, int time){
        LeftFront.setPower(pwr);
        LeftBack.setPower(pwr);
        RightFront.setPower(-pwr);
        RightBack.setPower(-pwr);
        sleep(time);
        LeftFront.setPower(0);
        LeftBack.setPower(0);
        RightFront.setPower(0);
        RightBack.setPower(0);
    }

    void moveBackward(double pwr, int time){
        LeftFront.setPower(-pwr);
        LeftBack.setPower(-pwr);
        RightFront.setPower(pwr);
        RightBack.setPower(pwr);
        sleep(time);
        LeftFront.setPower(0);
        LeftBack.setPower(0);
        RightFront.setPower(0);
        RightBack.setPower(0);
    }

    void moveLeft(double pwr, int time){
        LeftFront.setPower(-pwr);
        LeftBack.setPower(pwr);
        RightFront.setPower(-pwr);
        RightBack.setPower(pwr);
        sleep(time);
        LeftFront.setPower(0);
        LeftBack.setPower(0);
        RightFront.setPower(0);
        RightBack.setPower(0);
    }

    void moveRight(double pwr, int time){
        LeftFront.setPower(pwr);
        LeftBack.setPower(-pwr);
        RightFront.setPower(pwr);
        RightBack.setPower(-pwr);
        sleep(time);
        LeftFront.setPower(0);
        LeftBack.setPower(0);
        RightFront.setPower(0);
        RightBack.setPower(0);
    }

    public void runMeta (int posi) {
        switch (posi) {
            case 1:
                Lifting.setPower(-0.5);
                sleep(1000);
                Lifting.setPower(0);

                moveBackward(0.4, 800);


                if(key == 0) {

                    turnCheck(-125);
                    moveLeft(0.4,500);
                    moveForward(0.3, 400);
                }
                else if(key == 1) {
                    turnCheck(-140);
                    moveLeft(0.4,300);
                }
                else if(key == 2) {
                    turnCheck(-158);
                }
                else {
                    turnCheck(-140);
                    moveLeft(0.4,300);
                }

                moveForward(0.3, 300);
                Lifting.setPower(0.5);
                sleep(800);
                Lifting.setPower(0);
                sleep(200);

                moveForward(0.3, 500);

                leftservo.setPosition(0);
                rightservo.setPosition(1);
                sleep(500);
                moveForward(0.3,600);
                moveBackward(0.3, 200);
                if(key == 2)
                    moveBackward(0.3,200);
                break;

            case 2:
                Lifting.setPower(-0.5);
                sleep(1000);
                Lifting.setPower(0);

                if(key == 0) {
                    sleep(7000);
                    moveBackward(0.4,1500);
                    turnCheck(68);
                    moveLeft(0.4,300);
                }

                else if(key == 1) {
                    moveBackward(0.4, 1300);
                    turnCheck(70);
                }

                else if(key == 2) {
                    moveBackward(0.4,1100);
                    turnCheck(68);
                    moveRight(0.4,300);

                }

                else {
                    moveBackward(0.4, 1300);
                    turnCheck(70);
                }

                moveForward(0.3, 200);
                Lifting.setPower(0.5);
                sleep(800);
                Lifting.setPower(0);
                sleep(200);

                moveForward(0.3, 500);

                leftservo.setPosition(0.5);
                rightservo.setPosition(0.5);
                sleep(500);
                moveForward(0.3,600);
                moveBackward(0.3,200);
                break;

            case 3:
                Lifting.setPower(-0.5);
                sleep(1000);
                Lifting.setPower(0);

                if(key == 0) {
                    moveForward(0.4,1100);
                    turnCheck(112);
                    moveLeft(0.4,300);
                }

                else if(key == 1) {
                    moveForward(0.4, 1300);
                    turnCheck(110);
                }

                else if(key == 2) {
                    sleep(7000);
                    moveForward(0.4,1500);
                    turnCheck(112);
                    moveRight(0.4,300);
                }

                else {
                    moveForward(0.4, 1300);
                    turnCheck(110);
                }

                moveForward(0.3, 200);
                Lifting.setPower(0.5);
                sleep(800);
                Lifting.setPower(0);
                sleep(200);

                moveForward(0.3, 500);

                leftservo.setPosition(0.5);
                rightservo.setPosition(0.5);
                sleep(500);
                moveForward(0.3,600);
                moveBackward(0.3, 200);
                break;

            case 4:
                Lifting.setPower(-0.5);
                sleep(1000);
                Lifting.setPower(0);

                moveForward(0.4, 800);


                if(key == 0) {
                    turnCheck(-22);
                }
                else if(key == 1) {
                    turnCheck(-40);

                }
                else if(key == 2) {
                    turnCheck(-55);

                    moveForward(0.3, 400);
                }
                else {
                    turnCheck(-40);
                }

                moveForward(0.3, 300);
                Lifting.setPower(0.5);
                sleep(800);
                Lifting.setPower(0);
                sleep(200);

                moveForward(0.3, 500);

                leftservo.setPosition(0);
                rightservo.setPosition(1);
                sleep(500);
                moveForward(0.3,600);
                moveBackward(0.3, 200);
                if(key == 2)
                    moveBackward(0.3,200);
                break;
        }
    }
}
