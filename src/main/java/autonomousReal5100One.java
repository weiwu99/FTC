/**
 * Created by dwu on 10/24/17.
 */
        import android.app.Activity;
        import android.graphics.Color;
        import android.view.View;

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
        import org.firstinspires.ftc.robotcore.external.Func;
        import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
        import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
        import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
        import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
        import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
        import org.firstinspires.ftc.robotcore.external.navigation.Position;
        import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
        import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
        import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
        import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
        import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
        import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

        import java.util.Locale;

/**
 * Created by pliu on 10/23/17.
 */

@Autonomous(name = "autonomous5100ONE", group = "5100")
@Disabled
public class autonomousReal5100One extends LinearOpMode {
    private DcMotor LeftFront, RightFront, LeftBack, RightBack;
    private DcMotor Lifting;

    private Servo leftservo, rightservo;
    private Servo bottom, top;

    private ColorSensor sensorColor;
    private DistanceSensor sensorDistance;

    private double initHeading, heading;

    private BNO055IMU imu;

    private Orientation angles;
    private Acceleration gravity;

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

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

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
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity  = imu.getGravity();
        composeTelemetry();

        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        relicTrackables.activate();
        runtime.reset();

        //Start
        while (opModeIsActive()) {

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                telemetry.addData("VuMark", "%s visible", vuMark);
                telemetry.addData("Test: ", vuMark.toString());

                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();

                telemetry.addData("Pose", format(pose));

                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }
            } else {
                telemetry.addData("VuMark", "not visible");
            }

            telemetry.update();

            if (vuMark.toString() == "LEFT"){
                key = 0;
                break;
            }
            else if (vuMark.toString() == "CENTER"){
                key = 1;
                break;
            }
            else if (vuMark.toString() == "RIGHT") {
                key = 2;
                break;
            }

            if(runtime.seconds() > 5.0) break;
        }

        if(opModeIsActive()){
            //grab the glyph
            leftservo.setPosition(1);
            rightservo.setPosition(0);

            //color stick
            top.setPosition(1);
            sleep(380);
            top.setPosition(0.5);
            sleep(500);

            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);
            while(opModeIsActive()) {
                sleep(1500);
                if ((sensorColor.red() < sensorColor.blue()) || Math.abs((sensorColor.red() - sensorColor.blue())) <= 5) {
                    isRed = false;
                    telemetry.addData("Color: ", "Blue");
                    break;
                } else if (sensorColor.red() - sensorColor.blue() > 5) {
                    isRed = true;
                    telemetry.addData("Color: ", "Red");
                    break;
                }
            }

            telemetry.update();

            if (isRed == true) {

                bottom.setPosition(0);
                sleep(300);
                bottom.setPosition(0.5);
                sleep(100);
                bottom.setPosition(1);
                sleep(300);
                bottom.setPosition(0.5);
            } else {
                bottom.setPosition(1);
                sleep(300);
                bottom.setPosition(0.5);
                sleep(100);
                bottom.setPosition(0);
                sleep(300);
                bottom.setPosition(0.5);
            }

            top.setPosition(0);
            sleep(2000);
            top.setPosition(0.5);

            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });

            //Motion
            initHeading = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));

            composeTelemetry();
            telemetry.update();

            runMeta(1);

        }
//
    }



    public void initialize(){

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
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensorcd");
    }

    void turnCheck(double degree){
        if(degree > 0) {
            while (Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)) - initHeading < degree) {
                LeftFront.setPower(0.4);
                LeftBack.setPower(0.4);
                RightFront.setPower(0.4);
                RightBack.setPower(0.4);
                sleep(200);
                LeftFront.setPower(0);
                LeftBack.setPower(0);
                RightFront.setPower(0);
                RightBack.setPower(0);
                composeTelemetry();
                telemetry.update();
                sleep(100);
            }
        }
        else if (degree < 0) {
            while (Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)) - initHeading > degree) {
                LeftFront.setPower(-0.4);
                LeftBack.setPower(-0.4);
                RightFront.setPower(-0.4);
                RightBack.setPower(-0.4);
                sleep(200);
                LeftFront.setPower(0);
                LeftBack.setPower(0);
                RightFront.setPower(0);
                RightBack.setPower(0);
                composeTelemetry();
                telemetry.update();
                sleep(100);
            }
        }

        initHeading += degree;

        if(degree > 0) {
            while (Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)) - initHeading > 0) {
                LeftFront.setPower(-0.25);
                LeftBack.setPower(-0.25);
                RightFront.setPower(-0.25);
                RightBack.setPower(-0.25);
                sleep(100);
                LeftFront.setPower(0);
                LeftBack.setPower(0);
                RightFront.setPower(0);
                RightBack.setPower(0);
                composeTelemetry();
                telemetry.update();
                sleep(100);
            }
        }
        else if (degree < 0) {
            while (Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)) - initHeading < 0) {
                LeftFront.setPower(0.25);
                LeftBack.setPower(0.25);
                RightFront.setPower(0.25);
                RightBack.setPower(0.25);
                sleep(100);
                LeftFront.setPower(0);
                LeftBack.setPower(0);
                RightFront.setPower(0);
                RightBack.setPower(0);
                composeTelemetry();
                telemetry.update();
                sleep(100);
            }
        }
    }

    void composeTelemetry() {

        telemetry.addAction(new Runnable() { @Override public void run()
        {
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        heading = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {

                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
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

    void moveBackward(double pwr, int time){
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

    public void runMeta (int posi) {
        switch (posi) {
            case 1: //The position where we usually practice is counted as 1. Count Clockwise, so 3 and 4 are blue.
                moveForward(0.5, 700);

                if(key == 0)
                    turnCheck(50);
                else if (key == 1)
                    turnCheck(40);
                else if(key == 2)
                    turnCheck(25);
                else
                    turnCheck(40);

                moveForward(0.4, 700);
                leftservo.setPosition(0.75);
                rightservo.setPosition(0.25);
                sleep(300);
                moveBackward(0.3, 400);
                break;
            case 2:
                moveForward(0.5,1000);

                if(key == 0)
                    turnCheck(-100);
                else if(key == 1)
                    turnCheck(-120);
                else if(key == 2)
                    turnCheck(-130);
                else
                    turnCheck(-120);

                moveForward(0.4,700);
                leftservo.setPosition(0.75);
                rightservo.setPosition(0.25);
                sleep(300);
                moveBackward(0.4, 300);
                break;
            case 3:
                Lifting.setPower(0.4);
                sleep(800);
                Lifting.setPower(0);

                moveBackward(0.5,1000);

                if(key == 0)
                    turnCheck(-50);
                else if(key == 1)
                    turnCheck(-60);
                else if(key == 2)
                    turnCheck(-80);
                else
                    turnCheck(-60);

                moveForward(0.4,700);
                Lifting.setPower(-0.3);
                sleep(500);
                Lifting.setPower(0);
                leftservo.setPosition(0.75);
                rightservo.setPosition(0.25);
                sleep(300);
                moveBackward(0.4, 300);
                break;
            case 4:
                Lifting.setPower(0.4);
                sleep(800);
                Lifting.setPower(0);
                moveBackward(0.5,700);

                if(key == 0)
                    turnCheck(155);
                else if(key == 1)
                    turnCheck(140);
                else if(key == 2)
                    turnCheck(130);
                else
                    turnCheck(140);

                moveForward(0.4,700);
                Lifting.setPower(-0.3);
                sleep(500);
                Lifting.setPower(0);
                leftservo.setPosition(0.75);
                rightservo.setPosition(0.25);
                sleep(300);
                moveBackward(0.4, 300);
                break;
        }
    }
}