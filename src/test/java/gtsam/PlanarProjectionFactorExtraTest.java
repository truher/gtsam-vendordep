package gtsam;

import org.junit.jupiter.api.Test;

import gtsam.noiseModel.Diagonal;

public class PlanarProjectionFactorExtraTest {
    // exactly like testPlanarProjectionFactor.h
    @Test
    void testCheiralityError() throws Throwable {
        Point3 landmark = new Point3(0, 0, 0);
        Point2 measured = new Point2(200, 200);
        Pose3 offset = new Pose3(
                new Rot3(//
                        0, 0, 1, //
                        -1, 0, 0, //
                        0, -1, 0),
                new Point3(0, 0, 0.5));
        Cal3DS2 calib = new Cal3DS2(200.0, 200.0, 0.0, 200.0, 200.0, 0.0, 0.0);
        shared_ptr<Diagonal> model = Diagonal.Sigmas(new Vector2(2, 2));
        shared_ptr<PlanarProjectionFactor1> factor = //
                PlanarProjectionFactor1.newPlanarProjectionFactor1(
                        Key.X(0), landmark, measured, offset, calib, model);
        Pose2 pose = new Pose2(1, 0, 0);
        Matrix H = new Matrix();
        Vector2 err = factor.get().evaluateError(pose, H);
        System.out.printf("err (%f %f)\n", err.at(0), err.at(1));
        H.print("H");
        // do the same composition as the predict() method
        Pose3 p3 = new Pose3(pose).compose(offset);
        Point3 landmarkInCameraFrame = p3.transformTo(landmark);
        System.out.printf("landmark In Camera Frame (%f %f %f)\n",
                landmarkInCameraFrame.x(),
                landmarkInCameraFrame.y(),
                landmarkInCameraFrame.z());
    }

    // cheirality exception troubleshooting.
    @Test
    void testCheirality() throws Throwable {
        long t_us = 0;
        // put the landmark right in the middle.
        Point3 landmark = new Point3(8, 4, 0.5);
        Point2 measurement = new Point2(400, 300);
        Pose3 camera_offset = new Pose3(
                new Rot3(//
                        0, 0, 1, //
                        -1, 0, 0, //
                        0, -1, 0),
                new Point3(0, 0, 0.5));
        Cal3DS2 calib = new Cal3DS2(200.0, 200.0, 0.0, 400.0, 300.0, -0.2, 0.1);
        shared_ptr<Diagonal> noise = Diagonal.Sigmas(new Vector2(2, 2));
        shared_ptr<PlanarProjectionFactor1> f = //
                PlanarProjectionFactor1.newPlanarProjectionFactor1(
                        Key.X(t_us),
                        landmark,
                        measurement,
                        camera_offset,
                        calib,
                        noise);
        {
            // camera facing the target
            Matrix H = new Matrix();
            // body pose
            Pose2 pose = new Pose2(4, 4, 0);
            Vector2 err = f.get().evaluateError(pose, H);
            System.out.printf("err (%f %f)\n", err.at(0), err.at(1));
            H.print("H");
            // do the same composition as the predict() method
            Pose3 p3 = new Pose3(pose).compose(camera_offset);
            Point3 landmarkInCameraFrame = p3.transformTo(landmark);
            System.out.printf("landmark In Camera Frame (%f %f %f)\n",
                    landmarkInCameraFrame.x(),
                    landmarkInCameraFrame.y(),
                    landmarkInCameraFrame.z());
        }
        {
            // same position, looking away
            Matrix H = new Matrix();
            Pose2 pose = new Pose2(4, 4, 0.1);
            Vector2 err = f.get().evaluateError(pose, H);
            System.out.printf("err (%f %f)\n", err.at(0), err.at(1));
            H.print("H");
            // do the same composition as the predict() method
            Pose3 p3 = new Pose3(pose).compose(camera_offset);
            Point3 landmarkInCameraFrame = p3.transformTo(landmark);
            System.out.printf("landmark In Camera Frame (%f %f %f)\n",
                    landmarkInCameraFrame.x(),
                    landmarkInCameraFrame.y(),
                    landmarkInCameraFrame.z());
        }
        {
            // what if the view is "behind"?
            Matrix H = new Matrix();
            Pose2 pose = new Pose2(12, 4, 0);
            Vector2 err = f.get().evaluateError(pose, H);
            System.out.printf("err (%f %f)\n", err.at(0), err.at(1));
            H.print("H");
            // do the same composition as the predict() method
            Pose3 p3 = new Pose3(pose).compose(camera_offset);
            Point3 landmarkInCameraFrame = p3.transformTo(landmark);
            System.out.printf("landmark In Camera Frame (%f %f %f)\n",
                    landmarkInCameraFrame.x(),
                    landmarkInCameraFrame.y(),
                    landmarkInCameraFrame.z());
        }
    }

}
