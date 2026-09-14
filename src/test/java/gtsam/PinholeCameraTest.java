package gtsam;

import static gtsam.Testable.assert_equal;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

public class PinholeCameraTest {
    @Test
    void testConstruction() throws Throwable {
        Pose3 pose = new Pose3();
        Cal3DS2 cal = new Cal3DS2(100, 100, 0, 50, 50, 0, 0);
        PinholeCamera<Cal3DS2> cam = PinholeCamera.PinholeCameraCal3DS2(pose, cal);
        Point3 world = new Point3(0, 0, 1);
        Point2 px = cam.project(world);
        assertTrue(assert_equal(new Point2(50, 50), px, 1e-5));
    }
}
