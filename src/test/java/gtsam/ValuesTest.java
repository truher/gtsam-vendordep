package gtsam;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/**
 * This is just one of the many tests in gtsam/nonlinear/tests/testValues.cpp,
 * because most of them aren't directly relevant.
 * 
 * This is just the one case that broke, and actually only part of that one.
 */
public class ValuesTest {

    @Test
    void testbasic_functions() throws Throwable {
        Values values = new Values();
        values.insert(new Key(2), new Point3(0, 0, 0));
        values.insert(Key.X(4), new Point3(0, 0, 0));

        assertFalse(values.exists(new Key(1)));
        assertTrue(values.exists(new Key(2)));
        assertTrue(values.exists(Key.X(4)));
        assertFalse(values.exists(Key.Y(6)));
    }

}
