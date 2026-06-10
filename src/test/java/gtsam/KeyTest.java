package gtsam;

import org.junit.jupiter.api.Test;

public class KeyTest {
    @Test
    void testPrint() throws Throwable {
        Key x1 = Key.X(1);
        x1.print();
        Key b2 = Key.B(20000);
        b2.print();
    }
}
