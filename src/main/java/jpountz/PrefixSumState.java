package jpountz;

import org.openjdk.jmh.annotations.Level;
import org.openjdk.jmh.annotations.Scope;
import org.openjdk.jmh.annotations.Setup;
import org.openjdk.jmh.annotations.State;

@State(Scope.Benchmark)
public class PrefixSumState {

  static final int DEFAULT_ARRAY_LENGTH = 128;

  final int[] input ;
  final int[] output;

  public PrefixSumState() {
    this(DEFAULT_ARRAY_LENGTH);
  }

  public PrefixSumState(int size) {
    input = new int[size];
    output = new int[size];
  }

  @Setup(Level.Trial)
  public void setup() {
    for (int i = 0; i < input.length; ++i) {
      input[i] = (i+1) & 0x0F;
    }
  }

}
