See https://en.algorithmica.org/hpc/algorithms/prefix/ for a good reference on computing prefix sums using SIMD. SIMD is supposed to make prefix sums multiple times faster, but this isn't the case currently with Java's Panama vector API.

To run benchmarks:

$ mvn clean package

$ java --add-modules jdk.incubator.vector -jar target/benchmarks.jar

To see the produced assembly:

$ java --add-modules jdk.incubator.vector -jar target/benchmarks.jar -prof perfasm
