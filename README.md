To run benchmarks:
$ mvn clean package
$ java --add-modules jdk.incubator.vector -jar target/benchmarks.jar

To see the produced assembly:
$ java --add-modules jdk.incubator.vector -jar target/benchmarks.jar -prof perfasm
