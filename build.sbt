import sbt.Resolver

organization  := "ch.unibas.cs.gravis"

name := "template-registration-with-scala"

version       := "0.1"

scalaVersion  := "2.12.8"

scalacOptions := Seq("-unchecked", "-deprecation", "-encoding", "utf8")

libraryDependencies  ++= Seq(
  "ch.unibas.cs.gravis" % "scalismo-native-all" % "4.0.+",
  "ch.unibas.cs.gravis" %% "scalismo-ui" % "0.90.0",
  "io.github.cibotech" %% "evilplot" % "0.8.1"
)

assemblyJarName in assembly := "scala-fun.jar"

mainClass in assembly := Some("example.ExampleApp")

assemblyMergeStrategy in assembly :=  {
    case PathList("META-INF", "MANIFEST.MF") => MergeStrategy.discard
    case PathList("META-INF", s) if s.endsWith(".SF") || s.endsWith(".DSA") || s.endsWith(".RSA") => MergeStrategy.discard
    case "reference.conf" => MergeStrategy.concat
    case _ => MergeStrategy.first
}
