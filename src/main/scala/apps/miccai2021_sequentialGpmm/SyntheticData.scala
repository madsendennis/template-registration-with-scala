package apps.miccai2021_sequentialGpmm

import java.io.File

object SyntheticData {

  val syntheticDir = new File(DataDirectory.dir, "synthetic")
  val modelFile: File = new File(syntheticDir, "augmentedModel100.h5")

  val inputLocation = new File(syntheticDir, "mesh")

  val groundTruthMaps = new File(syntheticDir, "gtMaps")

  val gtLabelMaps = groundTruthMaps.listFiles()
    .filter(_.getName.contains(".stl-onGT"))
    .sorted

  private val rawnames = Seq("correctedField-removed205-", "correctedField-removed102-", "correctedField-removed411-", "correctedField-removed686-")

  lazy val namesToTest: Seq[String] = rawnames.flatMap { rawname =>
    (0 until 10).map { i =>
      rawname + i.toString
    }
  }

}
