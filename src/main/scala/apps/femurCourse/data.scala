package apps.femurCourse

import java.io.File

object data {
  val data = new File("data/ssmCourse")
  val step2 = new File(data, "step2")
  val step3 = new File(data, "paritals_step3")
  val trainingMeshes = new File(step2, "meshes")
  val trainingLms = new File(step2, "landmarks")
  val aligned = new File(data, "aligned")
  val alignedMeshes = new File(aligned, "meshes")
  val alignedLms = new File(aligned, "landmarks")
  val registeredMeshes = new File(data, "registered")
  val referenceMesh = new File(data, "femur.stl")
  val referenceLms = new File(data, "femur.json")
  val gpmm = new File(data, "gpmm.h5")
  val pca = new File(data, "pca.h5")
  val augmented = new File(data, "augmented.h5")
  val completed = new File(data, "completed")

  val groundTruth = new File(data, "evaluation/groundTruths")
}
