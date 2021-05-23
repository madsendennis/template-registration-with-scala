package apps.molar

import java.io.File

object data {
  val data = new File("data/molar")

  val raw = new File(data, "raw")
  val rawMeshes = new File(raw, "meshes")
  val rawLms = new File(raw, "landmarks")
//  val rawCrowns = new File("/Volumes/storage/Dropbox/Workspace/uni-data/albert/surface/trainingsets_teeth/uk6er/")
  val rawCrowns = new File("../uni-data/albert/surface/trainingsets_teeth/uk6er/")
//  val rawCrowns = new File("/Volumes/storage/Dropbox/Workspace/uni-data/albert/surface/trainingsets_teeth/uk6er_extended/")

  val aligned = new File(data, "aligned")
  val alignedMeshesComplete = new File(aligned, "meshesComplete")
  val alignedLmsComplete = new File(aligned, "landmarksComplete")
  val alignedMeshesPartial = new File(aligned, "meshesPartial")
  val alignedLmsPartial = new File(aligned, "landmarksPartial")

  val registered = new File(data, "registered")
  val registeredMeshesCoarse = new File(registered, "meshesComplete")
  val registeredMeshesCrown = new File(registered, "meshesCrown")
  val registeredMeshesCrownICP = new File(registered, "meshesCrownICP")


  val referenceMesh = new File(data, "molar.ply")
  val referenceLms = new File(data, "ref.json")
  val gpmmCoarse = new File(data, "gpmmCoarse.h5")
  val pca = new File(data, "pca.h5")
  val pcaDetailed = new File(data, "pcaDetailed.h5")
  val augmented = new File(data, "augmented.h5")
  val augmentedAligned = new File(data, "augmentedAligned.h5")
  val completed = new File(data, "completed")

  val groundTruth = new File(data, "evaluation/groundTruths")

  val log = new File(data, "log")
}
