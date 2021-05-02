package apps.femurCourse

import scalismo.common.interpolation.TriangleMeshInterpolator3D
import scalismo.geometry.{EuclideanVector, _3D}
import scalismo.io.{MeshIO, StatisticalModelIO}
import scalismo.kernels.{DiagonalKernel, GaussianKernel}
import scalismo.statisticalmodel.{GaussianProcess, LowRankGaussianProcess, PointDistributionModel}
import scalismo.statisticalmodel.dataset.DataCollection
import scalismo.ui.api.ScalismoUI
import scalismo.utils.Random.implicits.randomGenerator

object step4_createPDM extends App{
  scalismo.initialize()

  val ref = MeshIO.readMesh(data.referenceMesh).get
  val meshesRegistered = data.registeredMeshes.listFiles(_.getName.endsWith(".stl")).sorted
//  val meshes = meshesRegistered.map(f => MeshIO.readMesh(f).get)
  val meshes = meshesRegistered.map(f => MeshIO.readMesh(f).get)

  println(s"Number of meshes: ${meshes.length}")

  val dc = DataCollection.fromTriangleMesh3DSequence(ref, meshes)
  val dcGPA = DataCollection.gpa(dc)
  val pca = PointDistributionModel.createUsingPCA(dcGPA)

  val Diagkernel = DiagonalKernel(GaussianKernel[_3D](10)*3, 3)
  val gp = GaussianProcess[_3D, EuclideanVector[_3D]](Diagkernel)
  println("Computing low rank")
  val lowRankAug = LowRankGaussianProcess.approximateGPCholesky(ref, gp, relativeTolerance = 0.1, interpolator = TriangleMeshInterpolator3D[EuclideanVector[_3D]])
  println(s"Low-rank created with rank ${lowRankAug.rank}")
  val augModel = PointDistributionModel.augmentModel(pca, lowRankAug.truncate(200))

  StatisticalModelIO.writeStatisticalTriangleMeshModel3D(pca, data.pca)
  StatisticalModelIO.writeStatisticalTriangleMeshModel3D(augModel, data.augmented)

  println("Model saved!")
  val ui = ScalismoUI()
  ui.show(ui.createGroup("pca"), pca, "model")
  ui.show(ui.createGroup("augmented"), augModel, "aug")
}
