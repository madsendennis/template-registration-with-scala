package apps.kernels

import java.io.File

import api.registration.utils.{GPMMTriangleMesh3D, GaussianKernelParameters, PointSetHelper, ScalismoUiHelper}
import apps.kernels.CompareKernels.{gpmmHelp, maxDist, reference}
import breeze.linalg.{DenseMatrix, DenseVector}
import scalismo.common.DiscreteField.ScalarMeshField
import scalismo.geometry.{EuclideanVector, Point, _3D}
import scalismo.io.{LandmarkIO, MeshIO, StatisticalModelIO}
import scalismo.mesh.TriangleMesh
import scalismo.statisticalmodel.{MultivariateNormalDistribution, PointDistributionModel}
import scalismo.transformations.{Rotation, Translation, TranslationAfterRotation}
import scalismo.ui.api.{ScalismoUI, _3DLeft, _3DRight}
import scalismo.ui.model.properties.ScalarRange
import scalismo.utils.Random.implicits.randomGenerator

object VisualizeKernels extends App{
  scalismo.initialize()
  val p1 = (0.0 to 3.0 by 0.1)
  val p2 = p1.reverse
  val p3 = p1.map(f => f.*(-1.0))
  val p4 = p3.reverse
  val ss = p1 ++ p2 ++ p3 ++ p4


  val armadillo = MeshIO.readMesh(new File("data/armadillo/armadillo_scaled.ply")).get.operations.decimate(2000)
  val armailloLms = LandmarkIO.readLandmarksJson3D(new File("data/armadillo/armadillo.json")).get

  val toePoint = armailloLms.find(_.id=="foot_right").get
  val toeId = armadillo.pointSet.findClosestPoint(toePoint.point).id

  val reference = armadillo



  def showModelStuff(model: PointDistributionModel[_3D, TriangleMesh], maxColorRange: Float = 15.0f, name: String): Unit = {
    val gpmmHelp = GPMMTriangleMesh3D(model.reference, relativeTolerance = 0.0)
    val color = gpmmHelp.computeDistanceAbsMesh(model, toeId)
    val colorGroup = ui.createGroup("correlation")
    val showColor = ui.show(colorGroup, color, "correlation")
    val lms = ui.show(colorGroup, toePoint, "landmark")
    val modelGroup = ui.createGroup("model")
    val showModel = ui.show(modelGroup, model, s"GPMM-${name}")
    showColor.scalarRange = ScalarRange(0.0f, maxColorRange)
    ui.setVisibility(showModel.referenceView, List(_3DLeft))
    ui.setVisibility(showColor, List(_3DRight))
    ui.setVisibility(lms, List(_3DRight))
    val shapeTransView = showModel.shapeModelTransformationView.shapeTransformationView
    var pars = DenseVector.zeros[Double](model.rank)
//
//    def modelUpdate(msWait: Long = 0) = {
//      shapeTransView.coefficients = pars
//      Thread.sleep(msWait)
//    }
//    Thread.sleep(5000)
//    (0 until 5).foreach{pc =>
//      ss.foreach{i =>
//        pars(pc) = i
//        modelUpdate()
//      }
//      pars(pc) = 0
//      modelUpdate()
//    }
//    val mvnd = MultivariateNormalDistribution(DenseVector.zeros[Double](model.rank), DenseMatrix.eye[Double](model.rank)*0.5)
//    (0 until 50).foreach{i =>
//      pars = mvnd.sample()
//      modelUpdate(200)
//    }
//    ui.setVisibility(showModel.referenceView, List())
//    ui.setVisibility(showColor, List())
//    ui.setVisibility(lms, List())
//    colorGroup.remove()
//    modelGroup.remove()
  }

  val gpmmGuassS = StatisticalModelIO.readStatisticalTriangleMeshModel3D(new File("data/armadillo/models/gaussS.h5")).get
  val gpmmGuassM = StatisticalModelIO.readStatisticalTriangleMeshModel3D(new File("data/armadillo/models/gaussM.h5")).get
  val gpmmGuassL = StatisticalModelIO.readStatisticalTriangleMeshModel3D(new File("data/armadillo/models/gaussL.h5")).get
  val gpmmGuassMix = StatisticalModelIO.readStatisticalTriangleMeshModel3D(new File("data/armadillo/models/gaussMix.h5")).get
  val gpmmGuassLsymm = StatisticalModelIO.readStatisticalTriangleMeshModel3D(new File("data/armadillo/models/gaussLsymm.h5")).get
  val gpmmInvLap = StatisticalModelIO.readStatisticalTriangleMeshModel3D(new File("data/armadillo/models/invLap.h5")).get
  val gpmmInvLapDot = StatisticalModelIO.readStatisticalTriangleMeshModel3D(new File("data/armadillo/models/invLapDot.h5")).get

  val ui = ScalismoUI()
  ScalismoUiHelper.SetDualView(ui)
//  println("Wait 10 sec GaussL:");Thread.sleep(10000)
//  showModelStuff(gpmmGuassL, name = "GaussLarge")
//  println("Wait 2 sec GaussM:");Thread.sleep(2000)
//  showModelStuff(gpmmGuassM, name = "GaussMedium")
//  println("Wait 2 sec GaussS:");Thread.sleep(2000)
//  showModelStuff(gpmmGuassS, name = "GaussSmall")
//  println("Wait 2 sec GaussMix:");Thread.sleep(2000)
//  showModelStuff(gpmmGuassMix, name = "GaussMix")
//  println("Wait 2 sec GaussSymm:");Thread.sleep(2000)
//  showModelStuff(gpmmGuassLsymm, name = "GaussSymmetry")
//  println("Wait 2 sec InvLap:");Thread.sleep(2000)
  showModelStuff(gpmmInvLap, 60.0f, name = "InvLaplacian")
//  println("Wait 2 sec InvLapDot:");Thread.sleep(2000)
//  showModelStuff(gpmmInvLapDot, 60.0f, name = "InvLaplacianDot")
//  println("Wait 2 sec DONE!");Thread.sleep(2000)
}
