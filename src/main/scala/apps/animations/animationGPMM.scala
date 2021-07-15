package apps.animations

import java.io.File

import breeze.linalg.{DenseMatrix, DenseVector}
import scalismo.io.{LandmarkIO, MeshIO, StatisticalModelIO}
import scalismo.statisticalmodel.MultivariateNormalDistribution
import scalismo.utils.Random.implicits.randomGenerator

object animationGPMM extends App{
  scalismo.initialize()

  val pdm = StatisticalModelIO.readStatisticalMeshModel(new File("data/ssmCourse/pca.h5")).get.decimate(2000)

  val lmInit = LandmarkIO.readLandmarksJson3D(new File("data/animations/orig_PCA.json")).get.take(1)
  val lmNew = LandmarkIO.readLandmarksJson3D(new File("data/animations/def_PCA.json")).get

//  val pId = pdm.referenceMesh.pointSet.findClosestPoint(lmInit.point).id
//  val trainingData = IndexedSeq((pId, lmNew.point))

  val commonLmNames = lmInit.map(_.id) intersect lmNew.map(_.id)
  val landmarksPairs = commonLmNames.map(name => (lmInit.find(_.id == name).get.point, lmNew.find(_.id == name).get.point))

  println(s"Common LMS: ${commonLmNames.length}: ${commonLmNames}")

  val trainingData = landmarksPairs.map{p =>  (pdm.referenceMesh.pointSet.findClosestPoint(p._1).id, p._2)}.toIndexedSeq

  val posterior = pdm.posterior(trainingData, 5.0)

  def loopList(step: Int = 1): Seq[Double] = {
    val tmp = 0 to 20 by step
    val tmm = tmp.map(f => -f)
    val fin = tmp ++ tmp.reverse ++ tmm ++ tmm.reverse
    fin.map(_.toDouble/10.0)
  }

  def parConv(steps: Seq[Double], PC: Int, rank: Int) ={
    steps.map{s =>
      val v = DenseVector.zeros[Double](rank)
      v(PC) = s
      v
    }
  }

//  val myRange = loopList(2)
//
//  val PC1 = parConv(myRange, 0, pdm.rank)
//  val PC2 = parConv(myRange, 1, pdm.rank)
//  val PC3 = parConv(myRange, 2, pdm.rank)
//  val PC4 = parConv(myRange, 3, pdm.rank)
//  val PC5 = parConv(myRange, 4, pdm.rank)
//
//  val PCS = PC1 ++ PC2 ++ PC3 ++ PC4 ++ PC5

//  PCS.zipWithIndex.foreach{case (v, i) =>
//    MeshIO.writeMesh(pdm.instance(v), new File(s"data/animations/pdm/pdm_${i}.ply"))
//  }
  val mypath = new File("data/animations/pdmPosRnd_1/")
  MeshIO.writeMesh(posterior.mean, new File(mypath, "0.ply"))
  1 until 100 foreach{case (i) =>
    val rnd = MultivariateNormalDistribution(DenseVector.zeros[Double](pdm.rank), DenseMatrix.eye[Double](pdm.rank)*1.0)
    MeshIO.writeMesh(posterior.instance(rnd.sample()), new File(mypath, s"${i}.ply"))
  }

}
