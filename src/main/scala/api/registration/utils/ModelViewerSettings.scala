package api.registration.utils

import scalismo.geometry._3D
import scalismo.transformations.TranslationAfterRotation
import scalismo.ui.api.ShapeModelTransformationView


case class modelViewer(modelView: ShapeModelTransformationView, updateFrequency: Int)

trait ModelViewerHelper[D]{
  def getRigidTransformation(pars: SimilarityTransformParameters[D]): TranslationAfterRotation[_3D]
}

object ModelViewerHelper{
  implicit object ModelViewerHelper3D extends ModelViewerHelper[_3D] {
    override def getRigidTransformation(pars: SimilarityTransformParameters[_3D]): TranslationAfterRotation[_3D] = {
      TranslationAfterRotation(pars.t, pars.R)
    }
  }
}