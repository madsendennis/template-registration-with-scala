package api.registration.loggers

import api.GingrRegistrationState
import spray.json.DefaultJsonProtocol.jsonFormat10
import spray.json.RootJsonFormat
import spray.json.DefaultJsonProtocol._
import spray.json._
import scalismo.geometry._
import scalismo.sampling.{DistributionEvaluator, ProposalGenerator}
import scalismo.sampling.loggers.AcceptRejectLogger

import java.io.{BufferedWriter, File, FileOutputStream, IOException, OutputStreamWriter}
import java.text.SimpleDateFormat
import java.util.Calendar
import scala.collection.SortedSet
import scala.collection.mutable.ListBuffer
import scala.io.Source

case class jsonLogFormat(
                          index: Int,
                          name: String,
                          logvalue: Map[String, Double],
                          status: Boolean,
                          modelParameters: Seq[Double],
                          translation: Seq[Double],
                          rotation: Seq[Double],
                          rotationCenter: Seq[Double],
                          scaling: Double,
                          datetime: String
                        )

object JsonLoggerProtocol {
  implicit val myJsonFormatLogger: RootJsonFormat[jsonLogFormat] = jsonFormat10(jsonLogFormat.apply)
}

case class JSONStateLogger[State <: GingrRegistrationState[State]](filePath: File, evaluators: Option[Map[String, DistributionEvaluator[State]]] = None) extends AcceptRejectLogger[State] {

  import JsonLoggerProtocol._

  private var numOfRejected: Int = 0
  private var numOfAccepted: Int = 0
  private var generatedBy: SortedSet[String] = SortedSet()
  private val datetimeFormat = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss")

  def totalSamples: Int = numOfRejected + numOfAccepted

  val logStatus: ListBuffer[jsonLogFormat] = new ListBuffer[jsonLogFormat]

  private def identifier(sample: State): String = {
    val name = "some"
    generatedBy += name
    name
  }

  private def mapEvaluators(sample: State, default: Double): Map[String, Double] = {
    if (evaluators.isDefined) {
      evaluators.get.map { case (name, eval) => (name, eval.logValue(sample)) }
    }
    else {
      Map("product" -> default)
    }
  }

  override def accept(current: State, sample: State, generator: ProposalGenerator[State], evaluator: DistributionEvaluator[State]): Unit = {
    val evalValue = mapEvaluators(sample, evaluator.logValue(sample))
    logStatus += jsonLogFormat(
      totalSamples,
      identifier(sample),
      evalValue,
      status = true,
      sample.general.modelParameters.toArray.toSeq,
      sample.general.alignment.translation.parameters.toArray.toSeq,
      sample.general.alignment.rotation.parameters.toArray.toSeq,
      sample.general.alignment.rotation.center.toArray.toSeq,
      sample.general.scaling,
      datetimeFormat.format(Calendar.getInstance().getTime)
    )
    numOfAccepted += 1
  }

  // The rejected state will contain the same parameters as the previous accepted state, so no need to double store all the information
  override def reject(current: State, sample: State, generator: ProposalGenerator[State], evaluator: DistributionEvaluator[State]): Unit = {
    val evalValue = mapEvaluators(sample, evaluator.logValue(sample))
    logStatus += jsonLogFormat(
      totalSamples,
      identifier(sample),
      evalValue,
      status = false,
      Seq(),
      Seq(),
      Seq(),
      Seq(),
      sample.general.scaling,
      datetimeFormat.format(Calendar.getInstance().getTime)
    )
    numOfRejected += 1
  }

  def percentRejected: Double = BigDecimal(numOfRejected.toDouble / totalSamples.toDouble).setScale(2, BigDecimal.RoundingMode.HALF_UP).toDouble

  def percentAccepted: Double = 1.0 - percentRejected

  def writeLog(): Unit = {
    val content = logStatus.toIndexedSeq
    try {
      val writer = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(filePath.toString)))
      writer.write(content.toList.toJson.prettyPrint)
      writer.close()
    } catch {
      case e: Exception => throw new IOException("Writing JSON log file failed!")
    }
    println("Log written to: " + filePath.toString)
  }

  def loadLog(): IndexedSeq[jsonLogFormat] = {
    println(s"Loading JSON log file: ${filePath.toString}")
    val src = Source.fromFile(filePath.toString)
    val data = src.mkString.parseJson.convertTo[IndexedSeq[jsonLogFormat]]
    src.close()
    data
  }

  def prettyPrint: String = {
    logStatus.toIndexedSeq.toList.toJson.prettyPrint
  }

  def percentAcceptedOfType(id: String): Double = {
    val filtered = logStatus.filter(f => f.name == id)
    val accepted = filtered.filter(f => f.status)
    accepted.length.toDouble / filtered.length.toDouble
  }

  def percentAcceptedOfTypeLocal(id: String, localLog: ListBuffer[jsonLogFormat]): Double = {
    val filtered = localLog.filter(f => f.name == id)
    val accepted = filtered.filter(f => f.status)
    accepted.length.toDouble / filtered.length.toDouble
  }

  override def toString: String = {
    "# of Accepted: " + numOfAccepted + " = " + percentAccepted + "%\n" +
    "# of Rejected: " + numOfRejected + " = " + percentRejected + "%"
  }

}