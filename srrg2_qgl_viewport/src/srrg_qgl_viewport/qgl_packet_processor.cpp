#include "qgl_packet_processor.h"
#include "gl_helpers.h"
// I will also include this shit here otherwise bad things will happen in my IDE
// (hey Emacs! how are you?)
#include <srrg_viewer/viewer_core/packet_processor.h>
#include <srrg_viewer/viewer_core/packets.h>

namespace srrg2_qgl_viewport {

  using namespace srrg2_core;

  QGLPacketProcessor::QGLPacketProcessor(CvMatVector* cv_vector_ = nullptr) :
    PacketProcessorBase::PacketProcessorBase() {
    assert(cv_vector_ && "QGLPacketProcessor::QGLPacketProcessor|invalid cv vector");
    _cv_mat_vector = cv_vector_;
  }

  QGLPacketProcessor::~QGLPacketProcessor() {
  }

  void QGLPacketProcessor::processPacket(PacketBase* packet_) {
    switch (packet_->type) {
      case (PACKET_TYPE_POINTS):
        if (_process_payloads) {
          _drawPoints(dynamic_cast<PacketPayloadPoints*>(packet_));
        }
        break;
      case (PACKET_TYPE_LINES):
        if (_process_payloads) {
          _drawLines(dynamic_cast<PacketPayloadLines*>(packet_));
        }
        break;
      case (PACKET_TYPE_SEGMENTS):
        if (_process_payloads) {
          _drawSegments(dynamic_cast<PacketPayloadSegments*>(packet_));
        }
        break;

      case (PACKET_TYPE_COLOR_RGBA):
        _setAttributeColorRGBA(dynamic_cast<PacketAttributeColorRGBA*>(packet_));
        break;
      case (PACKET_TYPE_COLOR_RGB):
        _setAttributeColorRGB(dynamic_cast<PacketAttributeColorRGB*>(packet_));
        break;
      case (PACKET_TYPE_POINT_SIZE):
        _setAttributePointSize(dynamic_cast<PacketAttributePointSize*>(packet_));
        break;
      case (PACKET_TYPE_LINE_WIDTH):
        _setAttributeLineWidth(dynamic_cast<PacketAttributeLineWidth*>(packet_));
        break;

      case (PACKET_TYPE_PUSH_MATRIX):
        glPushMatrix();
        break;
      case (PACKET_TYPE_POP_MATRIX):
        glPopMatrix();
        break;
      case (PACKET_TYPE_PUSH_COLOR):
        glPushAttrib(GL_CURRENT_BIT);
        break;
      case (PACKET_TYPE_PUSH_POINT_SIZE):
        glPushAttrib(GL_POINT_BIT);
        break;
      case (PACKET_TYPE_PUSH_LINE_WIDTH):
        glPushAttrib(GL_LINE_BIT);
        break;
      case (PACKET_TYPE_POP_ATTRIBUTE):
        glPopAttrib();
        break;

      case (PACKET_TYPE_PLANE):
        if (_process_objects) {
          _drawPlane(dynamic_cast<PacketObjectPlane*>(packet_));
        }
        break;
      case (PACKET_TYPE_BOX):
        if (_process_objects) {
          _drawBox(dynamic_cast<PacketObjectBox*>(packet_));
        }
        break;
      case (PACKET_TYPE_BOX_WIREFRAME):
        if (_process_objects) {
          _drawBoxWireframe(dynamic_cast<PacketObjectBoxWireframe*>(packet_));
        }
        break;
      case (PACKET_TYPE_SPHERE):
        if (_process_objects) {
          _drawSphere(dynamic_cast<PacketObjectSphere*>(packet_));
        }
        break;
      case (PACKET_TYPE_DISK):
        if (_process_objects) {
          _drawDisk(dynamic_cast<PacketObjectDisk*>(packet_));
        }
        break;
      case (PACKET_TYPE_PYRAMID):
        if (_process_objects) {
          _drawPyramid(dynamic_cast<PacketObjectPyramid*>(packet_));
        }
        break;
      case (PACKET_TYPE_PYRAMID_WIREFRAME):
        if (_process_objects) {
          _drawPyramidWireframe(dynamic_cast<PacketObjectPyramidWireframe*>(packet_));
        }
        break;
      case (PACKET_TYPE_ELLIPSOID):
        if (_process_objects) {
          _drawEllipsoid(dynamic_cast<PacketObjectEllipsoid*>(packet_));
        }
        break;
      case (PACKET_TYPE_CONE):
        if (_process_objects) {
          _drawCone(dynamic_cast<PacketObjectCone*>(packet_));
        }
        break;
      case (PACKET_TYPE_CYLINDER):
        if (_process_objects) {
          _drawCylinder(dynamic_cast<PacketObjectCylinder*>(packet_));
        }
        break;
      case (PACKET_TYPE_ARROW2D):
        if (_process_objects) {
          _drawArrow2D(dynamic_cast<PacketObjectArrow2D*>(packet_));
        }
        break;
      case (PACKET_TYPE_REFERENCE_FRAME):
        if (_process_objects) {
          _drawReferenceFrame(dynamic_cast<PacketObjectReferenceFrame*>(packet_));
        }
        break;
      case (PACKET_TYPE_TEXT):
        if (_process_objects) {
          _drawText(dynamic_cast<PacketObjectText*>(packet_));
        }
        break;

      case (PACKET_TYPE_MULT_TRASLATION):
        _multTranslation(dynamic_cast<PacketTransformMultTraslation*>(packet_));
        break;
      case (PACKET_TYPE_MULT_SCALE):
        _multScale(dynamic_cast<PacketTransformMultScale*>(packet_));
        break;
      case (PACKET_TYPE_MULT_ROTATION):
        _multRotation(dynamic_cast<PacketTransformMultRotation*>(packet_));
        break;
      case (PACKET_TYPE_MULT_MATRIX4F):
        _multMatrix(dynamic_cast<PacketTransformMultMatrix*>(packet_));
        break;

      case (PACKET_TYPE_POINT_2F_VECTOR):
        if (_process_point_vectors) {
          _drawPointVector2f(dynamic_cast<PacketPoint2fVectorCloud*>(packet_));
        }
        break;
      case (PACKET_TYPE_POINT_NORMAL_2F_VECTOR):
        if (_process_point_vectors) {
          _drawPointVector2f(dynamic_cast<PacketPointNormal2fVectorCloud*>(packet_));
        }
        break;
      case (PACKET_TYPE_POINT_NORMAL_RGB_2F_VECTOR):
        if (_process_point_vectors) {
          _drawPointVector2f(dynamic_cast<PacketPointNormalColor2fVectorCloud*>(packet_));
        }
        break;
      case (PACKET_TYPE_POINT_NORMAL_CURVATURE_3D_VECTOR):
        if (_process_point_vectors) {
          _drawPointVector3f(dynamic_cast<PacketPointNormalCurvature3fVectorCloud*>(packet_));
        }
        break;
      case (PACKET_TYPE_POINT_3F_VECTOR):
        if (_process_point_vectors) {
          _drawPointVector3f(dynamic_cast<PacketPoint3fVectorCloud*>(packet_));
        }
        break;
      case (PACKET_TYPE_POINT_NORMAL_3F_VECTOR):
        if (_process_point_vectors) {
          _drawPointVector3f(dynamic_cast<PacketPointNormal3fVectorCloud*>(packet_));
        }
        break;
      case (PACKET_TYPE_POINT_NORMAL_RGB_3F_VECTOR):
        if (_process_point_vectors) {
          _drawPointVector3f(dynamic_cast<PacketPointNormalColor3fVectorCloud*>(packet_));
        }
        break;
      case (PACKET_TYPE_POINT_4F_VECTOR):
        if (_process_point_vectors) {
          _drawPointVector4f(dynamic_cast<PacketPoint4fVectorCloud*>(packet_));
        }
        break;
      case (PACKET_TYPE_POINT_NORMAL_4F_VECTOR):
        if (_process_point_vectors) {
          _drawPointVector4f(dynamic_cast<PacketPointNormal4fVectorCloud*>(packet_));
        }
        break;
      case (PACKET_TYPE_POINT_NORMAL_RGB_4F_VECTOR):
        if (_process_point_vectors) {
          _drawPointVector4f(dynamic_cast<PacketPointNormalColor4fVectorCloud*>(packet_));
        }
        break;

      case (PACKET_TYPE_POINT_2F_MATRIX):
        if (_process_point_vectors) {
          _drawPointMatrix2f(dynamic_cast<PacketPoint2fMatrixCloud*>(packet_));
        }
        break;
      case (PACKET_TYPE_POINT_NORMAL_2F_MATRIX):
        if (_process_point_vectors) {
          _drawPointMatrix2f(dynamic_cast<PacketPointNormal2fMatrixCloud*>(packet_));
        }
        break;
      case (PACKET_TYPE_POINT_NORMAL_RGB_2F_MATRIX):
        if (_process_point_vectors) {
          _drawPointMatrix2f(dynamic_cast<PacketPointNormalColor2fMatrixCloud*>(packet_));
        }
        break;
      case (PACKET_TYPE_POINT_3F_MATRIX):
        if (_process_point_vectors) {
          _drawPointMatrix3f(dynamic_cast<PacketPoint3fMatrixCloud*>(packet_));
        }
        break;
      case (PACKET_TYPE_POINT_NORMAL_3F_MATRIX):
        if (_process_point_vectors) {
          _drawPointMatrix3f(dynamic_cast<PacketPointNormal3fMatrixCloud*>(packet_));
        }
        break;
      case (PACKET_TYPE_POINT_NORMAL_RGB_3F_MATRIX):
        if (_process_point_vectors) {
          _drawPointMatrix3f(dynamic_cast<PacketPointNormalColor3fMatrixCloud*>(packet_));
        }
        break;
      case (PACKET_TYPE_POINT_4F_MATRIX):
        if (_process_point_vectors) {
          _drawPointMatrix4f(dynamic_cast<PacketPoint4fMatrixCloud*>(packet_));
        }
        break;
      case (PACKET_TYPE_POINT_NORMAL_4F_MATRIX):
        if (_process_point_vectors) {
          _drawPointMatrix4f(dynamic_cast<PacketPointNormal4fMatrixCloud*>(packet_));
        }
        break;
      case (PACKET_TYPE_POINT_NORMAL_RGB_4F_MATRIX):
        if (_process_point_vectors) {
          _drawPointMatrix4f(dynamic_cast<PacketPointNormalColor4fMatrixCloud*>(packet_));
        }
        break;
      case (PACKET_TYPE_POINT_INTENSITY_DESCRIPTOR_2F_VECTOR):
        if (_process_point_vectors) {
          _drawPointVector2f(dynamic_cast<PacketPointIntensityDescriptor2fVectorCloud*>(packet_));
        }
        break;
      case (PACKET_TYPE_POINT_INTENSITY_DESCRIPTOR_3F_VECTOR):
        if (_process_point_vectors) {
          _drawPointVector3f(dynamic_cast<PacketPointIntensityDescriptor3fVectorCloud*>(packet_));
        }
        break;

      case (PACKET_TYPE_POLYGON_WIREFRAME_POINT_NORMAL_RGB_3F_VECTOR):
        _drawPolygonWireframe(
          dynamic_cast<PacketPolygonWireframePointNormalColor3fVectorCloud*>(packet_));
        break;
      case (PACKET_TYPE_POLYGON_POINT_NORMAL_RGB_3F_VECTOR):
        _drawPolygon(dynamic_cast<PacketPolygonPointNormalColor3fVectorCloud*>(packet_));
        break;

      case (PACKET_TYPE_VISUAL_MATCHABLE_F_VECTOR):
        _drawVisualMatchables(dynamic_cast<PacketVisualMatchablefVector*>(packet_));
        break;

      // ds FIXME
      //      case PACKET_TYPE_CV_MAT: {
      //        // ds TODO mehhh
      //        PacketCvMat* image_packet(dynamic_cast<PacketCvMat*>(packet_));
      //        assert(image_packet);
      //        cv::imshow("cv::imshow", image_packet->data);
      //        break;
      //      }

      // ia deferred processing of cv things outside gl rendering loop
      case PACKET_TYPE_CV_MAT: {
        PacketCvMat* image_packet(dynamic_cast<PacketCvMat*>(packet_));
        assert(image_packet && &image_packet->data && _cv_mat_vector &&
               "QGLPacketProcessor::processPacket|invalid cv something");
        _cv_mat_vector->push_back(image_packet->data);
        break;
      }

      case (PACKET_TYPE_END_EPOCH):
        break;
      default:
        std::printf("QGLPacketProcessor::processPacket|unknown packet [%02X]\n", packet_->type);
        throw std::runtime_error("QGLPacketProcessor::processPacket|maybe you forgot to define the "
                                 "function to process it?");
    }
  }

  void QGLPacketProcessor::_drawPoints(PacketPayloadPoints* packet_) {
    // ia points
    glBegin(GL_POINTS);
    for (size_t i = 0; i < packet_->num_points; ++i) {
      const srrg2_core::Vector3f& p = packet_->points[i];
      glVertex3f(p.x(), p.y(), p.z());
    }
    glEnd();

    // ia normals
    glBegin(GL_LINES);
    for (size_t i = 0; i < packet_->num_normals; ++i) {
      const srrg2_core::Vector3f& p = packet_->points[i];
      const srrg2_core::Vector3f n  = packet_->normals[i] * _normal_scale;
      glVertex3f(p.x(), p.y(), p.z());
      glVertex3f(p.x() + n.x(), p.y() + n.y(), p.z() + n.z());
      glNormal3f(n.x(), n.y(), n.z());
    }
    glEnd();
  }

  void QGLPacketProcessor::_drawLines(PacketPayloadLines* packet_) {
    glBegin(GL_LINES);
    srrg2_core::Vector3f* prev_p = 0;
    for (size_t i = 0; i < packet_->num_points; ++i) {
      srrg2_core::Vector3f& p = packet_->points[i];

      if (prev_p) {
        glVertex3f(prev_p->x(), prev_p->y(), prev_p->z());
        glVertex3f(p.x(), p.y(), p.z());
      }

      prev_p = &p;
    }

    // ia normals
    for (size_t i = 0; i < packet_->num_normals; ++i) {
      const srrg2_core::Vector3f& p = packet_->points[i];
      const srrg2_core::Vector3f n  = packet_->normals[i] * _normal_scale;
      glVertex3f(p.x(), p.y(), p.z());
      glVertex3f(p.x() + n.x(), p.y() + n.y(), p.z() + n.z());
      glNormal3f(n.x(), n.y(), n.z());
    }
    glEnd();
  }

  void QGLPacketProcessor::_drawSegments(PacketPayloadSegments* packet_) {
    glBegin(GL_LINES);
    // ia segments
    for (size_t i = 0; i < packet_->num_points;) {
      const srrg2_core::Vector3f& p0 = packet_->points[i++];
      const srrg2_core::Vector3f& p1 = packet_->points[i++];

      glVertex3f(p0.x(), p0.y(), p0.z());
      glVertex3f(p1.x(), p1.y(), p1.z());
    }

    // ia normals
    for (size_t i = 0; i < packet_->num_normals; ++i) {
      const srrg2_core::Vector3f& p = packet_->points[i];
      const srrg2_core::Vector3f n  = packet_->normals[i] * _normal_scale;
      glVertex3f(p.x(), p.y(), p.z());
      glVertex3f(p.x() + n.x(), p.y() + n.y(), p.z() + n.z());
      glNormal3f(n.x(), n.y(), n.z());
    }
    glEnd();
  }

  void QGLPacketProcessor::_setAttributeColorRGB(PacketAttributeColorRGB* packet_color_rgb_) {
    glColor3f(packet_color_rgb_->data[0], packet_color_rgb_->data[1], packet_color_rgb_->data[2]);
  }

  void QGLPacketProcessor::_setAttributeColorRGBA(PacketAttributeColorRGBA* packet_color_rgba_) {
    glColor4f(packet_color_rgba_->data[0],
              packet_color_rgba_->data[1],
              packet_color_rgba_->data[2],
              packet_color_rgba_->data[3]);
  }

  void QGLPacketProcessor::_setAttributePointSize(PacketAttributePointSize* packet_point_size_) {
    glPointSize(packet_point_size_->data);
  }

  void QGLPacketProcessor::_setAttributeLineWidth(PacketAttributeLineWidth* packet_line_width_) {
    glPointSize(packet_line_width_->data);
  }

  void QGLPacketProcessor::_drawPlane(PacketObjectPlane* packet_) {
    drawPlane(packet_->data[0], packet_->data[1]);
  }

  void QGLPacketProcessor::_drawBox(PacketObjectBox* packet_) {
    drawBox(packet_->data[0], packet_->data[1], packet_->data[2]);
  }
  void QGLPacketProcessor::_drawBoxWireframe(PacketObjectBoxWireframe* packet_) {
    drawBoxWireframe(packet_->data[0], packet_->data[1], packet_->data[2]);
  }

  void QGLPacketProcessor::_drawSphere(PacketObjectSphere* packet_) {
    drawSphere(packet_->data);
  }

  void QGLPacketProcessor::_drawPyramid(PacketObjectPyramid* packet_) {
    drawPyramid(packet_->data[0], packet_->data[1]);
  }

  void QGLPacketProcessor::_drawPyramidWireframe(PacketObjectPyramidWireframe* packet_) {
    drawPyramidWireframe(packet_->data[0], packet_->data[1]);
  }

  void QGLPacketProcessor::_drawReferenceFrame(PacketObjectReferenceFrame* packet_) {
    drawReferenceSystem(packet_->data);
  }

  void QGLPacketProcessor::_drawText(srrg2_core::PacketObjectText* packet_) {
    drawText(packet_->data);
  }

  void QGLPacketProcessor::_drawEllipsoid(PacketObjectEllipsoid* packet_) {
    drawEllipsoid(packet_->data[0], packet_->data[1], packet_->data[2]);
  }

  void QGLPacketProcessor::_drawCone(PacketObjectCone* packet_) {
    drawCone(packet_->data[0], packet_->data[1]);
  }

  void QGLPacketProcessor::_drawDisk(PacketObjectDisk* packet_) {
    drawDisk(packet_->data);
  }

  void QGLPacketProcessor::_drawCylinder(PacketObjectCylinder* packet_) {
    drawCylinder(packet_->data[0], packet_->data[1]);
  }

  void QGLPacketProcessor::_drawArrow2D(PacketObjectArrow2D* packet_) {
    drawArrow2D(packet_->data[0], packet_->data[1], packet_->data[2]);
  }

  void QGLPacketProcessor::_multTranslation(PacketTransformMultTraslation* packet_) {
    Eigen::Isometry3f iso = Eigen::Isometry3f::Identity();
    iso.translation()     = packet_->data;
    glMultMatrix(iso);
  }

  void QGLPacketProcessor::_multScale(PacketTransformMultScale* packet_) {
  }

  void QGLPacketProcessor::_multRotation(PacketTransformMultRotation* packet_) {
    Eigen::Isometry3f iso = Eigen::Isometry3f::Identity();
    iso.linear()          = packet_->data;
    glMultMatrix(iso);
  }

  void QGLPacketProcessor::_multMatrix(PacketTransformMultMatrix* packet_) {
    glMultMatrixf(packet_->data.data());
  }

  void QGLPacketProcessor::_drawPointVector2f(srrg2_core::PacketPoint2fVectorCloud* packet_) {
    // ia points
    glBegin(GL_POINTS);
    for (size_t i = 0; i < packet_->num_elements; ++i) {
      const Point2f& p = packet_->data_vector->at(i);
      glVertex3f(p.coordinates().x(), p.coordinates().y(), 0.0);
    }
    glEnd();
  }

  void QGLPacketProcessor::_drawPointVector2f(srrg2_core::PacketPointNormal2fVectorCloud* packet_) {
    // ia points
    glBegin(GL_POINTS);
    for (size_t i = 0; i < packet_->num_elements; ++i) {
      const PointNormal2f& p = packet_->data_vector->at(i);
      glVertex3f(p.coordinates().x(), p.coordinates().y(), 0.0);
      glNormal3f(p.normal().x(), p.normal().y(), 0.0);
    }
    glEnd();

    // ia normals
    if (!_show_normal)
      return;
    glBegin(GL_LINES);
    for (size_t i = 0; i < packet_->num_elements; ++i) {
      const PointNormal2f& p = packet_->data_vector->at(i);
      const Vector2f n       = p.normal() * _normal_scale;

      glVertex3f(p.coordinates().x(), p.coordinates().y(), 0.0);
      glVertex3f(p.coordinates().x() + n.x(), p.coordinates().y() + n.y(), 0.0);
    }
    glEnd();
  }

  void
  QGLPacketProcessor::_drawPointVector2f(srrg2_core::PacketPointNormalColor2fVectorCloud* packet_) {
    // ia points
    glPushAttrib(GL_COLOR);
    glBegin(GL_POINTS);
    for (size_t i = 0; i < packet_->num_elements; ++i) {
      const PointNormalColor2f& p = packet_->data_vector->at(i);
      const Vector3f& p_color     = p.color();
      glColor3f(p_color[0], p_color[1], p_color[2]);
      glVertex3f(p.coordinates().x(), p.coordinates().y(), 0.0);
      glNormal3f(p.normal().x(), p.normal().y(), 0.0);
    }
    glEnd();
    glPopAttrib();

    // ia normals
    if (!_show_normal)
      return;

    glPushAttrib(GL_COLOR);
    glBegin(GL_LINES);
    for (size_t i = 0; i < packet_->num_elements; ++i) {
      const PointNormalColor2f& p = packet_->data_vector->at(i);
      const Vector2f n            = p.normal() * _normal_scale;
      const Vector3f& p_color     = p.color();
      glColor3f(p_color[0], p_color[1], p_color[2]);

      glVertex3f(p.coordinates().x(), p.coordinates().y(), 0.0);
      glVertex3f(p.coordinates().x() + n.x(), p.coordinates().y() + n.y(), 0.0);
    }
    glEnd();
    glPopAttrib();
  }

  void QGLPacketProcessor::_drawPointVector3f(srrg2_core::PacketPoint3fVectorCloud* packet_) {
    glBegin(GL_POINTS);
    for (size_t i = 0; i < packet_->num_elements; ++i) {
      const Point3f& p = packet_->data_vector->at(i);
      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
    }
    glEnd();
  }

  void QGLPacketProcessor::_drawPointVector3f(srrg2_core::PacketPointNormal3fVectorCloud* packet_) {
    // ia points
    glBegin(GL_POINTS);
    for (size_t i = 0; i < packet_->num_elements; ++i) {
      const PointNormal3f& p = packet_->data_vector->at(i);

      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
      glNormal3f(p.normal().x(), p.normal().y(), p.normal().z());
    }
    glEnd();

    // ia normals
    if (!_show_normal)
      return;
    glBegin(GL_LINES);
    for (size_t i = 0; i < packet_->num_elements; ++i) {
      const PointNormal3f& p = packet_->data_vector->at(i);
      const Vector3f n       = p.normal() * _normal_scale;

      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
      glVertex3f(
        p.coordinates().x() + n.x(), p.coordinates().y() + n.y(), p.coordinates().z() + n.z());
    }
    glEnd();
  }

  void QGLPacketProcessor::_drawPointVector3f(
    srrg2_core::PacketPointNormalCurvature3fVectorCloud* packet_) {
    // ia points
    glBegin(GL_POINTS);
    for (size_t i = 0; i < packet_->num_elements; ++i) {
      const PointNormalCurvature3f& p = packet_->data_vector->at(i);

      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
      glNormal3f(p.normal().x(), p.normal().y(), p.normal().z());
    }
    glEnd();

    // ia normals
    if (!_show_normal)
      return;
    glBegin(GL_LINES);
    for (size_t i = 0; i < packet_->num_elements; ++i) {
      const PointNormalCurvature3f& p = packet_->data_vector->at(i);
      const Vector3f n                = p.normal() * _normal_scale;

      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
      glVertex3f(
        p.coordinates().x() + n.x(), p.coordinates().y() + n.y(), p.coordinates().z() + n.z());
    }
    glEnd();
  }

  void
  QGLPacketProcessor::_drawPointVector3f(srrg2_core::PacketPointNormalColor3fVectorCloud* packet_) {
    // ia points
    glPushAttrib(GL_COLOR);
    glBegin(GL_POINTS);
    for (size_t i = 0; i < packet_->num_elements; ++i) {
      const PointNormalColor3f& p = packet_->data_vector->at(i);
      const Vector3f& p_color     = p.color();
      glColor3f(p_color[0], p_color[1], p_color[2]);
      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
      glNormal3f(p.normal().x(), p.normal().y(), p.normal().z());
    }
    glEnd();
    glPopAttrib();

    // ia normals
    if (!_show_normal)
      return;
    glPushAttrib(GL_COLOR);
    glBegin(GL_LINES);
    for (size_t i = 0; i < packet_->num_elements; ++i) {
      const PointNormalColor3f& p = packet_->data_vector->at(i);
      const Vector3f n            = p.normal() * _normal_scale;
      const Vector3f& p_color     = p.color();
      glColor3f(p_color[0], p_color[1], p_color[2]);

      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
      glVertex3f(
        p.coordinates().x() + n.x(), p.coordinates().y() + n.y(), p.coordinates().z() + n.z());
    }
    glEnd();
    glPopAttrib();
  }

  void QGLPacketProcessor::_drawPointVector4f(srrg2_core::PacketPoint4fVectorCloud* packet_) {
    glBegin(GL_POINTS);
    for (size_t i = 0; i < packet_->num_elements; ++i) {
      const Point4f& p = packet_->data_vector->at(i);
      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
    }
    glEnd();
  }

  void QGLPacketProcessor::_drawPointVector4f(srrg2_core::PacketPointNormal4fVectorCloud* packet_) {
    // ia points
    glBegin(GL_POINTS);
    for (size_t i = 0; i < packet_->num_elements; ++i) {
      const PointNormal4f& p = packet_->data_vector->at(i);
      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
      glNormal3f(p.normal().x(), p.normal().y(), p.normal().z());
    }
    glEnd();

    // ia normals
    if (!_show_normal)
      return;

    glBegin(GL_LINES);
    for (size_t i = 0; i < packet_->num_elements; ++i) {
      const PointNormal4f& p = packet_->data_vector->at(i);
      const Vector4f n       = p.normal() * _normal_scale;

      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
      glVertex3f(
        p.coordinates().x() + n.x(), p.coordinates().y() + n.y(), p.coordinates().z() + n.z());
    }
    glEnd();
  }

  void
  QGLPacketProcessor::_drawPointVector4f(srrg2_core::PacketPointNormalColor4fVectorCloud* packet_) {
    // ia points
    glPushAttrib(GL_COLOR);
    glBegin(GL_POINTS);
    for (size_t i = 0; i < packet_->num_elements; ++i) {
      const PointNormalColor4f& p = packet_->data_vector->at(i);
      const Vector3f& p_color     = p.color();
      glColor3f(p_color[0], p_color[1], p_color[2]);
      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
      glNormal3f(p.normal().x(), p.normal().y(), p.normal().z());
    }
    glEnd();
    glPopAttrib();

    // ia normals
    if (!_show_normal)
      return;

    glPushAttrib(GL_COLOR);
    glBegin(GL_LINES);
    for (size_t i = 0; i < packet_->num_elements; ++i) {
      const PointNormalColor4f& p = packet_->data_vector->at(i);
      const Vector4f n            = p.normal() * _normal_scale;
      const Vector3f& p_color     = p.color();
      glColor3f(p_color[0], p_color[1], p_color[2]);

      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
      glVertex3f(
        p.coordinates().x() + n.x(), p.coordinates().y() + n.y(), p.coordinates().z() + n.z());
    }
    glEnd();
    glPopAttrib();
  }

  //---------------------------------------------------------------------------
  //---------------------------------------------------------------------------
  //---------------------------------------------------------------------------
  void QGLPacketProcessor::_drawPointMatrix2f(srrg2_core::PacketPoint2fMatrixCloud* packet_) {
    // ia points
    const size_t& num_points = packet_->data_matrix->size();
    const auto& vector_data  = packet_->data_matrix->data();
    glBegin(GL_POINTS);
    for (size_t i = 0; i < num_points; ++i) {
      const Point2f& p = vector_data.at(i);
      glVertex3f(p.coordinates().x(), p.coordinates().y(), 0.0);
    }
    glEnd();
  }

  void QGLPacketProcessor::_drawPointMatrix2f(srrg2_core::PacketPointNormal2fMatrixCloud* packet_) {
    // ia points
    const size_t& num_points = packet_->data_matrix->size();
    const auto& vector_data  = packet_->data_matrix->data();
    glBegin(GL_POINTS);
    for (size_t i = 0; i < num_points; ++i) {
      const PointNormal2f& p = vector_data.at(i);
      glVertex3f(p.coordinates().x(), p.coordinates().y(), 0.0);
      glNormal3f(p.normal().x(), p.normal().y(), 0.0);
    }
    glEnd();

    // ia normals
    if (!_show_normal)
      return;
    glBegin(GL_LINES);
    for (size_t i = 0; i < num_points; ++i) {
      const PointNormal2f& p = vector_data.at(i);
      const Vector2f n       = p.normal() * _normal_scale;

      glVertex3f(p.coordinates().x(), p.coordinates().y(), 0.0);
      glVertex3f(p.coordinates().x() + n.x(), p.coordinates().y() + n.y(), 0.0);
    }
    glEnd();
  }

  void
  QGLPacketProcessor::_drawPointMatrix2f(srrg2_core::PacketPointNormalColor2fMatrixCloud* packet_) {
    // ia points
    const size_t& num_points = packet_->data_matrix->size();
    const auto& vector_data  = packet_->data_matrix->data();
    glPushAttrib(GL_COLOR);
    glBegin(GL_POINTS);
    for (size_t i = 0; i < num_points; ++i) {
      const PointNormalColor2f& p = vector_data.at(i);
      const Vector3f& p_color     = p.color();
      glColor3f(p_color[0], p_color[1], p_color[2]);
      glVertex3f(p.coordinates().x(), p.coordinates().y(), 0.0);
      glNormal3f(p.normal().x(), p.normal().y(), 0.0);
    }
    glEnd();
    glPopAttrib();

    // ia normals
    if (!_show_normal)
      return;

    glPushAttrib(GL_COLOR);
    glBegin(GL_LINES);
    for (size_t i = 0; i < num_points; ++i) {
      const PointNormalColor2f& p = vector_data.at(i);
      const Vector2f n            = p.normal() * _normal_scale;
      const Vector3f& p_color     = p.color();
      glColor3f(p_color[0], p_color[1], p_color[2]);

      glVertex3f(p.coordinates().x(), p.coordinates().y(), 0.0);
      glVertex3f(p.coordinates().x() + n.x(), p.coordinates().y() + n.y(), 0.0);
    }
    glEnd();
    glPopAttrib();
  }

  void QGLPacketProcessor::_drawPointMatrix3f(srrg2_core::PacketPoint3fMatrixCloud* packet_) {
    const size_t& num_points = packet_->data_matrix->size();
    const auto& vector_data  = packet_->data_matrix->data();

    glBegin(GL_POINTS);
    for (size_t i = 0; i < num_points; ++i) {
      const Point3f& p = vector_data.at(i);
      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
    }
    glEnd();
  }

  void QGLPacketProcessor::_drawPointMatrix3f(srrg2_core::PacketPointNormal3fMatrixCloud* packet_) {
    // ia points
    const size_t& num_points = packet_->data_matrix->size();
    const auto& vector_data  = packet_->data_matrix->data();

    glBegin(GL_POINTS);
    for (size_t i = 0; i < num_points; ++i) {
      const PointNormal3f& p = vector_data.at(i);

      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
      glNormal3f(p.normal().x(), p.normal().y(), p.normal().z());
    }
    glEnd();

    // ia normals
    if (!_show_normal)
      return;
    glBegin(GL_LINES);
    for (size_t i = 0; i < num_points; ++i) {
      const PointNormal3f& p = vector_data.at(i);
      const Vector3f n       = p.normal() * _normal_scale;

      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
      glVertex3f(
        p.coordinates().x() + n.x(), p.coordinates().y() + n.y(), p.coordinates().z() + n.z());
    }
    glEnd();
  }

  void
  QGLPacketProcessor::_drawPointMatrix3f(srrg2_core::PacketPointNormalColor3fMatrixCloud* packet_) {
    // ia points
    const size_t& num_points = packet_->data_matrix->size();
    const auto& vector_data  = packet_->data_matrix->data();

    glPushAttrib(GL_COLOR);
    glBegin(GL_POINTS);
    for (size_t i = 0; i < num_points; ++i) {
      const PointNormalColor3f& p = vector_data.at(i);
      const Vector3f& p_color     = p.color();
      glColor3f(p_color[0], p_color[1], p_color[2]);
      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
      glNormal3f(p.normal().x(), p.normal().y(), p.normal().z());
    }
    glEnd();
    glPopAttrib();

    // ia normals
    if (!_show_normal)
      return;

    glPushAttrib(GL_COLOR);
    glBegin(GL_LINES);
    for (size_t i = 0; i < num_points; ++i) {
      const PointNormalColor3f& p = vector_data.at(i);
      const Vector3f n            = p.normal() * _normal_scale;
      const Vector3f& p_color     = p.color();
      glColor3f(p_color[0], p_color[1], p_color[2]);

      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
      glVertex3f(
        p.coordinates().x() + n.x(), p.coordinates().y() + n.y(), p.coordinates().z() + n.z());
    }
    glEnd();
    glPopAttrib();
  }

  void QGLPacketProcessor::_drawPointMatrix4f(srrg2_core::PacketPoint4fMatrixCloud* packet_) {
    const size_t& num_points = packet_->data_matrix->size();
    const auto& vector_data  = packet_->data_matrix->data();

    glBegin(GL_POINTS);
    for (size_t i = 0; i < num_points; ++i) {
      const Point4f& p = vector_data.at(i);
      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
    }
    glEnd();
  }

  void QGLPacketProcessor::_drawPointMatrix4f(srrg2_core::PacketPointNormal4fMatrixCloud* packet_) {
    // ia points
    const size_t& num_points = packet_->data_matrix->size();
    const auto& vector_data  = packet_->data_matrix->data();

    glBegin(GL_POINTS);
    for (size_t i = 0; i < num_points; ++i) {
      const PointNormal4f& p = vector_data.at(i);
      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
      glNormal3f(p.normal().x(), p.normal().y(), p.normal().z());
    }
    glEnd();

    // ia normals
    if (!_show_normal)
      return;
    glBegin(GL_LINES);
    for (size_t i = 0; i < num_points; ++i) {
      const PointNormal4f& p = vector_data.at(i);
      const Vector4f n       = p.normal() * _normal_scale;

      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
      glVertex3f(
        p.coordinates().x() + n.x(), p.coordinates().y() + n.y(), p.coordinates().z() + n.z());
    }
    glEnd();
  }

  void
  QGLPacketProcessor::_drawPointMatrix4f(srrg2_core::PacketPointNormalColor4fMatrixCloud* packet_) {
    // ia points
    const size_t& num_points = packet_->data_matrix->size();
    const auto& vector_data  = packet_->data_matrix->data();

    glPushAttrib(GL_COLOR);
    glBegin(GL_POINTS);
    for (size_t i = 0; i < num_points; ++i) {
      const PointNormalColor4f& p = vector_data.at(i);
      const Vector3f& p_color     = p.color();
      glColor3f(p_color[0], p_color[1], p_color[2]);
      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
      glNormal3f(p.normal().x(), p.normal().y(), p.normal().z());
    }
    glEnd();
    glPopAttrib();

    // ia normals
    if (!_show_normal)
      return;

    glPushAttrib(GL_COLOR);
    glBegin(GL_LINES);
    for (size_t i = 0; i < num_points; ++i) {
      const PointNormalColor4f& p = vector_data.at(i);
      const Vector4f n            = p.normal() * _normal_scale;
      const Vector3f& p_color     = p.color();
      glColor3f(p_color[0], p_color[1], p_color[2]);

      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
      glVertex3f(
        p.coordinates().x() + n.x(), p.coordinates().y() + n.y(), p.coordinates().z() + n.z());
    }
    glEnd();
    glPopAttrib();
  }

  void QGLPacketProcessor::_drawPolygonWireframe(
    srrg2_core::PacketPolygonWireframePointNormalColor3fVectorCloud* packet_) {
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glBegin(GL_POLYGON);
    for (size_t i = 0; i < packet_->num_elements; ++i) {
      const auto& p = packet_->data_vector->at(i);
      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
      glNormal3f(p.normal().x(), p.normal().y(), p.normal().z());
    }
    glEnd();
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  }
  void QGLPacketProcessor::_drawPolygon(
    srrg2_core::PacketPolygonPointNormalColor3fVectorCloud* packet_) {
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glBegin(GL_POLYGON);
    for (size_t i = 0; i < packet_->num_elements; ++i) {
      const auto& p = packet_->data_vector->at(i);
      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
      glNormal3f(p.normal().x(), p.normal().y(), p.normal().z());
    }
    glEnd();
  }

  void QGLPacketProcessor::_drawPointVector2f(
    srrg2_core::PacketPointIntensityDescriptor2fVectorCloud* packet_) {
    // ia points
    glBegin(GL_POINTS);
    for (size_t i = 0; i < packet_->num_elements; ++i) {
      const Point2f& p = packet_->data_vector->at(i);
      glVertex3f(p.coordinates().x(), p.coordinates().y(), 0.0);
    }
    glEnd();
  }
  void QGLPacketProcessor::_drawPointVector3f(
    srrg2_core::PacketPointIntensityDescriptor3fVectorCloud* packet_) {
    glBegin(GL_POINTS);
    for (size_t i = 0; i < packet_->num_elements; ++i) {
      const Point3f& p = packet_->data_vector->at(i);
      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
    }
    glEnd();
  }

  void
  QGLPacketProcessor::_drawVisualMatchables(srrg2_core::PacketVisualMatchablefVector* packet_) {
    // ia group matchables by type
    std::vector<size_t> point_index_vector;
    std::vector<size_t> line_index_vector;
    std::vector<size_t> plane_index_vector;

    point_index_vector.reserve(packet_->num_elements);
    line_index_vector.reserve(packet_->num_elements);
    plane_index_vector.reserve(packet_->num_elements);

    for (size_t m = 0; m < packet_->num_elements; ++m) {
      const auto& matchable = packet_->data_vector->at(m);
      switch (matchable.type()) {
        case MatchableBase::Point:
          point_index_vector.emplace_back(m);
          break;
        case MatchableBase::Line:
          line_index_vector.emplace_back(m);
          break;
        case MatchableBase::Plane:
          plane_index_vector.emplace_back(m);
          break;
        default:
          break;
      }
    }

    // ia start visualizing points
    glPushAttrib(GL_CURRENT_BIT);
    glBegin(GL_POINTS);
    for (const size_t& p : point_index_vector) {
      const auto& m = packet_->data_vector->at(p);
      glColor3f(m.color().x(), m.color().y(), m.color().z());
      glVertex3f(m.origin().x(), m.origin().y(), m.origin().z());
    }
    glEnd();
    glPopAttrib();

    // ia visualizing lines
    glPushAttrib(GL_CURRENT_BIT);
    glBegin(GL_LINES);
    for (const size_t& p : line_index_vector) {
      const auto& m = packet_->data_vector->at(p);
      assert(m.support().size() &&
             "QGLPacketProcessor::_drawVisualMatchables|ERROR, no line support");
      const auto& p_start = m.support()[0].coordinates();
      const auto& p_end   = m.support()[1].coordinates();
      glColor3f(m.color().x(), m.color().y(), m.color().z());
      glVertex3f(p_start.x(), p_start.y(), p_start.z());
      glVertex3f(p_end.x(), p_end.y(), p_end.z());
    }
    glEnd();
    glPopAttrib();

    // ia visualizing planes
    glPushAttrib(GL_CURRENT_BIT);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    for (const size_t& p : plane_index_vector) {
      const auto& m = packet_->data_vector->at(p);
      assert(m.support().size() &&
             "QGLPacketProcessor::_drawVisualMatchables|ERROR, no plane support");
      glBegin(GL_POLYGON);
      glColor3f(m.color().x(), m.color().y(), m.color().z());
      for (const auto& support_point : m.support()) {
        glVertex3f(support_point.coordinates().x(),
                   support_point.coordinates().y(),
                   support_point.coordinates().z());
      }
      glEnd();
    }
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glPopAttrib();
  }

} // namespace srrg2_qgl_viewport
