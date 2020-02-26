#include "qgl_viewport.h"
#include <thread>

namespace srrg2_qgl_viewport {
  QGLViewport::QGLViewport(QWidget* parent) :
    QGLViewer(parent),
    _last_key_event(QEvent::None, 0, Qt::NoModifier),
    _last_key_event_processed(true),
    _camera(0) {
    _point_size = 2.0;
    _line_width = 2.0;

    _show_normals = false;
    _lighting     = false;
    _background   = BackgroundMode::Light;

    // ia show the viewport on construction
    show();
  }

  QGLViewport::~QGLViewport() {
    delete _packet_processor;
  }

  void QGLViewport::init() {
    // Init QGLViewer.
    QGLViewer::init();

    // Set background color white.
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    // setBackgroundColor(QColor::fromRgb(255, 255, 255));

    // Set some default settings.
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_NORMALIZE);
    glShadeModel(GL_FLAT);
    glDisable(GL_LIGHTING);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Don't save state.
    setStateFileName(QString::null);

    // Mouse bindings.
    setMouseBinding(Qt::NoModifier, Qt::RightButton, CAMERA, ZOOM);
    setMouseBinding(Qt::NoModifier, Qt::MidButton, CAMERA, TRANSLATE);
    setMouseBinding(Qt::ControlModifier, Qt::LeftButton, RAP_FROM_PIXEL);

    // ia key description
    setKeyDescription(Qt::Key_B, "Toggle background [Dark or Light]");
    setKeyDescription(Qt::Key_N, "Shows normals");
    setKeyDescription(Qt::Key_L, "Toggle light rendering");
    setKeyDescription(Qt::Key_1, "Toggle visualization of payload packets - aka array of Vector3f");
    setKeyDescription(Qt::Key_2,
                      "Toggle visualization of objects packets - aka Pyramid, Sphere, ...");
    setKeyDescription(Qt::Key_3,
                      "Toggle visualization of point vectors - aka PointNormalXGVectors");
    setKeyDescription(Qt::Key_4, "Increase normal scale by factor 2");
    setKeyDescription(Qt::Key_5, "Decrese normal scale by factor 2");

    // Replace camera.
    qglviewer::Camera* oldcam = camera();
    _camera                   = new StandardCamera();
    setCamera(_camera);
    _camera->setPosition(qglviewer::Vec(0.0f, 0.0f, 10.0f));
    _camera->setUpVector(qglviewer::Vec(0.0f, 0.0f, 0.0f));
    _camera->lookAt(qglviewer::Vec(0.0f, 0.0f, 0.0f));
    delete oldcam;

    // ia create a new qgl packet processor
    _packet_processor = new QGLPacketProcessor(&_cv_mat_vector);
  }

  void QGLViewport::draw() {
    if (!param_source.value())
      throw std::runtime_error("QGLViewport::draw|missing a BufferSource, quit");

    // ia clear background - overridden if you call setBackgroundColor
    if (_background)
      glClearColor(0.95, 0.95, 0.95, 1.0);
    else
      glClearColor(0.15, 0.15, 0.15, 1.00);

    glPushAttrib(GL_COLOR | GL_POINT_SIZE | GL_LINE_WIDTH);
    glPushMatrix();
    glPointSize(_point_size);
    glLineWidth(_line_width);

    // ia set the initial drawing color to blue,
    // ia so that no matter the backgroud color, everything will be visible
    glColor4f(srrg2_core::ColorPalette::color4fBlue().x(),
              srrg2_core::ColorPalette::color4fBlue().y(),
              srrg2_core::ColorPalette::color4fBlue().z(),
              1.0f);

    // ia reset cv canvasses
    if (_cv_mat_vector.size()) {
      _cv_mat_vector.clear();
    }

    // ia draw
    srrg2_core::BufferMemory* buffer_rec = param_source->getBuffer();
    // std::cerr << "[QGLViewport] received buffer with "
    //           << buffer_rec->num_packets << " packets" << std::endl;

    _deserializer.setBuffer(buffer_rec);
    for (size_t i = 0; i < buffer_rec->num_packets; ++i) {
      srrg2_core::PacketBase* packet = _deserializer.getPacket();
      _packet_processor->processPacket(packet);
      delete packet;
    }

    glPopMatrix();
    glPopAttrib();

    param_source->releaseBuffer(buffer_rec);
  }

  void QGLViewport::update() {
    if (!_qapp_ptr)
      throw std::runtime_error("QGLViewport::update|invalid qglserver, exit");

    if (!QGLViewer::isVisible())
      return;

    std::this_thread::sleep_for(
      std::chrono::milliseconds(param_rendering_sleep_milliseconds.value()));

    // ia gl rendering loop
    QGLViewer::updateGL();
    _qapp_ptr->processEvents();

    // ia cv rendering loop
    if (_cv_mat_vector.size()) {
      for (size_t i = 0; i < _cv_mat_vector.size(); ++i) {
        const std::string cv_window_name = "opencv canvas #" + std::to_string(i);
        cv::imshow(cv_window_name, _cv_mat_vector[i]);
      }
    }
  }

  void QGLViewport::keyPressEvent(QKeyEvent* event_) {
    _last_key_event           = *event_;
    _last_key_event_processed = false;

    switch (event_->key()) {
      case Qt::Key_B: {
        std::cerr << "[QGLViewport::keyPressEvent]| toggle background color" << std::endl;
        if (_background)
          _background = BackgroundMode::Dark;
        else
          _background = BackgroundMode::Light;
        break;
      }

      case Qt::Key_N: {
        std::cerr << "[QGLViewport::keyPressEvent]| toggle normals" << std::endl;
        if (_packet_processor->showNormal())
          _packet_processor->setShowNormal(
            srrg2_core::PacketProcessorBase::NormalProcessingType::ShaderOnly);
        else
          _packet_processor->setShowNormal(
            srrg2_core::PacketProcessorBase::NormalProcessingType::Shown);
        break;
      }

      case Qt::Key_L: {
        std::cerr << "[QGLViewport::keyPressEvent]| toggle lightning" << std::endl;
        if (_lighting) {
          _lighting = false;
          glDisable(GL_LIGHTING);
        } else {
          _lighting = true;
          glEnable(GL_LIGHTING);
        }
        break;
      }

      case Qt::Key_1: {
        std::cerr << "[QGLViewport::keyPressEvent]| toggle raw payload packets "
                     "visualization"
                  << std::endl;
        if (_packet_processor->processPayloads())
          _packet_processor->setProcessPayloads(false);
        else
          _packet_processor->setProcessPayloads(true);
        break;
      }

      case Qt::Key_2: {
        std::cerr << "[QGLViewport::keyPressEvent]| toggle object packets visualization"
                  << std::endl;
        if (_packet_processor->processObjects())
          _packet_processor->setProcessObjects(false);
        else
          _packet_processor->setProcessObjects(true);
        break;
      }

      case Qt::Key_3: {
        std::cerr << "[QGLViewport::keyPressEvent]| toggle point vector "
                     "packets visualization"
                  << std::endl;
        if (_packet_processor->processPointVectors())
          _packet_processor->setProcessPointVectors(false);
        else
          _packet_processor->setProcessPointVectors(true);
        break;
      }

      case Qt::Key_4: {
        std::cerr << "[QGLViewport::keyPressEvent]| increasing normal scale by "
                     "factor of 2"
                  << std::endl;
        const float scale = _packet_processor->normalScale() * 2.0;
        _packet_processor->setNormalScale(scale);
        break;
      }

      case Qt::Key_5: {
        std::cerr << "[QGLViewport::keyPressEvent]| decreasing normal scale by "
                     "factor of 2"
                  << std::endl;
        const float scale = _packet_processor->normalScale() * 0.5;
        _packet_processor->setNormalScale(scale);
        break;
      }

      default:
        QGLViewer::keyPressEvent(event_);
        break;
    }

    draw();
    QGLViewer::updateGL();
  }

  QString QGLViewport::helpString() const {
    QString text("<h2>QGLViewport</h2>");
    text += "Check the <b>Keyboard</b> tab for keyboard commands\n";
    text += "Press <b>ESC</b> to quit.";
    return text;
  }

} // namespace srrg2_qgl_viewport
