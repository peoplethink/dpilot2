#include "selfdrive/ui/qt/onroad.h"

#include <QDebug>
#include <QSound>
#include <numeric>
#include <cmath>
#include <algorithm>
#include "selfdrive/common/timing.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/common/params.h"
#ifdef ENABLE_MAPS
#include "selfdrive/ui/qt/maps/map.h"
#include "selfdrive/ui/qt/maps/map_helpers.h"
#endif

// ====== [ADD] Animated Text (ported from paint.h ui_draw_text_a/ui_draw_text_a2) ======
struct AnimTextState {
  float x = 0.f;
  float y = 0.f;
  float size = 0.f;
  int   time = -1;          // <=0 이면 미표시
  QString text;
  QColor color = QColor(255, 255, 255, 255);
  QString font = "Open Sans";
};

static AnimTextState g_anim_text;
static constexpr int kAnimMax = 100;

// QPainter 외곽선/그림자 텍스트 (Nanovg ui_draw_text() 대응)
static void drawOutlinedText(QPainter &p, float x, float y,
                             const QString &text,
                             int pixelSize,
                             const QColor &fg,
                             const QString &family,
                             int weight = QFont::Bold,
                             float borderWidth = 3.0f,
                             float shadowOffset = 0.0f,
                             const QColor &borderColor = QColor(0,0,0,255),
                             const QColor &shadowColor = QColor(0,0,0,255)) {
  // paint.h에서 y += 6 하던 것과 비슷하게 약간 내려줌
  y += 6.0f;

  QFont f(family);
  f.setPixelSize(pixelSize);
  f.setWeight(weight);
  p.setFont(f);

  // baseline 기반으로 찍기
  QFontMetrics fm(p.font());
  const float baseX = x;
  const float baseY = y; // 이미 baseline 느낌으로 쓰고 있으면 그대로, 아니면 필요시 조정

  // 외곽선(8방향)
  if (borderWidth > 0.0f) {
    p.setPen(borderColor);
    static const float angs[] = {0,45,90,135,180,225,270,315};
    for (float a : angs) {
      float rad = a * M_PI / 180.0f;
      float ox = borderWidth * std::cos(rad);
      float oy = borderWidth * std::sin(rad);
      p.drawText(QPointF(baseX + ox, baseY + oy), text);
    }
  }

  // 그림자
  if (shadowOffset != 0.0f) {
    p.setPen(shadowColor);
    p.drawText(QPointF(baseX + shadowOffset, baseY + shadowOffset), text);
  }

  // 본문
  p.setPen(fg);
  p.drawText(QPointF(baseX, baseY), text);
}

static inline float lerp01(float a, float b, float t) {
  return a * t + b * (1.0f - t);
}

// 애니메이션 시작 (paint.h ui_draw_text_a 대응)
static void startAnimText(float x, float y, const QString &text, float size,
                          const QColor &color, const QString &font) {
  g_anim_text.x = x;
  g_anim_text.y = y;
  g_anim_text.size = size;
  g_anim_text.text = text;
  g_anim_text.color = color;
  g_anim_text.font = font;
  g_anim_text.time = 130;   // paint.h 동일
}

// 매 프레임 그리기 (paint.h ui_draw_text_a2 대응)
static void drawAnimText(QPainter &p, const UIState *s) {
  if (g_anim_text.time <= 0) return;

  g_anim_text.time -= 10;   // paint.h 동일
  int t1 = g_anim_text.time;
  if (t1 > 100) t1 = 100;

  // paint.h 보간식 동일 (센터 -> 목표)
  const float cx0 = (s->fb_w / 2.0f);
  const float cy0 = (s->fb_h - 400.0f);

  const float t = (float)t1 / (float)kAnimMax;          // 0~1
  const float x = lerp01(cx0, g_anim_text.x, t);
  const float y = lerp01(cy0, g_anim_text.y, t);
  const float sz = lerp01(350.0f, g_anim_text.size, t);

  // 처음(>=100)엔 두껍게, 이후엔 기본
  if (g_anim_text.time >= 100) {
    drawOutlinedText(p, x, y, g_anim_text.text, (int)sz,
                     g_anim_text.color, g_anim_text.font,
                     QFont::Black,
                     /*borderWidth=*/9.0f,
                     /*shadowOffset=*/8.0f,
                     QColor(0,0,0,255),
                     QColor(0,0,0,255));
  } else {
    drawOutlinedText(p, x, y, g_anim_text.text, (int)sz,
                     g_anim_text.color, g_anim_text.font,
                     QFont::Bold,
                     /*borderWidth=*/3.0f,
                     /*shadowOffset=*/0.0f,
                     QColor(0,0,0,255),
                     QColor(0,0,0,255));
  }
}
// ====== [ADD END] ================================================================

static inline bool calc_soft_hold_active(const cereal::ControlsState::Reader &cs,
                                         const cereal::CarState::Reader &car_state) {
  const float v_ego = car_state.getVEgo();

  const int long_state = (int)cs.getLongControlState();
  const bool enabled = cs.getEnabled();

  const bool near_standstill = (v_ego < 0.08f);
  const bool stopping_state  = (long_state == 2);

  return enabled && near_standstill && stopping_state;
}

#define FONT_OPEN_SANS "Inter" //"Open Sans"
OnroadWindow::OnroadWindow(QWidget *parent) : QWidget(parent) {
  QVBoxLayout *main_layout  = new QVBoxLayout(this);
  main_layout->setMargin(bdr_s);
  QStackedLayout *stacked_layout = new QStackedLayout;
  stacked_layout->setStackingMode(QStackedLayout::StackAll);
  main_layout->addLayout(stacked_layout);

  QStackedLayout *road_view_layout = new QStackedLayout;
  road_view_layout->setStackingMode(QStackedLayout::StackAll);
  nvg = new NvgWindow(VISION_STREAM_RGB_ROAD, this);
  road_view_layout->addWidget(nvg);
  hud = new OnroadHud(this);
  road_view_layout->addWidget(hud);

  nvg->hud = hud;

  QWidget * split_wrapper = new QWidget;
  split = new QHBoxLayout(split_wrapper);
  split->setContentsMargins(0, 0, 0, 0);
  split->setSpacing(0);
  split->addLayout(road_view_layout);

  stacked_layout->addWidget(split_wrapper);

  alerts = new OnroadAlerts(this);
  alerts->setAttribute(Qt::WA_TransparentForMouseEvents, true);
  stacked_layout->addWidget(alerts);

  // setup stacking order
  alerts->raise();

  setAttribute(Qt::WA_OpaquePaintEvent);
  QObject::connect(uiState(), &UIState::uiUpdate, this, &OnroadWindow::updateState);
  QObject::connect(uiState(), &UIState::offroadTransition, this, &OnroadWindow::offroadTransition);

#ifdef QCOM2	
  // screen recoder - neokii

  record_timer = std::make_shared<QTimer>();
	QObject::connect(record_timer.get(), &QTimer::timeout, [=]() {
    if(recorder) {
      recorder->update_screen();
    }
  });
	record_timer->start(1000/UI_FREQ);
/*
  QWidget* recorder_widget = new QWidget(this);
  QVBoxLayout * recorder_layout = new QVBoxLayout (recorder_widget);
  recorder_layout->setMargin(35);
  recorder = new ScreenRecoder(this);
  recorder_layout->addWidget(recorder);
  recorder_layout->setAlignment(recorder, Qt::AlignRight | Qt::AlignBottom);

  stacked_layout->addWidget(recorder_widget);
  recorder_widget->raise();
  alerts->raise();*/
#endif
}

void OnroadWindow::updateState(const UIState &s) {
	
  QColor bgColor = bg_colors[s.status];
  Alert alert = Alert::get(*(s.sm), s.scene.started_frame);
  alerts->updateAlert(alert);

  hud->updateState(s);

  if (bg != bgColor) {
    // repaint border
    bg = bgColor;
    update();
  }
	
  UIState *my_s = uiState();

  my_s->scene.dynamic_lane_profile = 2;
  Params().put("DynamicLaneProfile", "2", 1);
	
  if (s.scene.blinkerstatus || my_s->scene.prev_blinkerstatus) {
    update();
    my_s->scene.prev_blinkerstatus = s.scene.blinkerstatus;
    my_s->scene.blinkerframe += my_s->scene.blinkerframe < 255? +20 : -255;
  }	
}

void OnroadWindow::mouseReleaseEvent(QMouseEvent* e) {
  
#ifdef QCOM2
  // neokii
  QPoint endPos = e->pos();
  int dx = endPos.x() - startPos.x();
  int dy = endPos.y() - startPos.y();
  if(std::abs(dx) > 250 || std::abs(dy) > 200) {

    if(std::abs(dx) < std::abs(dy)) {  

      if(dy < 0) { // upward
        Params().remove("CalibrationParams");
        Params().remove("LiveParameters");
        QTimer::singleShot(1500, []() {
          Params().putBool("SoftRestartTriggered", true);
        });    

        QSound::play("../assets/sounds/reset_calibration.wav");
      }
      else { // downward
        QTimer::singleShot(500, []() {
          Params().putBool("SoftRestartTriggered", true);
        });
      }	
    }
    else if(std::abs(dx) > std::abs(dy)) {
      if(dx < 0) { // right to left
        if(recorder)
          recorder->toggle();  
        }
      }
      else { // left to right
        if(recorder)
          recorder->toggle();
      }
    }  
    return;
  }

  if (map != nullptr) {
    bool sidebarVisible = geometry().x() > 0;
    map->setVisible(!sidebarVisible && !map->isVisible());
  }

  // propagation event to parent(HomeWindow)
  QWidget::mousePressEvent(e);
#endif
}

void OnroadWindow::mousePressEvent(QMouseEvent* e) {
#ifdef QCOM2
  startPos = e->pos();
#else
  if (map != nullptr) {
    bool sidebarVisible = geometry().x() > 0;
    map->setVisible(!sidebarVisible && !map->isVisible());
  }

  // propagation event to parent(HomeWindow)
  QWidget::mouseReleaseEvent(e);
#endif  
}

void OnroadWindow::offroadTransition(bool offroad) {
#ifdef ENABLE_MAPS
  if (!offroad) {
    if (map == nullptr && (uiState()->prime_type || !MAPBOX_TOKEN.isEmpty())) {
      MapWindow * m = new MapWindow(get_mapbox_settings());
      map = m;

      QObject::connect(uiState(), &UIState::offroadTransition, m, &MapWindow::offroadTransition);

      m->setFixedWidth(topWidget(this)->width() / 2);
      split->addWidget(m, 0, Qt::AlignRight);

      // Make map visible after adding to split
      m->offroadTransition(offroad);
    }
  }
#endif

  alerts->updateAlert({});

  // update stream type
  bool wide_cam = Hardware::TICI() && Params().getBool("EnableWideCamera");
  nvg->setStreamType(wide_cam ? VISION_STREAM_RGB_WIDE_ROAD : VISION_STREAM_RGB_ROAD);

#ifdef QCOM2	
  if(offroad && recorder) {
    recorder->stop(false);
  }
#endif	
}

void OnroadWindow::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  p.fillRect(rect(), QColor(bg.red(), bg.green(), bg.blue(), 255));

  // Begin AleSato Blinker Indicator
  p.setPen(Qt::NoPen);
  UIState *s = uiState();
  p.setBrush(QBrush(QColor(0, 0, 0, 0xff)));
  if (s->scene.blinkerstatus == 1) {
    // left rectangle for blinker indicator
    float rightcorner = width() * 0.75;
    QRect blackground = QRect(0, height()*0.75, rightcorner, height());
    p.drawRect(blackground);
    float bottomsect = rightcorner / (rightcorner + (height()/4)); // time proportion
    float delta = 1 - (float(s->scene.blinkerframe)/(255*bottomsect));
    delta = std::clamp(delta, 0.0f, 1.0f);
    QRect r = QRect(rightcorner*delta, height()-30, rightcorner-(rightcorner*delta), 30);
    p.setBrush(QBrush(QColor(255, 150, 0, 255)));
    p.drawRect(r);
    float delta2 = (float(s->scene.blinkerframe) - float(255 * bottomsect)) / (255 * (1 - bottomsect));
    delta2 = std::clamp(delta2, 0.0f, 1.0f);
    r = QRect(0, height() - height()*0.25*delta2, 30, height());
    p.drawRect(r);
  } else if (s->scene.blinkerstatus == 2) {
    // right rectangle for blinker indicator
    float leftcorner = width() * 0.25;
    QRect blackground = QRect(leftcorner, height()*0.75, width(), height());
    p.drawRect(blackground);
    float bottomsect = (width() - leftcorner) / (width() - leftcorner + (height()/4)); // time proportion
    float delta = float(s->scene.blinkerframe)/(255*bottomsect);
    delta = std::clamp(delta, 0.0f, 1.0f);
    QRect r = QRect(leftcorner, height()-30, (width()-leftcorner)*delta, 30);
    p.setBrush(QBrush(QColor(255, 150, 0, 250)));
    p.drawRect(r);
    float delta2 = (float(s->scene.blinkerframe) - float(255 * bottomsect)) / (255 * (1 - bottomsect));
    delta2 = std::clamp(delta2, 0.0f, 1.0f);
    r = QRect(width()-30, height() - height()*0.25*delta2, width(), height());
    p.drawRect(r);
  }
  // End AleSato Blinker Indicator	
}

// OnroadAlerts
void OnroadAlerts::updateAlert(const Alert &a) {
  if (!alert.equal(a)) {
    alert = a;
    update();
  }
}

void OnroadAlerts::paintEvent(QPaintEvent *event) {
  if (alert.size == cereal::ControlsState::AlertSize::NONE) {
    return;
  }
  static std::map<cereal::ControlsState::AlertSize, const int> alert_heights = {
    {cereal::ControlsState::AlertSize::SMALL, 271},
    {cereal::ControlsState::AlertSize::MID, 420},
    {cereal::ControlsState::AlertSize::FULL, height()},
  };
  int h = alert_heights[alert.size];

  int margin = 40;
  int radius = 30;
  if (alert.size == cereal::ControlsState::AlertSize::FULL) {
    margin = 0;
    radius = 0;
  }
  QRect r = QRect(0 + margin, height() - h + margin, width() - margin*2, h - margin*2);

  QPainter p(this);

  // draw background + gradient
  p.setPen(Qt::NoPen);
  p.setBrush(QBrush(alert_colors[alert.status]));
  p.drawRoundedRect(r, radius, radius);	
  p.setCompositionMode(QPainter::CompositionMode_SourceOver);

  // text
  const QPoint c = r.center();
  p.setPen(QColor(0xff, 0xff, 0xff));
  p.setRenderHint(QPainter::TextAntialiasing);
  if (alert.size == cereal::ControlsState::AlertSize::SMALL) {
    configFont(p, "Open Sans", 74, "SemiBold");
    p.drawText(r, Qt::AlignCenter, alert.text1);
  } else if (alert.size == cereal::ControlsState::AlertSize::MID) {
    configFont(p, "Open Sans", 88, "Bold");
    p.drawText(QRect(0, c.y() - 125, width(), 150), Qt::AlignHCenter | Qt::AlignTop, alert.text1);
    configFont(p, "Open Sans", 66, "Regular");
    p.drawText(QRect(0, c.y() + 21, width(), 90), Qt::AlignHCenter, alert.text2);
  } else if (alert.size == cereal::ControlsState::AlertSize::FULL) {
    bool l = alert.text1.length() > 15;
    configFont(p, "Open Sans", l ? 132 : 177, "Bold");
    p.drawText(QRect(0, r.y() + (l ? 240 : 270), width(), 600), Qt::AlignHCenter | Qt::TextWordWrap, alert.text1);
    configFont(p, "Open Sans", 88, "Regular");
    p.drawText(QRect(0, r.height() - (l ? 361 : 420), width(), 300), Qt::AlignHCenter | Qt::TextWordWrap, alert.text2);
  }
}

// OnroadHud
OnroadHud::OnroadHud(QWidget *parent) : QWidget(parent) {
  engage_img = QPixmap("../assets/img_chffr_wheel.png").scaled(img_size, img_size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  experimental_img = loadPixmap("../assets/img_experimental.svg", {img_size - 5, img_size - 5});
  //dm_img = QPixmap("../assets/img_driver_face.png").scaled(img_size, img_size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  traffic_green_img = QPixmap("../assets/img_traffic_green.png");
  traffic_red_img = QPixmap("../assets/img_traffic_red.png");
  connect(this, &OnroadHud::valueChanged, [=] { update(); });
}

void OnroadHud::updateState(const UIState &s) {	
  const SubMaster &sm = *(s.sm);
  const auto cs = sm["controlsState"].getControlsState();
  const auto lo = sm["longitudinalPlan"].getLongitudinalPlan();
	
  setProperty("status", s.status);
  setProperty("ang_str", s.scene.angleSteers);
  setProperty("traffic_state", lo.getTrafficState());
	
  // update engageability and DM icons at 2Hz
  if (sm.frame % (UI_FREQ / 2) == 0) {
    setProperty("engageable", cs.getEngageable() || cs.getEnabled());
    //setProperty("dmActive", sm["driverMonitoringState"].getDriverMonitoringState().getIsActiveMode());
    setProperty("compass", s.scene.compass);
    setProperty("bearingDeg", sm["gpsLocationExternal"].getGpsLocationExternal().getBearingDeg());
    setProperty("bearingAccuracyDeg", sm["gpsLocationExternal"].getGpsLocationExternal().getBearingAccuracyDeg());
    //dp
    const auto lp = sm["longitudinalPlan"].getLongitudinalPlan();
    const auto vtcState = lp.getVisionTurnControllerState();
    const float vtc_speed = lp.getVisionTurnSpeed() * (s.scene.is_metric ? MS_TO_KPH : MS_TO_MPH);
    const auto lpSoruce = lp.getLongitudinalPlanSource();
    QColor vtc_color = tcs_colors[int(vtcState)];
    vtc_color.setAlpha(lpSoruce == cereal::LongitudinalPlan::LongitudinalPlanSource::TURN ? 255 : 100);

    setProperty("showVTC", vtcState > cereal::LongitudinalPlan::VisionTurnControllerState::DISABLED);
    setProperty("vtcSpeed", QString::number(std::nearbyint(vtc_speed)));
    setProperty("vtcColor", vtc_color);
  }
  if(uiState()->recording) {
    update();
  }
}

void OnroadHud::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);
	
  // Header gradient
  QLinearGradient bg(0, header_h - (header_h / 2.5), 0, header_h);
  bg.setColorAt(0, QColor::fromRgbF(0, 0, 0, 0.45));
  bg.setColorAt(1, QColor::fromRgbF(0, 0, 0, 0));
  p.fillRect(0, 0, width(), header_h, bg);
	
  // engage-ability icon
  if (showVTC) {
      drawVisionTurnControllerUI(p, rect().right() - 184 - bdr_s, bdr_s, 184, vtcColor, vtcSpeed, 100);
  } else if (true) {
	SubMaster &sm = *(uiState()->sm);
    drawIcon(p, rect().right() - radius / 2 - bdr_s * 2, radius / 2 + bdr_s,
             sm["controlsState"].getControlsState().getExperimentalMode() ? experimental_img : engage_img, bg_colors[status], 5.0, true, ang_str );
  }

  if (traffic_state >= 0) {
    int w = 200;
    int h = 80;
    int x = (width() + (bdr_s * 2)) / 2 - 400;
    int y = 30 - bdr_s * 3 + 60;
    if (traffic_state == 1) {
      p.drawPixmap(x, y, w, h, traffic_red_img);
    } else if (traffic_state == 2) {
      p.drawPixmap(x, y, w, h, traffic_green_img);
    }
  }
  drawAnimText(p, uiState());
}

void OnroadHud::drawCenteredText(QPainter &p, int x, int y, const QString &text, QColor color) {
  QFontMetrics fm(p.font());
  QRect init_rect = fm.boundingRect(text);
  QRect real_rect = fm.boundingRect(init_rect, 0, text);
  real_rect.moveCenter({x, y});

  p.setPen(color);
  p.drawText(real_rect, Qt::AlignCenter, text);
}

void OnroadHud::drawVisionTurnControllerUI(QPainter &p, int x, int y, int size, const QColor &color,
                                           const QString &vision_speed, int alpha) {
  QRect rvtc(x, y, size, size);
  p.setPen(QPen(color, 10));
  p.setBrush(QColor(0, 0, 0, alpha));
  p.drawRoundedRect(rvtc, 20, 20);
  p.setPen(Qt::NoPen);

  configFont(p, FONT_OPEN_SANS, 56, "SemiBold");
  drawCenteredText(p, rvtc.center().x(), rvtc.center().y(), vision_speed, color);
}

void NvgWindow::drawTextWithColor(QPainter &p, int x, int y, const QString &text, const QColor &color) {
  QFontMetrics fm(p.font());
  QRect init_rect = fm.boundingRect(text);
  QRect real_rect = fm.boundingRect(init_rect, 0, text);
  real_rect.moveCenter({x, y - real_rect.height() / 2});

  p.setPen(color);
  p.drawText(real_rect.x(), real_rect.bottom(), text);
}

void OnroadHud::drawIcon(QPainter &p, int x, int y, QPixmap &img, QBrush bg, float opacity, bool rotation, float angle) {
  // 
  if (rotation) {
    p.setPen(Qt::NoPen);
    p.setBrush(bg);
    p.drawEllipse(x - radius / 2, y - radius / 2, radius, radius);
    p.setOpacity(opacity);
    p.save();
    p.translate(x, y);
    p.rotate(-angle);
    QRect r = img.rect();
    r.moveCenter(QPoint(0,0));
    p.drawPixmap(r, img);
    p.restore();
  } else {
    p.setPen(Qt::NoPen);
    p.setBrush(bg);
    p.drawEllipse(x - radius / 2, y - radius / 2, radius, radius);
    p.setOpacity(opacity);
    p.drawPixmap(x - img.size().width() / 2, y - img.size().height() / 2, img);
  }
}

// NvgWindow

NvgWindow::NvgWindow(VisionStreamType type, QWidget* parent) : last_update_params(0), fps_filter(UI_FREQ, 3, 1. / UI_FREQ), CameraViewWidget("camerad", type, true, parent) {
  leadPulseTimer.start();
}

void NvgWindow::initializeGL() {
  CameraViewWidget::initializeGL();
  qInfo() << "OpenGL version:" << QString((const char*)glGetString(GL_VERSION));
  qInfo() << "OpenGL vendor:" << QString((const char*)glGetString(GL_VENDOR));
  qInfo() << "OpenGL renderer:" << QString((const char*)glGetString(GL_RENDERER));
  qInfo() << "OpenGL language version:" << QString((const char*)glGetString(GL_SHADING_LANGUAGE_VERSION));

  prev_draw_t = millis_since_boot();
  setBackgroundColor(bg_colors[STATUS_DISENGAGED]);

  //neokii
  ic_brake = QPixmap("../assets/images/img_brake_disc.png");
  ic_autohold_warning = QPixmap("../assets/images/img_autohold_warning.png").scaled(img_size, img_size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  ic_autohold_active = QPixmap("../assets/images/img_autohold_active.png").scaled(img_size, img_size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  ic_nda = QPixmap("../assets/images/img_nda.png");
  ic_hda = QPixmap("../assets/images/img_hda.png");
  //ic_tire_pressure = QPixmap("../assets/images/img_tire_pressure.png");
  ic_satellite = QPixmap("../assets/images/satellite.png");
  ic_scc2 = QPixmap("../assets/images/img_scc2.png");
  ic_radar = QPixmap("../assets/images/radar.png");
  ic_radar_vision = QPixmap("../assets/images/radar_vision.png");
  ic_safety_speed_bump = QPixmap("../assets/images/safety_speed_bump.png");
}

void NvgWindow::updateFrameMat(int w, int h) {
  CameraViewWidget::updateFrameMat(w, h);

  UIState *s = uiState();
  s->fb_w = w;
  s->fb_h = h;
  auto intrinsic_matrix = s->wide_camera ? ecam_intrinsic_matrix : fcam_intrinsic_matrix;
  float zoom = ZOOM / intrinsic_matrix.v[0];
  if (s->wide_camera) {
    zoom *= 0.5;
  }
  // Apply transformation such that video pixel coordinates match video
  // 1) Put (0, 0) in the middle of the video
  // 2) Apply same scaling as video
  // 3) Put (0, 0) in top left corner of video
  s->car_space_transform.reset();
  s->car_space_transform.translate(w / 2, h / 2 + y_offset)
      .scale(zoom, zoom)
      .translate(-intrinsic_matrix.v[2], -intrinsic_matrix.v[5]);
}

void NvgWindow::ui_draw_line(QPainter &painter, const line_vertices_data &vd) 
{
  if (vd.cnt == 0) return;
 
  QPainterPath path = QPainterPath();

  const QPointF *v = &vd.v[0];
  path.moveTo( v[0].x(), v[0].y() );
  for (int i = 1; i < vd.cnt; i++) {
    path.lineTo( v[i].x(), v[i].y());
  }
  painter.drawPath( path );
}

void NvgWindow::drawLaneLines(QPainter &painter, const UIState *s) {
  painter.save();

  const UIScene &scene = s->scene;
  SubMaster &sm = *(s->sm);
  int steerOverride = (*s->sm)["carState"].getCarState().getSteeringPressed();

  // paint blindspot line
  painter.setBrush(QColor(255, 94, 0, 150));

  if( scene.leftblindspot  )
  {
       ui_draw_line(  painter, scene.lane_blindspot_vertices[0] );
  }

  if( scene.rightblindspot  )
  {
   //  if( right_cnt > 1 )
        ui_draw_line( painter, scene.lane_blindspot_vertices[1] );
  }
	
  // lanelines
  for (int i = 0; i < std::size(scene.lane_line_vertices); ++i) {
    painter.setBrush(QColor::fromRgbF(1.0, 1.0, 1.0, std::clamp<float>(scene.lane_line_probs[i], 0.0, 0.7)));
    ui_draw_line( painter, scene.lane_line_vertices[i] );
  }
	
  // road edges
  for (int i = 0; i < std::size(scene.road_edge_vertices); ++i) {
    painter.setBrush(QColor::fromRgbF(1.0, 0, 0, std::clamp<float>(1.0 - scene.road_edge_stds[i], 0.0, 1.0)));

    ui_draw_line( painter, scene.road_edge_vertices[i] );
    //painter.drawPolygon(scene.road_edge_vertices[i].v, scene.road_edge_vertices[i].cnt);
  }
	
  // paint path
  QLinearGradient bg(0, height(), 0, height() / 4);
  float start_hue, end_hue;
  if (sm["controlsState"].getControlsState().getExperimentalMode()) {
    const auto &acceleration = sm["modelV2"].getModelV2().getAcceleration();
    float acceleration_future = 0;
    if (acceleration.getZ().size() > 16) {
      acceleration_future = acceleration.getX()[16];  // 2.5 seconds
    }
    start_hue = 60;
    // speed up: 120, slow down: 0
    end_hue = fmax(fmin(start_hue + acceleration_future * 45, 148), 0);

    // FIXME: painter.drawPolygon can be slow if hue is not rounded
    end_hue = int(end_hue * 100 + 0.5) / 100;

    bg.setColorAt(0.0, QColor::fromHslF(start_hue / 360., 0.97, 0.56, 0.4));
    bg.setColorAt(0.5, QColor::fromHslF(end_hue / 360., 1.0, 0.68, 0.35));
    bg.setColorAt(1.0, QColor::fromHslF(end_hue / 360., 1.0, 0.68, 0.0));
  } else {
    bg.setColorAt(0.0, QColor::fromHslF(148 / 360., 0.94, 0.51, 0.4));
    bg.setColorAt(0.5, QColor::fromHslF(112 / 360., 1.0, 0.68, 0.35));
    bg.setColorAt(1.0, QColor::fromHslF(112 / 360., 1.0, 0.68, 0.0));
  }
	  
  painter.setBrush(bg);
  ui_draw_line( painter, scene.track_vertices );
	
  painter.restore();
}

void NvgWindow::drawLead(QPainter &painter,
                         const cereal::RadarState::LeadData::Reader &lead_data,
                         const QPointF &vd, int num) {
  painter.save();
  painter.setRenderHint(QPainter::Antialiasing);

  const float speedBuff = 10.f;
  const float leadBuff  = 40.f;
  const float d_rel = lead_data.getDRel();
  const float v_rel = lead_data.getVRel();

  float fillAlpha = 0.f;
  if (d_rel < leadBuff) {
    fillAlpha = 255.f * (1.0f - (d_rel / leadBuff));
    if (v_rel < 0.f) {
      fillAlpha += 255.f * (-v_rel / speedBuff);
    }
    fillAlpha = std::clamp(fillAlpha, 0.f, 255.f);
  }

  float sz = std::clamp((25.f * 30.f) / (d_rel / 3.f + 30.f), 15.0f, 30.0f) * 2.35f;

  // ====== 튜닝 스케일 (요청: 1.15) ======
  const float LEAD_SCALE  = 1.15f;  // 원 + R/V
  const float LABEL_SCALE = 1.15f;  // 거리 라벨 + 숫자
  // =====================================

  const float circleScale = 1.20f;        // 1.10~1.35 취향
  sz *= (circleScale * LEAD_SCALE);

  float x = std::clamp((float)vd.x(), 0.f, width() - sz / 2.f);
  float y = std::fmin(height() - sz * 0.6f, (float)vd.y());

  const bool is_radar = uiState()->scene.lead_radar[num];
  QColor circleColor = is_radar ? QColor(255, 0, 255) : QColor(0, 160, 255);

  const float t = leadPulseTimer.elapsed() * 0.001f;
  const float pulse_speed = 2.6f;          // 속도(클수록 빠름)
  const float pulse = std::sin(t * pulse_speed);

  float pulse_strength = std::clamp(1.0f - (d_rel / 45.f), 0.f, 1.f);
  if (v_rel < -1.0f) {
    pulse_strength = std::min(1.0f, pulse_strength * (1.0f + std::clamp((-v_rel) / 8.f, 0.f, 0.6f)));
  }

  const float pulse_scale = 1.0f + pulse * 0.06f * pulse_strength;   // 크기 호흡
  const float pulse_alpha = pulse * 28.f * pulse_strength;           // 알파 호흡

  const float r_base = sz * 0.85f;
  const float r = r_base * pulse_scale;
  QRectF circleRect(x - r, y - r, r * 2.f, r * 2.f);

  int a = (int)(fillAlpha + pulse_alpha);
  a = std::clamp(a, 110, 255);
  circleColor.setAlpha(a);

  painter.setPen(QPen(QColor(0, 0, 0, 160), 3));
  painter.setBrush(circleColor);
  painter.drawEllipse(circleRect);

  // ===== R/V 글자(원과 같이) 크게 =====
  const QString rv_txt = is_radar ? "R" : "V";
  const int rv_font_px = std::clamp((int)(r * 0.9f * LEAD_SCALE), 28, 58);
  configFont(painter, FONT_OPEN_SANS, rv_font_px, "ExtraBold");

  painter.setPen(QColor(0, 0, 0, 200));
  painter.drawText(circleRect.translated(2, 2), Qt::AlignCenter, rv_txt);

  painter.setPen(QColor(255, 255, 255, 255));
  painter.drawText(circleRect, Qt::AlignCenter, rv_txt);

  // ===== 거리 라벨: 박스+숫자 확대 + 흰 테두리 =====
  const int dist_i = (int)std::nearbyint(d_rel);
  const QString dist_txt = QString::number(dist_i);

  const int label_font_px = std::clamp((int)(r * 0.55f * LABEL_SCALE), 22, 40);
  configFont(painter, FONT_OPEN_SANS, label_font_px, "ExtraBold");

  QFontMetrics fm(painter.font());
  const int text_w = fm.horizontalAdvance(dist_txt);
  const int text_h = fm.height();

  const float pad_x = std::clamp(r * 0.25f * LABEL_SCALE, 12.f, 20.f);
  const float pad_y = std::clamp(r * 0.15f * LABEL_SCALE,  8.f, 14.f);

  const float label_w = text_w + pad_x * 2.f;
  const float label_h = text_h + pad_y * 2.f;

  const float gap = std::clamp(r * 0.20f * LABEL_SCALE, 9.f, 16.f);

  QRectF labelRect(circleRect.center().x() - label_w * 0.5f,
                   circleRect.top() - label_h - gap,
                   label_w, label_h);

  if (labelRect.top() < 0.f) labelRect.moveTop(0.f);
  if (labelRect.left() < 0.f) labelRect.moveLeft(0.f);
  if (labelRect.right() > width()) labelRect.moveRight(width());

  const float radius = std::clamp(label_h * 0.35f, 7.f, 16.f);

  int bgA = (int)(145 + pulse_alpha * 0.8f);
  bgA = std::clamp(bgA, 120, 205);

  // 흰색 테두리 요청
  painter.setPen(QPen(QColor(255, 255, 255, 220), 2));
  painter.setBrush(QColor(0, 0, 0, bgA));
  painter.drawRoundedRect(labelRect, radius, radius);

  painter.setPen(QColor(0, 0, 0, 220));
  painter.drawText(labelRect.translated(2, 2), Qt::AlignCenter, dist_txt);

  painter.setPen(QColor(255, 255, 255, 255));
  painter.drawText(labelRect, Qt::AlignCenter, dist_txt);

  painter.restore();
}

void NvgWindow::paintGL() {
  CameraViewWidget::paintGL();

  UIState *s = uiState();
  if (s->worldObjectsVisible()) { 
    if(!s->recording) {
      QPainter p(this);
      drawCommunity(p);
    }
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setPen(Qt::NoPen);

    drawLaneLines(painter, s);
  }

  double cur_draw_t = millis_since_boot();
  double dt = cur_draw_t - prev_draw_t;
  double fps = fps_filter.update(1. / dt * 1000);
  if (fps < 15) {
    LOGW("slow frame rate: %.2f fps", fps);
  }
  prev_draw_t = cur_draw_t;
}

void NvgWindow::showEvent(QShowEvent *event) {
  CameraViewWidget::showEvent(event);

  auto now = millis_since_boot();
  if(now - last_update_params > 1000) {
    last_update_params = now;
    ui_update_params(uiState());
  }
  
  prev_draw_t = millis_since_boot();
}


void NvgWindow::drawCommunity(QPainter &p) {
  p.save();

  // Header gradient
  QLinearGradient bg(0, header_h - (header_h / 2.5), 0, header_h);
  bg.setColorAt(0, QColor::fromRgbF(0, 0, 0, 0.45));
  bg.setColorAt(1, QColor::fromRgbF(0, 0, 0, 0));
  p.fillRect(0, 0, width(), header_h, bg);

  UIState *s = uiState();
  SubMaster &sm = *(s->sm);

  if (!sm.alive("lateralPlan") || !sm.alive("longitudinalPlan") || !sm.alive("liveParameters") || !sm.alive("roadLimitSpeed") || !sm.alive("liveTorqueParameters")) {
      return;
  }
  const double start_draw_t = millis_since_boot();
  const cereal::ModelDataV2::Reader &model = sm["modelV2"].getModelV2();
  const cereal::RadarState::Reader &radar_state = sm["radarState"].getRadarState();
	
  const auto leads = model.getLeadsV3();
  size_t leads_num = leads.size();
  auto lead_one = radar_state.getLeadOne();
  auto lead_two = radar_state.getLeadTwo();

  if (lead_one.getStatus()) {
    drawLead(p, lead_one, s->scene.lead_vertices[0], 0);
  }
  if (lead_two.getStatus() && (!lead_one.getStatus() || std::abs(lead_one.getDRel() - lead_two.getDRel()) > 3.0)) {
    drawLead(p, lead_two, s->scene.lead_vertices[1], 1);
  }
	
  drawGpsStatus(p);
  drawBrake(p);
  drawMisc(p);
  drawLeftStatusPanel(p);
	
  if(s->show_steer)
    drawSteer(p);	
	
  if(s->show_engrpm)
    drawEngRpm(p);
	
  if(s->show_tpms && width() > 1200)
    drawTpms(p);
  	
  char str[128];	
  const auto car_state = sm["carState"].getCarState();
  const auto controls_state = sm["controlsState"].getControlsState();
  const auto car_params = sm["carParams"].getCarParams();
  const auto live_params = sm["liveParameters"].getLiveParameters();
  const auto device_state = sm["deviceState"].getDeviceState();
  const auto live_torque_params = sm["liveTorqueParameters"].getLiveTorqueParameters();
  const auto torque_state = controls_state.getLateralControlState().getTorqueState();
  float distance_traveled = sm["controlsState"].getControlsState().getDistanceTraveled() / 1000;
  	
  	
  int lateralControlState = controls_state.getLateralControlSelect();
  const char* lateral_state[] = {"PID", "INDI", "LQR", "TORQUE" };
	
  auto cpuList = device_state.getCpuTempC();
  float cpuTemp = 0;

  if (cpuList.size() > 0) {
      for(int i = 0; i < cpuList.size(); i++)
          cpuTemp += cpuList[i];
      cpuTemp /= cpuList.size();
  }

  auto cpu_loads = device_state.getCpuUsagePercent();
  int cpu_usage = std::accumulate(cpu_loads.begin(), cpu_loads.end(), 0) / cpu_loads.size();
	
  //int mdps_bus = car_params.getMdpsBus();
  int scc_bus = car_params.getSccBus();

  QString infoText;
  infoText.sprintf("TP(%.2f/%.2f)LTP(%.2f/%.2f/%.0f)SR(%.2f)SAD(%.2f)온도(%.0f°C)주행(%.1f km)SCC(%d)",
	              torque_state.getLatAccelFactor(),
                      torque_state.getFriction(),

                      live_torque_params.getLatAccelFactorRaw(),
                      live_torque_params.getFrictionCoefficientRaw(),
                      live_torque_params.getTotalBucketPoints(),
                      controls_state.getSteerRatio(),
                      controls_state.getSteerActuatorDelay(),
		      cpuTemp,
		      controls_state.getDistanceTraveled() / 1000,
                      scc_bus
                      );

  // info
  configFont(p, "Open Sans", 35, "Bold");
  p.setPen(QColor(0xff, 0xff, 0xff, 0xff));
  p.drawText(rect().left() + 180, rect().height() - 15, infoText);	
  const int h = 60;
  QRect bar_rc(rect().left(), rect().bottom() - h, rect().width(), h);
  p.setBrush(QColor(0, 0, 0, 20));
  p.drawRect(bar_rc);
  drawBottomIcons(p);
	
  p.setOpacity(1.);
}

static const QColor get_tpms_color(float tpms) {
    if(tpms < 5 || tpms > 60) // N/A
        return QColor(255, 255, 255, 220);
    if(tpms < 31)
        return QColor(255, 90, 90, 220);
    return QColor(255, 255, 255, 220);
}

static const QString get_tpms_text(float tpms) {
    if(tpms < 5 || tpms > 60)
        return "";

    char str[32];
    snprintf(str, sizeof(str), "%.0f", round(tpms));
    return QString(str);
}

void NvgWindow::drawIcon(QPainter &p, int x, int y, QPixmap &img, QBrush bg, float opacity) {
  p.setPen(Qt::NoPen);
  p.setBrush(bg);
  p.drawEllipse(x - radius / 2, y - radius / 2, radius, radius);
  p.setOpacity(opacity);
  p.drawPixmap(x - img_size / 2, y - img_size / 2, img_size, img_size, img);
}

void NvgWindow::drawText2(QPainter &p, int x, int y, int flags, const QString &text, const QColor& color) {
  QFontMetrics fm(p.font());
  QRect rect = fm.boundingRect(text);
  rect.adjust(-1, -1, 1, 1);
  p.setPen(color);
  p.drawText(QRect(x, y, rect.width()+1, rect.height()), flags, text);
}

void NvgWindow::drawText(QPainter &p, int x, int y, const QString &text, int alpha) {
  QFontMetrics fm(p.font());
  QRect init_rect = fm.boundingRect(text);
  QRect real_rect = fm.boundingRect(init_rect, 0, text);
  real_rect.moveCenter({x, y - real_rect.height() / 2});

  p.setPen(QColor(0xff, 0xff, 0xff, alpha));
  p.drawText(real_rect.x(), real_rect.bottom(), text);
}

void NvgWindow::drawBottomIcons(QPainter &p) {
  UIState *s = uiState();
  const SubMaster &sm = *(s->sm);

  const auto car_state = sm["carState"].getCarState();
  const auto controls_state = sm["controlsState"].getControlsState();
  const auto lp = sm["longitudinalPlan"].getLongitudinalPlan();
  const auto scc_smoother = sm["carControl"].getCarControl().getSccSmoother();

  int x = radius / 2 + (bdr_s * 2) + (radius + 50);
  const int y = rect().bottom() - footer_h / 2 - 10;
	
  // Accel표시
  float accel = car_state.getAEgo();  
  float dx = 138 + 1330;
#ifdef __TEST  
  static float accel1 = 0.0;
  accel1 += 0.2;
  if (accel1 > 2.5) accel1 = -2.5;
  accel = accel1;
#endif	
  QRect rectAccelPos(x + dx, y - 375, 35, -std::clamp((float)accel, -2.0f, 2.0f) / 2. * 550);
  p.setBrush((accel>=0.0)?greenColor(255):redColor(255));
  p.drawRect(rectAccelPos);
	
 if (s->show_datetime && width() > 1200) {
     // ajouatom: 현재시간표시
     QTextOption  textOpt = QTextOption(Qt::AlignLeft);
     configFont(p, "Open Sans", 65, "Bold");
     p.drawText(QRect(270, 30, width(), 70), QDateTime::currentDateTime().toString("hh:mm"), textOpt);
     configFont(p, "Open Sans", 60, "Bold");
     p.drawText(QRect(270, 150, width(), 70), QDateTime::currentDateTime().toString("MM-dd(ddd)"), textOpt);
  }
	
  p.setOpacity(1.);
}

void NvgWindow::drawLeftStatusPanel(QPainter &p) {
  UIState *s = uiState();
  const SubMaster &sm = *(s->sm);

  const auto car_state = sm["carState"].getCarState();
  const auto cs = sm["controlsState"].getControlsState();
  const auto lp = sm["longitudinalPlan"].getLongitudinalPlan();
  const auto device = sm["deviceState"].getDeviceState();

  // ===== 위치 / 크기 =====
  const int x = 40;
  const int y = height() - 560;
  const int w = 420;
  const int h = 520;

  p.save();
  p.setRenderHint(QPainter::Antialiasing);

  // ===== 배경 카드 =====
  p.setPen(QPen(QColor(255, 255, 255, 50), 2));
  p.setBrush(QColor(0, 0, 0, 170));
  p.drawRoundedRect(x, y, w, h, 28, 28);

  // ===== 상단 박스 공통 =====
  auto drawTopBox = [&](int bx, const QString &txt, QColor c) {
    QRect r(bx, y + 18, 110, 48);
    p.setPen(Qt::NoPen);
    p.setBrush(c);
    p.drawRoundedRect(r, 12, 12);
    configFont(p, "Inter", 20, "Bold");
    p.setPen(Qt::black);
    p.drawText(r, Qt::AlignCenter, txt);
  };

  // ===== CPU 온도 =====
  int cpuTemp = 0;
  for (auto t : device.getCpuTempC()) cpuTemp += t;
  if (device.getCpuTempC().size()) cpuTemp /= device.getCpuTempC().size();

  // ===== ACC / E2E 텍스트 =====
  QString acc_txt = "ACC";
  if (cs.getEnabled()) {
    if (lp.getLongitudinalPlanSource() ==
        cereal::LongitudinalPlan::LongitudinalPlanSource::E2E) {
      acc_txt = "E2E";
    }
  }

  // ===== SoftHold: 작동 시 깜빡임 =====
  bool soft_hold = calc_soft_hold_active(cs, car_state);
  const int blink_period_ms = 350;
  const int64_t ms = (int64_t)millis_since_boot();
  const bool blink_on = ((ms / blink_period_ms) % 2) == 0;

  QColor soft_col = soft_hold ? QColor(255, 80, 80, blink_on ? 255 : 80)
                              : QColor(80, 80, 80, 200);
  QString soft_txt = soft_hold ? "SOFT HOLD" : "HOLD OFF";

  drawTopBox(x + 20,  QString("CPU %1°").arg(cpuTemp), QColor(0, 200, 0));
  drawTopBox(x + 155, acc_txt, QColor(0, 160, 255, 230));
  drawTopBox(x + 290, soft_txt, soft_col);

  // ===== 현재속도 / 목표속도 =====
  float cur_speed = std::max(0.0f,
  car_state.getCluSpeedMs() * (s->scene.is_metric ? MS_TO_KPH : MS_TO_MPH));
  if (cur_speed < 0) cur_speed = 0;

  float set_speed = cs.getVCruise() * (s->scene.is_metric ? 1.0 : KM_TO_MILE);

  configFont(p, "Inter", 110, "Bold");
  p.setPen(Qt::white);
  p.drawText(QRect(x + 40, y + 100, 220, 150),
             Qt::AlignLeft | Qt::AlignVCenter,
             QString::number((int)std::nearbyint(cur_speed)));

  configFont(p, "Inter", 48, "Bold");
  p.setPen(QColor(0, 255, 0));
  p.drawText(QRect(x + 220, y + 140, 160, 80),
             Qt::AlignLeft | Qt::AlignVCenter,
             QString::number((int)std::nearbyint(set_speed)));

  // 곡선 강조선(기존 유지)
  QPainterPath path;
  path.moveTo(x + 40, y + 250);
  path.cubicTo(x + 160, y + 200, x + 260, y + 320, x + 360, y + 260);
  p.setPen(QPen(QColor(0, 255, 0, 180), 4));
  p.drawPath(path);

  // ===== 기어 =====
  QString gear = "D";
  switch (car_state.getGearShifter()) {
    case cereal::CarState::GearShifter::PARK:    gear = "P"; break;
    case cereal::CarState::GearShifter::REVERSE: gear = "R"; break;
    case cereal::CarState::GearShifter::NEUTRAL: gear = "N"; break;
    default: break;
  }

  int cur_gear = (int)std::nearbyint(car_state.getCurrentGear());
  bool show_gear_num = (gear == "D") && (cur_gear >= 1 && cur_gear <= 8);

  QRect gr(x + w - 95, y + 155, 70, 110);
  p.setPen(QPen(Qt::white, 2));
  p.setBrush(QColor(0, 0, 0, 150));
  p.drawRoundedRect(gr, 12, 12);

  if (show_gear_num) {
    // D일 때 숫자만 표시
    configFont(p, "Inter", 52, "Black");
    p.setPen(QColor(0, 255, 0, 240));
    p.drawText(gr, Qt::AlignCenter, QString::number(cur_gear));
  }
  else {
    configFont(p, "Inter", 46, "Bold");
    p.setPen(Qt::white);
    p.drawText(gr, Qt::AlignCenter, gear);
  }

  // ===== Driving Mode =====
  QString mode = "일반";
  switch (cs.getMyDrivingMode()) {
    case 1: mode = "연비"; break;
    case 2: mode = "안전"; break;
    case 3: mode = "일반"; break;
    case 4: mode = "고속"; break;
    default: break;
  }

  configFont(p, "Inter", 28, "Bold");
  p.setPen(QColor(255, 255, 255, 230));
  p.drawText(QRect(x + 30, y + h - 140, 160, 50),
             Qt::AlignCenter, mode);

  // =============================================================================
  // [반영] nanovg 로직: limit_speed/left_dist/roadLimitSpeed (턴인포 제거)
  // cam > section > road 우선
  // =============================================================================
  int limit_speed = 0;
  int left_dist = 0;

  int roadLimitSpeed = 0;
  int camLimitSpeed = 0;
  int camLeftDist = 0;
  int sectionLimitSpeed = 0;
  int sectionLeftDist = 0;

  int camType = 0;
  int xSignType = 0;   // 포크에 필드 없으면 이 변수/대입 줄 삭제

  if (sm.alive("roadLimitSpeed")) {
    const auto rls = sm["roadLimitSpeed"].getRoadLimitSpeed();

    roadLimitSpeed = rls.getRoadLimitSpeed();
    camLimitSpeed = rls.getCamLimitSpeed();
    camLeftDist = rls.getCamLimitSpeedLeftDist();
    sectionLimitSpeed = rls.getSectionLimitSpeed();
    sectionLeftDist = rls.getSectionLeftDist();

    // camType은 drawMaxSpeed에서도 쓰던 필드라 대부분 존재
    camType = rls.getCamType();

    // 포크에 없으면 아래 1줄 삭제
    //if (rls.hasXSignType()) xSignType = rls.getXSignType();
  }

  if (camLimitSpeed > 0 && camLeftDist > 0) {
    limit_speed = camLimitSpeed;
    left_dist = camLeftDist;
  } else if (sectionLimitSpeed > 0 && sectionLeftDist > 0) {
    limit_speed = sectionLimitSpeed;
    left_dist = sectionLeftDist;
  }

  // 패널 내부 표지판 위치(원 중심)
  int bx = x + 260;
  int by = y + h - 115;

  // --- left_dist 텍스트 (있으면 항상 표시) ---
  if (left_dist > 0) {
    QString dist_txt;
    if (left_dist < 1000) dist_txt = QString("%1 m").arg(left_dist);
    else dist_txt = QString::asprintf("%.1f km", left_dist / 1000.f);

    configFont(p, "Inter", 30, "Bold");
    p.setPen(QColor(255,255,255,230));
    QRect dr(x + 200, y + h - 70, 200, 40);
    p.drawText(dr, Qt::AlignLeft | Qt::AlignVCenter, dist_txt);
  }

  // --- limit_speed 표시 (있으면 우선) ---
  if (limit_speed > 0) {
    // speed bump (xSignType==124 or camType==22)
    if (camType == 22) {
      QRect ir(bx - 40, by - 45, 80, 90);

      // 이미 가지고 있는 ic_safety_speed_bump 쓰고 싶으면 ic_speed_bump 대신 그걸로 교체 가능
      if (!ic_safety_speed_bump.isNull()) {
        p.drawPixmap(ir, ic_safety_speed_bump);
      } else {
        p.setPen(Qt::NoPen);
        p.setBrush(QColor(255,255,255,200));
        p.drawRoundedRect(ir, 14, 14);
        configFont(p, "Inter", 18, "Bold");
        p.setPen(Qt::black);
        p.drawText(ir, Qt::AlignCenter, "BUMP");
      }
    } else {
      // 원형 속도 표지(흰-빨-흰) + 숫자
      p.setPen(Qt::NoPen);

      p.setBrush(QColor(255,255,255,255));
      p.drawEllipse(QPoint(bx, by), 70, 70);

      p.setBrush(QColor(255,0,0,255));
      p.drawEllipse(QPoint(bx, by), 65, 65);

      p.setBrush(QColor(255,255,255,255));
      p.drawEllipse(QPoint(bx, by), 55, 55);

      configFont(p, "Inter", 46, "Black");
      p.setPen(Qt::black);
      p.drawText(QRect(bx - 60, by - 30, 120, 70),
                 Qt::AlignCenter, QString::number(limit_speed));
    }
  }
  // --- limit_speed 없으면 roadLimitSpeed(도로 제한) ---
  else if (roadLimitSpeed > 0 && roadLimitSpeed < 200) {
    // 아이콘 없으면 박스로 fallback
    QRect rr(bx - 70, by - 50, 140, 100);
    p.setPen(QPen(Qt::white, 2));
    p.setBrush(QColor(0,0,0,150));
    p.drawRoundedRect(rr, 16, 16);

    configFont(p, "Inter", 20, "Bold");
    p.setPen(QColor(255,255,255,230));
    p.drawText(QRect(rr.x(), rr.y()+10, rr.width(), 30), Qt::AlignCenter, "ROAD");

    configFont(p, "Inter", 36, "Black");
    p.drawText(QRect(rr.x(), rr.y()+38, rr.width(), 50),
               Qt::AlignCenter, QString::number(roadLimitSpeed));
  }

  // ===== GAP 표시 (위 나노VG 스타일 동일 반영: 세로바+4칸+채움+"GAP"+숫자) =====
  float gap_f = lp.getCruiseGap();                         // 채움 높이용 (0~4 float)
  int gap1 = std::clamp((int)std::nearbyint(lp.getCruiseGap()), 0, 4);

  gap_f = std::clamp(gap_f, 0.0f, 4.0f);
  gap1  = std::clamp(gap1, 0, 4);

  // 기존 점 위치 기준 (필요하면 좌표만 살짝 조정)
  const int bar_x   = x + 340;
  const int bar_top = y + h - 140;

  const int bar_w = 40;
  const int bar_h = 64;
  const int seg_h = bar_h / 4;

  // 1) 채움(아래에서 위로 올라가는 초록바)  --- nanovg ui_fill_rect와 동일한 개념
  int fill_h = (int)(gap_f / 4.0f * (float)bar_h);

  p.setPen(Qt::NoPen);
  p.setBrush(QColor(0, 255, 0, 255));
  p.drawRect(QRect(bar_x + 2, bar_top + bar_h, bar_w - 4, -fill_h));

  // 2) 4칸 테두리 (nanovg ui_draw_rect 4번과 동일)
  p.setBrush(Qt::NoBrush);
  p.setPen(QPen(QColor(255,255,255,255), 4));

  for (int i = 0; i < 4; i++) {
    p.drawRoundedRect(QRect(bar_x, bar_top + i * seg_h, bar_w, seg_h), 6, 6);
  }

  // 3) "GAP" 라벨 (nanovg ui_draw_text(... "GAP")와 동일 위치감)
  configFont(p, "Inter", 25, "Bold");
  p.setPen(QColor(255,255,255,230));
  p.drawText(QRect(bar_x - 10, bar_top + bar_h + 18, bar_w + 20, 30),
             Qt::AlignCenter, "GAP");

  // 4) 중앙 큰 숫자 표시 (nanovg: ui_draw_text(s, x+dxGap+15+60, y+60, ...))
  configFont(p, "Inter", 50, "Bold");
  p.setPen(QColor(255,255,255,230));
  p.drawText(QRect(bar_x + 60, bar_top - 5, 90, 80),
             Qt::AlignCenter, QString::number(gap1));

  p.restore();
}

void NvgWindow::drawBrake(QPainter &p) {
  const SubMaster &sm = *(uiState()->sm);
  auto car_state = sm["carState"].getCarState();
  bool brake_valid = car_state.getBrakeLights();
	
  int w = 3200;
  int h = 70;
  int x = (width() + (bdr_s*2))/2 - w/2 - bdr_s;
  int y = 40 - bdr_s + 960;
  
  if (brake_valid) {
    p.drawPixmap(x, y, w, h, ic_brake);
    p.setOpacity(1.f);
  }
}
	  
void NvgWindow::drawTpms(QPainter &p) {
  p.save();
	
  UIState *s = uiState();
  const SubMaster &sm = *(uiState()->sm);	
  auto car_state = sm["carState"].getCarState();

  const int w = 58;
  const int h = 126;
  const int x = 110 + 1610;
  const int y = height() - h - 68;

  auto tpms = car_state.getTpms();
  const float fl = tpms.getFl();
  const float fr = tpms.getFr();
  const float rl = tpms.getRl();
  const float rr = tpms.getRr();

  p.setOpacity(0.8);
  p.drawPixmap(x, y, w, h, ic_tire_pressure);

  configFont(p, "Open Sans", 38, "Bold");

  QFontMetrics fm(p.font());
  QRect rcFont = fm.boundingRect("9");

  int center_x = x + 4;
  int center_y = y + h/2;
  const int marginX = (int)(rcFont.width() * 2.7f);
  const int marginY = (int)((h/2 - rcFont.height()) * 0.7f);

  drawText2(p, center_x-marginX-10, center_y-marginY-10-rcFont.height(), Qt::AlignRight, get_tpms_text(fl), get_tpms_color(fl));
  drawText2(p, center_x+marginX+10, center_y-marginY-10-rcFont.height(), Qt::AlignLeft, get_tpms_text(fr), get_tpms_color(fr));
  drawText2(p, center_x-marginX-10, center_y+marginY+10, Qt::AlignRight, get_tpms_text(rl), get_tpms_color(rl));
  drawText2(p, center_x+marginX+10, center_y+marginY+10, Qt::AlignLeft, get_tpms_text(rr), get_tpms_color(rr));

  p.restore();
}

static QRect getRect(QPainter &p, int flags, QString text) {
  QFontMetrics fm(p.font());
  QRect init_rect = fm.boundingRect(text);
  return fm.boundingRect(init_rect, flags, text);
}

void NvgWindow::drawMisc(QPainter &p) {
  p.save();
  UIState *s = uiState();
  const SubMaster &sm = *(s->sm);

  const auto road_limit_speed = sm["roadLimitSpeed"].getRoadLimitSpeed();
  QString currentRoadName = QString::fromStdString(road_limit_speed.getCurrentRoadName().cStr());

  QColor color = QColor(255, 255, 255, 250);

  configFont(p, "Open Sans", 70, "Bold");
  drawText(p, (width()-(bdr_s*2))/4 + bdr_s + 900, 110, currentRoadName, 200);

  p.restore();
}

void NvgWindow::drawSteer(QPainter &p) {
  const SubMaster &sm = *(uiState()->sm);
  auto car_state = sm["carState"].getCarState();
  auto car_control = sm["carControl"].getCarControl();

  float steer_angle = car_state.getSteeringAngleDeg();
  float desire_angle = car_control.getActuators().getSteeringAngleDeg();

  configFont(p, "Open Sans", 50, "Bold");

  QString str;
  
  QRect rc(1660, 260, 184, 130);
  p.setPen(QPen(QColor(0xff, 0xff, 0xff, 100), 10));
  p.setBrush(QColor(0, 0, 0, 100));
  p.drawRoundedRect(rc, 20, 20);
  p.setPen(Qt::NoPen);
	
  QColor textColor0 = QColor(255, 255, 255, 200); // white
  QColor textColor1 = QColor(120, 255, 120, 200); // green
	
  str.sprintf("%.0f°", steer_angle);
  drawTextWithColor(p, rc.center().x(), rc.center().y(), str, textColor0);
	
  str.sprintf("%.0f°", desire_angle);
  drawTextWithColor(p, rc.center().x(), rc.center().y() + 50, str, textColor1);
}

void NvgWindow::drawGpsStatus(QPainter &p) {
  const SubMaster &sm = *(uiState()->sm);
  auto gps = sm["gpsLocationExternal"].getGpsLocationExternal();
  float accuracy = gps.getAccuracy();
  if(accuracy < 0.01f || accuracy > 20.f)
    return;

  int w = 150;
  int h = 62;
  int x = width() - w - 57;
  int y = 690;
  p.setOpacity(1.5);
  p.drawPixmap(x, y, w, h, ic_satellite);

  configFont(p, "Open Sans", 32, "Bold");
  p.setPen(QColor(255, 255, 255, 200));
  p.setRenderHint(QPainter::TextAntialiasing);

  QRect rect = QRect(x, y + h + 10, w, 40);
  rect.adjust(-30, 0, 30, 0);

  QString str;
  str.sprintf("GPS %.1f m", accuracy);
  p.drawText(rect, Qt::AlignHCenter, str);
  p.setOpacity(1.0);
}

void NvgWindow::drawEngRpm(QPainter &p) {
  const SubMaster &sm = *(uiState()->sm);
  auto car_state = sm["carState"].getCarState();

  float eng_rpm = car_state.getEngRpm();
  float textSize = 50;
	
  int x = (width() + (bdr_s*2))/2 - bdr_s;
  int y = bdr_s + 290;

  QString rpm;

  rpm.sprintf("%.0f", eng_rpm);
  configFont(p, "Open Sans", textSize, "Bold");

  QColor textColor0 = QColor(255, 255, 255, 250);
  QColor textColor1 = QColor(120, 255, 120, 250);
  QColor textColor2 = QColor(255, 255, 0, 250);
  QColor textColor3 = QColor(255, 0, 0, 250);

  if (eng_rpm < 1099) {
   drawTextWithColor(p, x, y, rpm, textColor0);
  } else if (eng_rpm < 2300) {
   drawTextWithColor(p, x, y, rpm, textColor1);
  } else if (eng_rpm < 2999) {
   drawTextWithColor(p, x, y, rpm, textColor2);
  } else if (eng_rpm > 3000) {
   drawTextWithColor(p, x, y, rpm, textColor2);
  }
}
