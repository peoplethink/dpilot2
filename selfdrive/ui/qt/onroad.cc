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


// ================== [ADD] NanoVG rect helpers (Qt port) ==================
static void qp_draw_rect(QPainter &p, const QRectF &r, const QColor &color,
                         float stroke_w, float radius) {
  if (stroke_w <= 0.f) return;
  p.save();
  p.setRenderHint(QPainter::Antialiasing);
  p.setBrush(Qt::NoBrush);
  p.setPen(QPen(color, stroke_w));
  if (radius > 0.f) p.drawRoundedRect(r, radius, radius);
  else p.drawRect(r);
  p.restore();
}

static void qp_fill_rect(QPainter &p, const QRectF &r, const QColor *fill_color,
                         float radius, float stroke_w, const QColor *stroke_color) {
  p.save();
  p.setRenderHint(QPainter::Antialiasing);

  // fill
  p.setPen(Qt::NoPen);
  p.setBrush(fill_color ? QBrush(*fill_color) : Qt::NoBrush);
  if (radius > 0.f) p.drawRoundedRect(r, radius, radius);
  else p.drawRect(r);

  // stroke
  if (stroke_w > 0.f) {
    QColor sc = stroke_color ? *stroke_color : QColor(0, 0, 0, 255);  // NanoVG: 없으면 검정
    p.setBrush(Qt::NoBrush);
    p.setPen(QPen(sc, stroke_w));
    if (radius > 0.f) p.drawRoundedRect(r, radius, radius);
    else p.drawRect(r);
  }

  p.restore();
}

static inline float nvg_mix(float center, float target, int time1, int a_max) {
  // NanoVG: (center*time1 + target*(a_max-time1)) / a_max
  return (center * (float)time1 + target * (float)(a_max - time1)) / (float)a_max;
}

// 애니메이션 시작 (paint.h ui_draw_text_a 대응) - 그대로 사용
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

// 매 프레임 그리기 (paint.h ui_draw_text_a2 대응) - "수식 동일"로 교체
static void drawAnimText(QPainter &p, const UIState *s) {
  if (g_anim_text.time <= 0) return;

  g_anim_text.time -= 10;   // paint.h 동일
  int time1 = g_anim_text.time;
  if (time1 > 100) time1 = 100;

  const float cx = (float)s->fb_w / 2.0f;
  const float cy = (float)s->fb_h - 400.0f;

  const float x  = nvg_mix(cx,      g_anim_text.x,    time1, kAnimMax);
  const float y  = nvg_mix(cy,      g_anim_text.y,    time1, kAnimMax);
  const float sz = nvg_mix(350.0f,  g_anim_text.size, time1, kAnimMax);

  if (g_anim_text.time >= 100) {
    drawOutlinedText(p, x, y, g_anim_text.text, (int)sz,
                     g_anim_text.color, g_anim_text.font,
                     QFont::Black,
                     /*borderWidth=*/9.0f,
                     /*shadowOffset=*/8.0f,
                     QColor(0,0,0,255),
                     QColor(0,0,0,255));
  } else {
    // NanoVG else는 "기본 텍스트"라 외곽선 없음에 가깝게
    drawOutlinedText(p, x, y, g_anim_text.text, (int)sz,
                     g_anim_text.color, g_anim_text.font,
                     QFont::Bold,
                     /*borderWidth=*/0.0f,
                     /*shadowOffset=*/0.0f,
                     QColor(0,0,0,255),
                     QColor(0,0,0,255));
  }
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
  const auto device = sm["deviceState"].getDeviceState();

  // ---------- Params helper (Params에는 getInt가 없어서 파싱) ----------
  Params params;
  auto paramsGetInt = [&](const char *key, int def = 0) -> int {
    std::string v = params.get(key);
    if (v.empty()) return def;
    return std::atoi(v.c_str());
  };

  // ---------- Boot 1회 적용: InitMyDrivingMode(1~5) -> MyDrivingMode ----------
  const int boot_mode = paramsGetInt("InitMyDrivingMode", 0);
  if (boot_mode >= 1 && boot_mode <= 5) {
    params.put("MyDrivingMode", std::to_string(boot_mode));
    params.remove("InitMyDrivingMode");
  }

  int myDrivingMode = paramsGetInt("MyDrivingMode", 3);
  if (myDrivingMode < 1 || myDrivingMode > 5) myDrivingMode = 3;

  // ---------- traffic state (longitudinalPlan 기반) ----------
  int trafficState = 0;
  int trafficState_carrot = 0;  // carrot 필드 없으면 0 고정
  float lp_cruise_gap = 0.0f;
  if (sm.alive("longitudinalPlan")) {
    const auto lp = sm["longitudinalPlan"].getLongitudinalPlan();
    trafficState = lp.getTrafficState();
    lp_cruise_gap = lp.getCruiseGap();
  }

  // ---------- Apply/Target (변수 없어서 컴파일 에러 나던 부분: 로컬 기본값) ----------
  QString apply_source;            // 기본: 빈 값 -> 표시 안 함
  float apply_speed = 0.0f;        // 기본 0
  float cruiseTarget = cs.getVCruise();  // 기본: 현재 크루즈와 동일

  // ---------- NanoVG drawHud() 좌표계 그대로 ----------
  const int bx = 140;
  const int by = height() - 230;
  const int panel_x = bx - 120;
  const int panel_y_full = by - 270;
  const int panel_w = 475;
  const int panel_h = 495;
  const int cut_h = 140;

  const int64_t ms = (int64_t)millis_since_boot();
  const int blink_timer = (int)((ms / 80) % 16);

  // ---------- roadLimitSpeed -> CAM 감지 ----------
  int nRoadLimitSpeed_ = 0;   // 멤버 nRoadLimitSpeed 없으면 0 시작
  int camLimitSpeed = 0;
  int camLeftDist = 0;
  int sectionLimitSpeed = 0;
  int sectionLeftDist = 0;
  int camType = 0;
  int xSignType = 0;

  int activeNDA = 0;  // NDA 표시용

  if (sm.alive("roadLimitSpeed")) {
    const auto rls = sm["roadLimitSpeed"].getRoadLimitSpeed();
    nRoadLimitSpeed_ = rls.getRoadLimitSpeed();
    camLimitSpeed = rls.getCamLimitSpeed();
    camLeftDist = rls.getCamLimitSpeedLeftDist();
    sectionLimitSpeed = rls.getSectionLimitSpeed();
    sectionLeftDist = rls.getSectionLeftDist();
    camType = rls.getCamType();
    // xSignType는 포크 필드 없으면 0 유지
    activeNDA = rls.getActive();
  }

  const bool cam_detected = (camLimitSpeed > 0 && xSignType != 22 && xSignType != 4);

  // ---------- 공용 helper (중복 제거) ----------
  auto badgeRect = [](int cx, int cy, int w, int h) {
    return QRectF(cx - w/2, cy - h/2, w, h);
  };

  auto drawBadge = [&](int cx, int cy, int w, int h, float radius,
                       const QColor &fill, float stroke_w, const QColor &stroke,
                       const QString &txt, int px, const QColor &txt_col,
                       bool shadow = true) {
    QRectF r = badgeRect(cx, cy, w, h);
    qp_fill_rect(p, r, &fill, radius, stroke_w, &stroke);

    configFont(p, "Inter", px, "Bold");
    if (shadow) {
      p.setPen(QColor(0,0,0,220));
      p.drawText(r.translated(2,2).toRect(), Qt::AlignCenter, txt);
    }
    p.setPen(txt_col);
    p.drawText(r.toRect(), Qt::AlignCenter, txt);
  };

  auto drawTextCenter = [&](float cx, float cy, const QString &txt, int px, const QColor &col,
                            bool shadow = true, int dx = 3, int dy = 3) {
    if (shadow) {
      drawOutlinedText(p, cx, cy, txt, px, col, "Inter", QFont::Bold,
                       0.0f, (float)dx, QColor(0,0,0,255), QColor(0,0,0,255));
    } else {
      drawOutlinedText(p, cx, cy, txt, px, col, "Inter", QFont::Bold,
                       0.0f, 0.0f, QColor(0,0,0,0), QColor(0,0,0,0));
    }
  };

  // ---------- draw start ----------
  p.save();
  p.setRenderHint(QPainter::Antialiasing);

  // 배경 (show_device_state 조건 제거하고 항상 전체 배경 사용해도 되지만,
  // 기존 레이아웃 유지 위해 "컷"은 그대로 두고, show_device_state는 의미만 제거)
  QColor stroke_col(255,255,255,255);
  QColor bg_col = (cam_detected && blink_timer > 8) ? QColor(201,34,49,180) : QColor(0,0,0,90);

  // 기존 레이아웃 그대로(상단 컷은 유지)
  qp_fill_rect(p, QRectF(panel_x, panel_y_full + cut_h, panel_w, panel_h - cut_h),
               &bg_col, 30.f, 2.f, &stroke_col);

  // ===== (1) 교통신호 아이콘 =====
  {
    int icon_size = 80;
    int icon_red = icon_size;
    int icon_green = icon_size;

    bool red_light = (trafficState == 1);
    bool green_light = (trafficState == 2);

    if (trafficState_carrot == 1) { red_light = true; icon_red = (int)(icon_red * 1.5); }
    else if (trafficState_carrot == 2) { green_light = true; icon_green = (int)(icon_green * 1.5); }

    auto drawIconCentered = [&](const QPixmap &pm, int cx, int cy, int sz) {
      if (pm.isNull()) return;
      p.drawPixmap(QRect(cx - sz/2, cy - sz/2, sz, sz), pm);
    };

    if (red_light) drawIconCentered(ic_traffic_red, bx, by, icon_red);
    else if (green_light) drawIconCentered(ic_traffic_green, bx, by, icon_green);
  }

  // ===== (2) 현재속도 + speed_bg =====
  const float v = car_state.getCluSpeedMs() * (s->scene.is_metric ? (float)MS_TO_KPH : (float)MS_TO_MPH);
  float cur_speed = std::max(0.0f, v);


  if (!ic_speed_bg.isNull()) {
    p.drawPixmap(QRect(bx - 100, by - 60, 350, 150), ic_speed_bg);
  } else {
    QColor fill(0,0,0,120), st(255,255,255,80);
    qp_fill_rect(p, QRectF(bx - 100, by - 60, 350, 150), &fill, 18.f, 2.f, &st);
  }

  drawTextCenter((float)bx, (float)(by + 50), QString::number((int)std::nearbyint(cur_speed)),
                 120, QColor(255,255,255,255), true, 3, 3);

  // ===== (3) 크루즈 속도 + 변경 시 애니 =====
  {
    static QString last_cruise;
    const bool longActive = cs.getEnabled();
    float v_cruise = cs.getVCruise();
    QString cruise_txt = longActive ? QString::number((int)std::nearbyint(s->scene.is_metric ? v_cruise : (v_cruise * KM_TO_MILE)))
                                    : "--";

    if (cruise_txt != last_cruise) {
      last_cruise = cruise_txt;
      if (cruise_txt != "--") {
        startAnimText((float)(bx + 170), (float)(by + 15), cruise_txt, 60.f, QColor(0,255,0,255), "Inter");
      }
    }
    drawTextCenter((float)(bx + 170), (float)(by + 15), cruise_txt, 60, QColor(0,255,0,255), true, 2, 2);
  }

  // ===== (4) Apply speed (값 없으면 자연히 미표시) =====
  {
    const int apply_x = bx + 250;
    const int apply_y = by - 50;
    const QColor ochre(218,202,37,255);
    const QColor green(0,255,0,255);

    if (!apply_source.isEmpty()) {
      int as = (int)std::nearbyint(s->scene.is_metric ? apply_speed : (apply_speed * KM_TO_MILE));
      drawTextCenter((float)apply_x, (float)apply_y, QString::number(as), 50, ochre, true, 2, 2);
      drawTextCenter((float)apply_x, (float)(apply_y - 50), apply_source, 30, ochre, true, 2, 2);
    } else {
      const float v_cruise = cs.getVCruise();
      if (std::fabs(cruiseTarget - v_cruise) > 0.5f) {
        int ts = (int)std::nearbyint(s->scene.is_metric ? cruiseTarget : (cruiseTarget * KM_TO_MILE));
        drawTextCenter((float)apply_x, (float)apply_y, QString::number(ts), 50, green, true, 2, 2);
        drawTextCenter((float)apply_x, (float)(apply_y - 50), "eco", 30, green, true, 2, 2);
      }
    }
  }

  // ===== (5) Driving Mode 배지 (GPS 제거 완료) =====
  {
    static QString last_mode;

    int dx = bx - 50;
    int dy = by + 175;

    QString mode_txt = "일반";
    QColor mode_fill(128,128,128,210);
    QColor mode_text(255,255,255,255);

    switch (myDrivingMode) {
      case 1: mode_txt="연비";  mode_fill=QColor(0, 255, 0, 210);     break;
      case 2: mode_txt="안전";  mode_fill=QColor(255, 165, 0, 210);   break;
      case 3: mode_txt="일반";  mode_fill=QColor(128, 128, 128, 210); break;
      case 4: mode_txt="고속";  mode_fill=QColor(201, 34, 49, 210);   break;
      case 5: mode_txt="AUTO";  mode_fill=QColor(0, 160, 255, 210);   break;
      default: break;
    }

    drawBadge(dx, dy - 14, 140, 48, 15.f, mode_fill, 2.f, QColor(0,0,0,0),
              mode_txt, 30, mode_text, false);

    if (mode_txt != last_mode) {
      last_mode = mode_txt;
      startAnimText((float)dx, (float)dy, mode_txt, 30.f, QColor(255,255,255,255), "Inter");
    }
  }

  // ===== (6) GAP 숫자 + 세로바 (위치 그대로, lp.getCruiseGap 기반) =====
  {
    static int last_gap = -999;

    float gap_f = std::clamp(lp_cruise_gap, 0.0f, 4.0f);
    int gap = std::clamp((int)std::nearbyint(gap_f), 0, 4);

    // 숫자
    int dx_num = bx + 220;
    int dy_num = by + 77;
    drawTextCenter((float)dx_num, (float)dy_num, QString::number(gap), 40, QColor(255,255,255,255), true, 2, 2);

    if (gap != last_gap) {
      last_gap = gap;
      startAnimText((float)dx_num, (float)dy_num, QString::number(gap), 40.f, QColor(255,255,255,255), "Inter");
    }

    // 바(기존과 동일)
    int dx = bx + 270;
    int dy = by + 185;
    float ddy = 80.f / 4.f;

    QColor fill(0,255,0,210);
    QColor white(255,255,255,255);

    for (int i = 0; i < gap; i++) {
      QRectF r(dx, (dy - ddy*(i+1) + 2), 70, (ddy - 2));
      qp_fill_rect(p, r, &fill, 4.f, 3.f, &white);
    }
  }

  // ===== (7) 기어 박스 + 변경시 애니 =====
  {
    static QString last_gear;

    int dx = bx + 305;
    int dy = by + 60;

    QString gear = "D";
    switch (car_state.getGearShifter()) {
      case cereal::CarState::GearShifter::PARK:    gear = "P"; break;
      case cereal::CarState::GearShifter::REVERSE: gear = "R"; break;
      case cereal::CarState::GearShifter::NEUTRAL: gear = "N"; break;
      default: break;
    }

    int cur_gear = (int)std::nearbyint(car_state.getCurrentGear());
    bool show_gear_num = (gear == "D") && (cur_gear >= 1 && cur_gear <= 8);

    QString draw_txt = show_gear_num ? QString::number(cur_gear) : gear;

    QRectF gr(dx - 35, dy - 70, 70, 80);
    QColor fill(0,255,0,210);
    QColor white(255,255,255,255);
    qp_fill_rect(p, gr, &fill, 15.f, 3.f, &white);

    drawTextCenter((float)dx, (float)dy, draw_txt, 70, QColor(255,255,255,255), true, 2, 2);

    if (draw_txt != last_gear) {
      last_gear = draw_txt;
      startAnimText((float)dx, (float)dy, draw_txt, 70.f, QColor(255,255,255,255), "Inter");
    }
  }

  // ===== (8) NDA (roadLimitSpeed.getActive() 기반만) =====
  {
    int dx = bx + 200;
    int dy = by + 175;

    if (activeNDA > 0) {
      const char *txt = (activeNDA == 1) ? "NDA" : "HDA";
      drawBadge(dx, dy, 110, 48, 15.f,
                QColor(0, 255, 0, 255),
                2.f, QColor(0,0,0,0),
                txt, 40, QColor(255,255,255,255), false);
    }
  }

  // ===== (9) CAM/LIMIT 박스 =====
  {
    int dx = bx + 75;
    int dy = by + 175;

    int disp_speed = 0;
    QColor fill(0,255,0,210);

    if (camLimitSpeed > 0 && xSignType != 22) {
      disp_speed = (int)std::nearbyint(camLimitSpeed * (s->scene.is_metric ? 1.0 : KM_TO_MILE));
      fill = (blink_timer <= 8) ? QColor(201,34,49,210) : QColor(218,202,37,210);
      drawTextCenter((float)dx, (float)(dy - 45), "CAM", 30, QColor(255,255,255,255), true, 2, 2);
    } else {
      disp_speed = nRoadLimitSpeed_;
      disp_speed = (int)std::nearbyint(disp_speed * (s->scene.is_metric ? 1.0 : KM_TO_MILE));
      const float v_kph = v_ms * 3.6f;
      fill = (v_kph > disp_speed + 2) ? QColor(201,34,49,210) : QColor(255,255,255,210);
      drawTextCenter((float)dx, (float)(dy - 45), "LIMIT", 30, QColor(255,255,255,255), true, 2, 2);
    }

    drawBadge(dx, dy, 110, 48, 15.f, fill, 2.f, QColor(0,0,0,0),
              QString::number(disp_speed), 40, QColor(255,255,255,255), false);
  }

  // ===== (10) Device State: CPU TEMP always (조건 없이 항상) =====
  {
    int cpuTempAvg = 0;
    for (auto t : device.getCpuTempC()) cpuTempAvg += (int)t;
    if (device.getCpuTempC().size()) cpuTempAvg /= (int)device.getCpuTempC().size();

    int dx = bx - 35;
    int dy = by - 200;

    QColor normal(0,255,0,190);
    QColor red(255,0,0,255);

    auto devBox = [&](int cx, const QString &title, const QString &val, bool warn) {
      QColor fill = (warn && blink_timer <= 8) ? red : normal;
      QRectF r(cx - 65, dy - 38, 130, 90);
      qp_fill_rect(p, r, &fill, 15.f, 2.f, nullptr);

      configFont(p, "Inter", 25, "Bold");
      p.setPen(QColor(255,255,255,255));
      p.drawText(QRect(cx - 65, dy - 38, 130, 40), Qt::AlignCenter, title);

      configFont(p, "Inter", 40, "Bold");
      p.drawText(QRect(cx - 65, dy + 5, 130, 50), Qt::AlignCenter, val);
    };

    devBox(dx, "CPU", QString("%1°C").arg(cpuTempAvg), cpuTempAvg > 80);
  }

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
