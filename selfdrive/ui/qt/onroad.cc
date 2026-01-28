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

static void drawGapBars(QPainter &p, int x, int y, int gap, bool active_long) {
  p.save();

  int bars = std::clamp(gap, 0, 4);

  const int bar_w   = 26;
  const int bar_h   = 24;
  const int bar_gap = 8;
  const int radius  = 4;

  QColor onColor  = QColor(0, 255, 0, 255);   // 활성: 진한 녹색
  QColor offColor = QColor(0, 200, 0, 60);    // 비활성: 연한 녹색
  QColor borderColor = QColor(0, 120, 0, 200);  // 테두리도 항상 녹색

  p.setPen(QPen(borderColor, 1));

  for (int i = 0; i < 4; i++) {
    int yy = y - i * (bar_h + bar_gap);

    QRect r(x, yy, bar_w, bar_h);
    p.setBrush(i < bars ? onColor : offColor);
    p.drawRoundedRect(r, radius, radius);
  }

  p.restore();
}

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
	
  buttons = new ButtonsWindow(this);
  stacked_layout->addWidget(buttons);


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
  buttons->updateState(s);
	
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

// ***** onroad widgets *****
// ***** onroad widgets *****

ButtonsWindow::ButtonsWindow(QWidget *parent) : QWidget(parent) {
  QVBoxLayout *main_layout  = new QVBoxLayout(this);

  QWidget *btns_wrapper = new QWidget(this);
  QHBoxLayout *btns_layout  = new QHBoxLayout(btns_wrapper);

  // ===== 레이아웃 기본 =====
  btns_layout->setContentsMargins(0, 770, 30, 30);
  btns_layout->setSpacing(14);
  btns_layout->setAlignment(Qt::AlignLeft | Qt::AlignTop);
  main_layout->addWidget(btns_wrapper, 0, Qt::AlignTop | Qt::AlignLeft);

  dlpBtn = new QPushButton("");
  dlpBtn->setFixedWidth(186);
  dlpBtn->setFixedHeight(140);

  QObject::connect(dlpBtn, &QPushButton::clicked, [=]() {
    uiState()->scene.dynamic_lane_profile = uiState()->scene.dynamic_lane_profile + 1;
    if (uiState()->scene.dynamic_lane_profile > 2) {
      uiState()->scene.dynamic_lane_profile = 0;
    }

    if (uiState()->scene.dynamic_lane_profile == 0) {
      Params().put("DynamicLaneProfile", "0", 1);
      dlpBtn->setText("Lane\nonly");
    } else if (uiState()->scene.dynamic_lane_profile == 1) {
      Params().put("DynamicLaneProfile", "1", 1);
      dlpBtn->setText("Lane\nless");
    } else if (uiState()->scene.dynamic_lane_profile == 2) {
      Params().put("DynamicLaneProfile", "2", 1);
      dlpBtn->setText("Auto\nLane");
    }
  });

  btns_layout->addWidget(dlpBtn, 0, Qt::AlignLeft);

  // 2) E2E/ACC 표시 박스 (표시 전용)
  modeBtn = new QPushButton("ACC");
  modeBtn->setFixedWidth(186);
  modeBtn->setFixedHeight(140);
  modeBtn->setEnabled(false);
  modeBtn->setFocusPolicy(Qt::NoFocus);

  btns_layout->addWidget(modeBtn, 0, Qt::AlignLeft);
  btns_layout->addStretch(1);

  setStyleSheet(R"(
    QPushButton {
      color: white;
      text-align: center;
      padding: 0px;
      border-width: 9px;
      border-style: solid;
      background-color: rgba(0, 0, 0, 0.3);
    }
  )");
}

void ButtonsWindow::updateState(const UIState &s) {
  if (uiState()->scene.dynamic_lane_profile == 0) {
    dlpBtn->setStyleSheet(QString(
      "font-size: 45px;"
      "border-radius: 100px;"
      "border-width: 9px;"
      "border-style: solid;"
      "border-color: %1;"
      "color: white;"
      "background-color: rgba(0,0,0,0.30);"
    ).arg(dlpBtnColors.at(0)));
    dlpBtn->setText("Lane\nonly");

  } else if (uiState()->scene.dynamic_lane_profile == 1) {
    dlpBtn->setStyleSheet(QString(
      "font-size: 45px;"
      "border-radius: 100px;"
      "border-width: 9px;"
      "border-style: solid;"
      "border-color: %1;"
      "color: white;"
      "background-color: rgba(0,0,0,0.30);"
    ).arg(dlpBtnColors.at(1)));
    dlpBtn->setText("Lane\nless");

  } else if (uiState()->scene.dynamic_lane_profile == 2) {
    dlpBtn->setStyleSheet(QString(
      "font-size: 45px;"
      "border-radius: 100px;"
      "border-width: 9px;"
      "border-style: solid;"
      "border-color: %1;"
      "color: white;"
      "background-color: rgba(0,0,0,0.30);"
    ).arg(dlpBtnColors.at(2)));
    dlpBtn->setText("Auto\nLane");
  }

  // 2) E2E / ACC 표시 업데이트
  bool is_e2e = false;

  if (s.sm && s.sm->alive("longitudinalPlan")) {
    const auto lp = (*s.sm)["longitudinalPlan"].getLongitudinalPlan();
    is_e2e = (lp.getMpcMode() == 1);
  }

  if (is_e2e) {
    modeBtn->setText("E2E");
    modeBtn->setStyleSheet(
      "font-size: 54px;"
      "border-radius: 100px;"
      "border-width: 9px;"
      "border-style: solid;"
      "border-color: rgba(0,160,255,0.90);"   // E2E: 파란 테두리
      "color: white;"
      "background-color: rgba(0,0,0,0.30);"
    );
  } else {
    modeBtn->setText("ACC");
    modeBtn->setStyleSheet(
      "font-size: 54px;"
      "border-radius: 100px;"
      "border-width: 9px;"
      "border-style: solid;"
      "border-color: rgba(0,220,0,0.85);"    // ACC: 초록 테두리
      "color: white;"
      "background-color: rgba(0,0,0,0.30);"
    );
  }
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
  compass_inner_img = QPixmap("../assets/images/compass_inner.png").scaled(img_size, img_size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  compass_outer_img = QPixmap("../assets/images/compass_outer.png").scaled(img_size, img_size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
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
  // compass
  if (compass && bearingAccuracyDeg != 180.00) {
    drawCompass(p, rect().right() - radius / 2 - bdr_s * 2, radius / 2 + bdr_s + 530,
                compass_outer_img, blackColor(180), 5.0, bearingDeg);
  }

  {
    const SubMaster &sm = *(uiState()->sm);
    const auto cs = sm["controlsState"].getControlsState();
    const auto car_state = sm["carState"].getCarState();

    const bool soft_hold_active = calc_soft_hold_active(cs, car_state);

    if (soft_hold_active) {
      p.save();

      const int cx = rect().right() - radius / 2 - bdr_s * 2;
      const int cy = radius / 2 + bdr_s + 530;

      const int r = radius;
      p.setPen(Qt::NoPen);

      p.setBrush(QColor(220, 0, 0, 200));   // R,G,B,Alpha (알파는 취향대로 180~220)
      p.drawEllipse(cx - r / 2, cy - r / 2, r, r);

      const QColor textColor(255, 255, 255, 255);   // ✅ 가독성 좋게 흰색 추천
      const QColor shadow(0, 0, 0, 220);

      configFont(p, "Open Sans", 34, "Black");       // 기존 26 -> 34, Bold -> Black
      QFont f = p.font();
      f.setWeight(QFont::Black);                     // ✅ 더 두껍게 강제
      p.setFont(f);

      QFontMetrics fm(p.font());

      const QString line1 = "SOFT";
      const QString line2 = "HOLD";

      const int line_gap = fm.height() - 4;          // 글자 커졌으니 간격도 살짝 조정
      const int total_h = line_gap * 2;
      const int y_start = cy - total_h / 2 + fm.ascent();

      // SOFT
      const int w1 = fm.horizontalAdvance(line1);
      p.setPen(shadow);
      p.drawText(cx - w1 / 2 + 3, y_start + 3, line1);   // ✅ 그림자도 조금 더 두껍게(오프셋 증가)
      p.setPen(textColor);
      p.drawText(cx - w1 / 2, y_start, line1);

      // HOLD
      const int w2 = fm.horizontalAdvance(line2);
      p.setPen(shadow);
      p.drawText(cx - w2 / 2 + 3, y_start + line_gap + 3, line2);
      p.setPen(textColor);
      p.drawText(cx - w2 / 2, y_start + line_gap, line2);

      p.restore();
	}
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
  drawCarrotHud_ByPath(p);
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

void OnroadHud::drawCompass(QPainter &p, int x, int y, QPixmap &img, QBrush bg, float opacity, float bearing_Deg) {
  // Draw the circle background
  p.setBrush(bg);
  p.drawEllipse(x - radius / 2, y - radius / 2, radius, radius);

  // Rotate the compass_inner_img image
  p.save();
  p.translate(x, y);
  p.rotate(bearing_Deg);
  p.drawPixmap(-compass_inner_img.width() / 2, -compass_inner_img.height() / 2, compass_inner_img);
  p.restore();

  // Display compass_outer_img
  //QPixmap imgScaled = img.scaled(img.width() * 2, img.height() * 2, Qt::KeepAspectRatio);
  p.drawPixmap(x - img_size / 2, y - img_size / 2, img);

  // Set the font for the direction labels
  QFont font = p.font();
  font.setFamily("Inter");
  font.setBold(true);
  font.setPointSize(10);
  p.setFont(font);
  p.setPen(Qt::white);

  // Draw the cardinal directions
  const auto drawDirection = [&](const QString &text, float from, float to, int hAlign, int vAlign) {
    // Set the opacity based on whether the direction label is currently being pointed at
    p.setOpacity((bearing_Deg >= from && bearing_Deg < to) ? 1.0 : 0.2);
    p.drawText(x - radius / 2, y - radius / 2, radius, radius, hAlign | vAlign, text);
  };
  drawDirection("N", 0, 67.5, Qt::AlignTop | Qt::AlignHCenter, {});
  drawDirection("E", 22.5, 157.5, Qt::AlignRight | Qt::AlignVCenter, {});
  drawDirection("S", 112.5, 247.5, Qt::AlignBottom | Qt::AlignHCenter, {});
  drawDirection("W", 202.5, 337.5, Qt::AlignLeft | Qt::AlignVCenter, {});
  drawDirection("N", 292.5, 360, Qt::AlignTop | Qt::AlignHCenter, {});
}

// ================= Carrot HUD (paint.h 위치 기준) =================
void OnroadHud::drawCarrotHud_ByPath(QPainter &p) {
  auto *ui = uiState();
  if (!ui || !ui->sm || !ui->sm->alive("carState") ||
      !ui->sm->alive("controlsState") || !ui->sm->alive("longitudinalPlan")) {
    return;
  }

  const SubMaster &sm = *(ui->sm);
  const auto car_state = sm["carState"].getCarState();
  const auto cs = sm["controlsState"].getControlsState();
  const auto lp = sm["longitudinalPlan"].getLongitudinalPlan();

  // ---------- paint.h 기준점 계산 ----------
  float path_fx = width() / 2.f;
  float path_fy = height() - 400.f;

  bool has_path_end =
      ui->scene.path_end_left_vertices.size() > 0 &&
      ui->scene.path_end_right_vertices.size() > 0;

  if (has_path_end) {
    float lex = ui->scene.path_end_left_vertices[0].x();
    float rex = ui->scene.path_end_right_vertices[0].x();
    float ley = ui->scene.path_end_left_vertices[0].y();
    float rey = ui->scene.path_end_right_vertices[0].y();

    float cx = (lex + rex) / 2.f;
    float cy = (ley + rey) / 2.f;

    cx = std::clamp(cx, 550.f, (float)width() - 550.f);
    cy = std::clamp(cy, 200.f, (float)height() - 100.f);

    path_fx = cx;
    path_fy = cy;
  }

  int x = (int)path_fx;
  int y = (int)(path_fy - 135.f);

  // ---------- 현재 속도 ----------
  float v_ego = car_state.getVEgoCluster();
  float cur_speed = v_ego * (ui->scene.is_metric ? MS_TO_KPH : MS_TO_MPH);
  if (cur_speed < 0) cur_speed = 0;

  int bx = x;
  int by = y + 270;

  QPixmap speed_bg("../assets/images/speed_bg.png");
  if (!speed_bg.isNull()) {
    p.drawPixmap(bx - 100, by - 60, 350, 150, speed_bg);
  }

  configFont(p, "Open Sans", 120, "Bold");
  p.setPen(Qt::white);
  p.drawText(QRect(bx - 200, by - 80, 400, 200),
             Qt::AlignCenter,
             QString::number((int)std::nearbyint(cur_speed)));

  // ---------- 기어 ----------
  QString gear = "D";
  switch (car_state.getGearShifter()) {
    case cereal::CarState::GearShifter::PARK: gear = "P"; break;
    case cereal::CarState::GearShifter::REVERSE: gear = "R"; break;
    case cereal::CarState::GearShifter::NEUTRAL: gear = "N"; break;
    case cereal::CarState::GearShifter::SPORT: gear = "S"; break;
    case cereal::CarState::GearShifter::LOW: gear = "L"; break;
    default: break;
  }

  configFont(p, "Open Sans", 44, "Bold");
  p.setPen(QColor(0,255,0,230));
  p.drawText(QRect(bx + 120, by + 55, 140, 70),
             Qt::AlignLeft | Qt::AlignVCenter, gear);

  // ---------- Driving Mode + GAP ----------
  int dxGap = -128 - 10 - 40;

  QString mode = "GAP";
  switch (cs.getMyDrivingMode()) {
    case 1: mode = "연비"; break;
    case 2: mode = "안전"; break;
    case 3: mode = "일반"; break;
    case 4: mode = "고속"; break;
  }

  float tFollow = lp.getTFollow();
  int gap = std::clamp((int)std::nearbyint(lp.getCruiseGap()), 0, 4);

  configFont(p, "Open Sans", 30, "Bold");
  p.setPen(Qt::white);

  p.drawText(QRect(x + dxGap - 165, y + 80, 300, 50),
             Qt::AlignCenter, QString::number(tFollow, 'f', 2));

  p.drawText(QRect(x + dxGap - 165, y + 120, 300, 50),
             Qt::AlignCenter,
             QString("%1M").arg(QString::number(tFollow * v_ego + 6.f, 'f', 0)));

  p.drawText(QRect(x + dxGap - 165, y + 160, 300, 50),
             Qt::AlignCenter, mode);

  dxGap -= 60;

  bool active_long = cs.getLongActiveUser() > 0;
  drawGapBars(p, x + dxGap, y + 5 + 64, gap, active_long);

  configFont(p, "Open Sans", 25, "Bold");
  p.drawText(QRect(x + dxGap - 40, y + 90, 160, 60),
             Qt::AlignCenter, "GAP");
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
	
  drawMaxSpeed(p);
  drawGpsStatus(p);
  drawBrake(p);
  drawMisc(p);
	
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

void NvgWindow::drawSpeed(QPainter &p) {
  UIState *s = uiState();
  const SubMaster &sm = *(s->sm);
  float cur_speed = std::max(0.0, sm["carState"].getCarState().getCluSpeedMs() * (s->scene.is_metric ? MS_TO_KPH : MS_TO_MPH));
  m_cur_speed = cur_speed;
  auto car_state = sm["carState"].getCarState();
  float accel = car_state.getAEgo();

  QColor color = QColor(255, 255, 255, 250);

  if(accel > 0) {
    int a = (int)(255.f - (180.f * (accel/2.f)));
    a = std::min(a, 255);
    a = std::max(a, 80);
    color = QColor(a, a, 255, 250);
  }
  else {
    int a = (int)(255.f - (255.f * (-accel/3.f)));
    a = std::min(a, 255);
    a = std::max(a, 60);
    color = QColor(255, a, a, 250);
  }

  QString speed;
  speed.sprintf("%.0f", cur_speed);
  configFont(p, "Open Sans", 176, "Bold");
  drawTextWithColor(p, rect().center().x(), 250, speed, color);

  configFont(p, "Open Sans", 66, "Regular");
  //drawText(p, rect().center().x(), 310, s->scene.is_metric ? "km/h" : "mph", 200)
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

void NvgWindow::drawMaxSpeed(QPainter &p) {
  p.save();

  UIState *s = uiState();
  const SubMaster &sm = *(s->sm);
  const auto scc_smoother = sm["carControl"].getCarControl().getSccSmoother();
  const auto road_limit_speed = sm["roadLimitSpeed"].getRoadLimitSpeed();
  const auto car_params = sm["carParams"].getCarParams();

  bool is_metric = s->scene.is_metric;
  bool long_control = scc_smoother.getLongControl();

 // kph
  float applyMaxSpeed = scc_smoother.getApplyMaxSpeed();
  float cruiseMaxSpeed = scc_smoother.getCruiseMaxSpeed();

  bool is_cruise_set = (cruiseMaxSpeed > 0 && cruiseMaxSpeed < 255);

  int activeNDA = road_limit_speed.getActive();
  int roadLimitSpeed = road_limit_speed.getRoadLimitSpeed();
  int camLimitSpeed = road_limit_speed.getCamLimitSpeed();
  int camLimitSpeedLeftDist = road_limit_speed.getCamLimitSpeedLeftDist();
  int sectionLimitSpeed = road_limit_speed.getSectionLimitSpeed();
  int sectionLeftDist = road_limit_speed.getSectionLeftDist();

  int limit_speed = 0;
  int left_dist = 0;

  if(camLimitSpeed > 0 && camLimitSpeedLeftDist > 0) {
    limit_speed = camLimitSpeed;
    left_dist = camLimitSpeedLeftDist;
  }
  else if(sectionLimitSpeed > 0 && sectionLeftDist > 0) {
    limit_speed = sectionLimitSpeed;
    left_dist = sectionLeftDist;
  }

  if(activeNDA > 0)
  {
      int w = 150;
      int h = 54;
      int x = (width() + (bdr_s*2))/2 - w/2 - bdr_s;
      int y = 40 - bdr_s;

      p.setOpacity(1.f);
      p.drawPixmap(x, y, w, h, activeNDA == 1 ? ic_nda : ic_hda);
  }
  
  const int x_start = 30;
  const int y_start = 30;

  int board_width = 210;
  int board_height = 384;

  const int corner_radius = 32;
  int max_speed_height = 210;

  QColor bgColor = QColor(0, 0, 0, 166);

  {
    // draw board
    QPainterPath path;
    path.setFillRule(Qt::WindingFill);

    if(limit_speed > 0 && left_dist > 0) {
      board_width = limit_speed < 100 ? 210 : 230;
      board_height = max_speed_height + board_width;

      path.addRoundedRect(QRectF(x_start, y_start, board_width, board_height-board_width/2), corner_radius, corner_radius);
      path.addRoundedRect(QRectF(x_start, y_start+corner_radius, board_width, board_height-corner_radius), board_width/2, board_width/2);
    }
    else if(roadLimitSpeed > 0 && roadLimitSpeed < 200) {
      board_height = 485;
      path.addRoundedRect(QRectF(x_start, y_start, board_width, board_height), corner_radius, corner_radius);
    }
    else {
      max_speed_height = 235;
      board_height = max_speed_height;
      path.addRoundedRect(QRectF(x_start, y_start, board_width, board_height), corner_radius, corner_radius);
    }

    p.setPen(Qt::NoPen);
    p.fillPath(path.simplified(), bgColor);
  }
	
  QString str;
	
  // Max Speed
  {
    p.setPen(QColor(255, 255, 255, 230));
     
    if(is_cruise_set) {
      configFont(p, "Inter", 80, "Bold");

      if(is_metric)
        str.sprintf( "%d", (int)(cruiseMaxSpeed + 0.5));
      else
        str.sprintf( "%d", (int)(cruiseMaxSpeed*KM_TO_MILE + 0.5));
    }
    else {
      configFont(p, "Inter", 60, "Bold");
      str = "N/A";
    }

    QRect speed_rect = getRect(p, Qt::AlignCenter, str);
    QRect max_speed_rect(x_start, y_start, board_width, max_speed_height/2);
    speed_rect.moveCenter({max_speed_rect.center().x(), 0});
    speed_rect.moveTop(max_speed_rect.top() + 35);
    p.drawText(speed_rect, Qt::AlignCenter | Qt::AlignVCenter, str);
  } 

    
  // applyMaxSpeed
  {
    p.setPen(QColor(255, 255, 255, 180));

    configFont(p, "Inter", 50, "Bold");
    if(is_cruise_set && applyMaxSpeed > 0) {
      if(is_metric)
        str.sprintf( "%d", (int)(applyMaxSpeed + 0.5));
      else
        str.sprintf( "%d", (int)(applyMaxSpeed*KM_TO_MILE + 0.5));
    }
    else {
      str = long_control ? "OP" : "MAX";
    }

    QRect speed_rect = getRect(p, Qt::AlignCenter, str);
    QRect max_speed_rect(x_start, y_start + max_speed_height/2, board_width, max_speed_height/2);
    speed_rect.moveCenter({max_speed_rect.center().x(), 0});
    speed_rect.moveTop(max_speed_rect.top() + 24);
    p.drawText(speed_rect, Qt::AlignCenter | Qt::AlignVCenter, str);  
  }
	
  //
  if(limit_speed > 0 && left_dist > 0) {
    QRect board_rect = QRect(x_start, y_start+board_height-board_width, board_width, board_width);

    if(road_limit_speed.getCamType() == 22) {
      int padding = 25;
      board_rect.adjust(padding, padding, -padding, -padding);
      p.drawPixmap(board_rect.x(), board_rect.y()-10, board_rect.width(), board_rect.height(), ic_safety_speed_bump);
    }
    else {
      int padding = 14;
      board_rect.adjust(padding, padding, -padding, -padding);
      p.setBrush(QBrush(Qt::white));
      p.drawEllipse(board_rect);

      padding = 18;
      board_rect.adjust(padding, padding, -padding, -padding);

      p.setBrush(Qt::NoBrush);
      p.setPen(QPen(Qt::red, 25));
      p.drawEllipse(board_rect);

      p.setPen(QPen(Qt::black, padding));

      str.sprintf("%d", limit_speed);
      p.setFont(InterFont(70, QFont::Bold));

      QRect text_rect = getRect(p, Qt::AlignCenter, str);
      QRect b_rect = board_rect;
      text_rect.moveCenter({b_rect.center().x(), 0});
      text_rect.moveTop(b_rect.top() + (b_rect.height() - text_rect.height()) / 2);
      p.drawText(text_rect, Qt::AlignCenter, str);
    }
	  
    // left dist
    QRect rcLeftDist;
    QString strLeftDist;

    if(left_dist < 1000)
      strLeftDist.sprintf("%dm", left_dist);
    else
      strLeftDist.sprintf("%.1fkm", left_dist / 1000.f);

    QFont font("Inter");
    font.setPixelSize(55);
    font.setStyleName("Bold");

    QFontMetrics fm(font);
    int width = fm.width(strLeftDist);

    int padding = 10;

    int center_x = x_start + board_width / 2;
    rcLeftDist.setRect(center_x - width / 2, y_start+board_height+15, width, font.pixelSize()+10);
    rcLeftDist.adjust(-padding*2, -padding, padding*2, padding);

    p.setPen(Qt::NoPen);
    p.setBrush(bgColor);
    p.drawRoundedRect(rcLeftDist, 20, 20);

    configFont(p, "Inter", 55, "Bold");
    p.setBrush(Qt::NoBrush);
    p.setPen(QColor(255, 255, 255, 230));
    p.drawText(rcLeftDist, Qt::AlignCenter|Qt::AlignVCenter, strLeftDist);  
  }
  else if(roadLimitSpeed > 0 && roadLimitSpeed < 200) {
    QRectF board_rect = QRectF(x_start, y_start+max_speed_height, board_width, board_height-max_speed_height);
    int padding = 14;
    board_rect.adjust(padding, padding, -padding, -padding);
    p.setBrush(QBrush(Qt::white));
    p.drawRoundedRect(board_rect, corner_radius-padding/2, corner_radius-padding/2);

    padding = 10;
    board_rect.adjust(padding, padding, -padding, -padding);
    p.setBrush(Qt::NoBrush);
    p.setPen(QPen(Qt::black, padding));
    p.drawRoundedRect(board_rect, corner_radius-12, corner_radius-12);

    {
      str = "SPEED\nLIMIT";
      configFont(p, "Inter", 35, "Bold");

      QRect text_rect = getRect(p, Qt::AlignCenter, str);
      QRect b_rect(board_rect.x(), board_rect.y(), board_rect.width(), board_rect.height()/2);
      text_rect.moveCenter({b_rect.center().x(), 0});
      text_rect.moveTop(b_rect.top() + 20);
      p.drawText(text_rect, Qt::AlignCenter, str);
    }

    {
      str.sprintf("%d", roadLimitSpeed);
      configFont(p, "Inter", 75, "Bold");

      QRect text_rect = getRect(p, Qt::AlignCenter, str);
      QRect b_rect(board_rect.x(), board_rect.y()+board_rect.height()/2, board_rect.width(), board_rect.height()/2);
      text_rect.moveCenter({b_rect.center().x(), 0});
      text_rect.moveTop(b_rect.top() + 3);
      p.drawText(text_rect, Qt::AlignCenter, str);
    }

    {
      configFont(p, "Inter", 10, "Bold");

      QRect text_rect = getRect(p, Qt::AlignCenter, str);
      QRect b_rect(board_rect.x(), board_rect.y(), board_rect.width(), board_rect.height()/2);
      text_rect.moveCenter({b_rect.center().x(), 0});
      text_rect.moveTop(b_rect.top() + 20);
      p.drawText(text_rect, Qt::AlignCenter, str);
    } 
  }

  p.restore();
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

void NvgWindow::drawCgear(QPainter &p) {
  const SubMaster &sm = *(uiState()->sm);
  auto car_state = sm["carState"].getCarState();

  auto t_gear = car_state.getCurrentGear();
  int shifter;

  shifter = int(car_state.getGearShifter());

  QString tgear, tgearshifter;

  tgear.sprintf("%.0f", t_gear);
  configFont(p, "Open Sans", 130, "Semi Bold");

  //shifter = 1;
	
  QRect rc(30, 620, 182, 135);
  p.setPen(QPen(QColor(0xff, 0xff, 0xff, 100), 10));
  p.setBrush(QColor(0, 0, 0, 100));
  p.drawRoundedRect(rc, 20, 20);
  p.setPen(Qt::NoPen);
	
  if ((t_gear < 9) && (t_gear !=0)) { 
    p.setPen(QColor(255, 255, 255, 255)); 
    p.drawText(rc.center().x() - 38, rc.center().y() + 48, tgear);
  } else if (t_gear == 14 ) { 
    p.setPen(QColor(201, 34, 49, 255));
    p.drawText(rc.center().x() - 38, rc.center().y() + 48, "R");
  } else if (shifter == 1 ) { 
    p.setPen(QColor(255, 255, 255, 255));
    p.drawText(rc.center().x() - 38, rc.center().y() + 48, "P");
  } else if (shifter == 3 ) {  
    p.setPen(QColor(255, 255, 255, 255));
    p.drawText(rc.center().x() - 40, rc.center().y() + 48, "N");
  }
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
