#ifndef LIGHTINGWIDGET_H
#define LIGHTINGWIDGET_H

#include <QWidget>
#include <QAction>
#include <QHash>

#include <memory>

class GLView;
class QSlider;

namespace Ui {
class LightingWidget;
}

class LightingWidget : public QWidget
{
	Q_OBJECT

public:
	LightingWidget( GLView * ogl, QWidget * parent = nullptr);
	~LightingWidget();

	void setDefaults();
	void setActions( QVector<QAction *> actions );

public slots:
	void saveSettings();
	//! Reset all sliders, the light direction and the display toggles to their defaults
	void resetAll();

protected:
	bool eventFilter( QObject * watched, QEvent * event ) override;

private:
	std::unique_ptr<Ui::LightingWidget> ui;
	GLView * glView;
	QVector<QAction *> actions;
	//! Default value of each slider, for double-click reset
	QHash<QObject *, int> sliderDefaults;

	enum
	{
		BRIGHT = 1440,
		POS = 720,

		DirMin = 0,
		DirMax = BRIGHT,
		LightColorMin = 0,
		LightColorMax = BRIGHT,

		AmbientMin = 0,
		AmbientMax = BRIGHT,
		EnvMapRotationMin = -POS,
		EnvMapRotationMax = POS,
		GlowScaleMin = 0,
		GlowScaleMax = BRIGHT,

		LightScaleMin = 0,
		LightScaleMax = BRIGHT,
		ToneMappingMin = 0,
		ToneMappingMax = BRIGHT,

		DirDefault = DirMax / 2,
		LightColorDefault = POS,
		AmbientDefault = AmbientMax / 2,
		EnvMapRotationDefault = 0,
		GlowScaleDefault = GlowScaleMax / 2,
		LightScaleDefault = POS,
		ToneMappingDefault = POS
	};
};

#endif // LIGHTINGWIDGET_H
