#pragma once
#include <QSlider>

class QDoubleSlider : public QSlider
{
	Q_OBJECT

public:
	QDoubleSlider(QWidget* vParent = 0) : QSlider(vParent)
	{
		connect(this, SIGNAL(valueChanged(int)),
			this, SLOT(notifyValueChanged(int)));
	}

signals:
	void doubleValueChanged(double vValue);

public slots:
	void notifyValueChanged(int vValue)
	{
		const double Value = vValue / 1000.0;
		emit doubleValueChanged(Value);
	}

	void setValue(double vValue)
	{
		const int Value = static_cast<int>(vValue * 1000.0);
		QAbstractSlider::setValue(Value);
	}
};