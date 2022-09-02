#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_main.h"

#include "Visualizer.h"

class CGUI : public QMainWindow
{
	Q_OBJECT

public:
	CGUI(int argc, char* argv[], QWidget *parent = Q_NULLPTR);
	~CGUI() override;

private Q_SLOTS:



private:
	void __initWidget();


private:
	std::shared_ptr<Ui::CMainGUI> m_pMainGUI;
	std::shared_ptr<visualization::CVisualizer> m_pVisualizer;
};
