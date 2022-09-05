#include "GUI.h"
#include <QtWidgets/QFileDialog>
#include <vtkAutoInit.h>

VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);

CGUI::CGUI(int argc, char* argv[], QWidget *vParent)
	: QMainWindow(vParent)
	, m_pMainGUI(new Ui::CMainGUI)
	, m_pVisualizer(new visualization::CVisualizer(argc, argv))
{
	m_pMainGUI->setupUi(this);
}

CGUI::~CGUI()
{
	m_pVisualizer = nullptr;
}

void CGUI::init()
{
	__initWidget();
	__connectSignal();
}

void CGUI::__refresh()
{
	std::pair<int, int> CurGeo(this->width(), this->height());
	this->adjustSize();
	this->resize(CurGeo.first, CurGeo.second);
}

void CGUI::__initWidget()
{
	m_pMainGUI->CVTKWidget->SetRenderWindow(m_pVisualizer->getRenderWindow());
	m_pVisualizer->setInteractor(m_pMainGUI->CVTKWidget->GetInteractor(), m_pMainGUI->CVTKWidget->GetRenderWindow());
}

void CGUI::__connectSignal()
{
	QObject::connect(m_pMainGUI->COpenButton, SIGNAL(clicked(bool)), this, SLOT(__onActionOpen()));
	QObject::connect(m_pMainGUI->CRemoveButton, SIGNAL(clicked(bool)), this, SLOT(__onActionRemove()));
}

void CGUI::__onActionOpen()
{
	QString FilePath = QFileDialog::getOpenFileName(this, tr("Open PointCloud"), QString::fromStdString(m_DirectoryOpenPath), tr("PointCloud Files(*.pcd *.ply *.las);;"));

	m_pVisualizer->unloadModel();
	m_pVisualizer->loadModel(FilePath.toStdString());
	m_pVisualizer->refresh();

	this->__refresh();
}

void CGUI::__onActionRemove()
{
	m_pVisualizer->unloadModel();
}
