#include "PCLViewer.h"
#include "ui_PCLViewer.h"
#include <Definition.h>
#include <Registry.h>

PCLViewer::~PCLViewer()
{
	m_pPickModes = nullptr;
	m_pUi = nullptr;
	m_pVisualizer = nullptr;
}

PCLViewer::PCLViewer(int argc, char* argv[], QWidget* vParent)
	: QMainWindow(vParent)
	, m_pVisualizer(new visualization::CVisualizer(argc, argv))
	, m_pUi(new Ui::PCLViewer)
	, m_pPickModes(new QButtonGroup(this))
{
	m_pUi->setupUi(this);
	std::string WindowTitle;
	GET_PROPERTY(definition::WINDOW_TITLE, WindowTitle);
	this->setWindowTitle(WindowTitle.c_str());

	__set2Default();
	__connectSignal();
	__initWidget();

	m_pVisualizer->loadModel();
	m_pVisualizer->refresh();
}

//*****************************************************************
//FUNCTION:
void PCLViewer::__connectSignal()
{
	// Slider to spin box
	connect(m_pUi->NeighborSlider, SIGNAL(valueChanged(int)), m_pUi->NeighborSpinBox, SLOT(setValue(int)));
	connect(m_pUi->AngleThresholdSlider, SIGNAL(doubleValueChanged(double)), m_pUi->AngleThresholdSpinBox, SLOT(setValue(double)));
	connect(m_pUi->RadiusSlider, SIGNAL(doubleValueChanged(double)), m_pUi->RadiusSpinBox, SLOT(setValue(double)));
	connect(m_pUi->NormalSlider, SIGNAL(doubleValueChanged(double)), m_pUi->NormalSpinBox, SLOT(setValue(double)));
	connect(m_pUi->ScaleSlider, SIGNAL(doubleValueChanged(double)), m_pUi->ScaleSpinBox, SLOT(setValue(double)));
	connect(m_pUi->IterationSlider, SIGNAL(valueChanged(int)), m_pUi->IterationSpinBox, SLOT(setValue(int)));
	connect(m_pUi->DensitySlider, SIGNAL(valueChanged(int)), m_pUi->DensitySpinBox, SLOT(setValue(int)));
	connect(m_pUi->FrequencySlider, SIGNAL(doubleValueChanged(double)), m_pUi->FrequencySpinBox, SLOT(setValue(double)));
	connect(m_pUi->OctaveSlider, SIGNAL(valueChanged(int)), m_pUi->OctaveSpinBox, SLOT(setValue(int)));
	connect(m_pUi->PersistenceSlider, SIGNAL(doubleValueChanged(double)), m_pUi->PersistenceSpinBox, SLOT(setValue(double)));

	// Spin box to slider
	connect(m_pUi->NeighborSpinBox, SIGNAL(valueChanged(int)), m_pUi->NeighborSlider, SLOT(setValue(int)));
	connect(m_pUi->AngleThresholdSpinBox, SIGNAL(valueChanged(double)), m_pUi->AngleThresholdSlider, SLOT(setValue(double)));
	connect(m_pUi->RadiusSpinBox, SIGNAL(valueChanged(double)), m_pUi->RadiusSlider, SLOT(setValue(double)));
	connect(m_pUi->NormalSpinBox, SIGNAL(valueChanged(double)), m_pUi->NormalSlider, SLOT(setValue(double)));
	connect(m_pUi->ScaleSpinBox, SIGNAL(valueChanged(double)), m_pUi->ScaleSlider, SLOT(setValue(double)));
	connect(m_pUi->IterationSpinBox, SIGNAL(valueChanged(int)), m_pUi->IterationSlider, SLOT(setValue(int)));
	connect(m_pUi->DensitySpinBox, SIGNAL(valueChanged(int)), m_pUi->DensitySlider, SLOT(setValue(int)));
	connect(m_pUi->FrequencySpinBox, SIGNAL(valueChanged(double)), m_pUi->FrequencySlider, SLOT(setValue(double)));
	connect(m_pUi->OctaveSpinBox, SIGNAL(valueChanged(int)), m_pUi->OctaveSlider, SLOT(setValue(int)));
	connect(m_pUi->PersistenceSpinBox, SIGNAL(valueChanged(double)), m_pUi->PersistenceSlider, SLOT(setValue(double)));

	// Boundary detection
	connect(m_pUi->NeighborSpinBox, SIGNAL(valueChanged(int)), this, SLOT(__onNeighborChanged(int)));
	connect(m_pUi->AngleThresholdSpinBox, SIGNAL(valueChanged(double)), this, SLOT(__onAngleThresholdChanged(double)));
	connect(m_pUi->RadiusSpinBox, SIGNAL(valueChanged(double)), this, SLOT(__onRadiusChanged(double)));
	connect(m_pUi->NormalSpinBox, SIGNAL(valueChanged(double)), this, SLOT(__onNormalChanged(double)));
	connect(m_pUi->DetectButton, SIGNAL(clicked()), this, SLOT(__onDetectButtonClicked()));
	connect(m_pUi->ClearBoundaryButton, SIGNAL(clicked()), this, SLOT(__onClearBoundaryButtonClicked()));

	// Point generation
	connect(m_pUi->ScaleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(__onScaleFactorChanged(double)));
	connect(m_pUi->IterationSpinBox, SIGNAL(valueChanged(int)), this, SLOT(__onIterationChanged(int)));
	connect(m_pUi->DensitySpinBox, SIGNAL(valueChanged(int)), this, SLOT(__onDensityChanged(int)));
	connect(m_pUi->GenerateButton, SIGNAL(clicked()), this, SLOT(__onGenerateButtonClicked()));
	connect(m_pUi->ClearNewPointButton, SIGNAL(clicked()), this, SLOT(__onClearNewPointButtonClicked()));
	connect(m_pUi->SmoothCorrectionCheckBox, SIGNAL(stateChanged(int)), this, SLOT(__onSmoothCorrectionStateChanged(int)));
	connect(m_pUi->InwardCorrectionCheckBox, SIGNAL(stateChanged(int)), this, SLOT(__onInwardCorrectionStateChanged(int)));
	connect(m_pUi->StepCorrectionCheckBox, SIGNAL(stateChanged(int)), this, SLOT(__onStepCorrectionStateChanged(int)));

	// Disturbance
	connect(m_pUi->FrequencySpinBox, SIGNAL(valueChanged(double)), this, SLOT(__onFrequencyChanged(double)));
	connect(m_pUi->OctaveSpinBox, SIGNAL(valueChanged(int)), this, SLOT(__onOctaveChanged(int)));
	connect(m_pUi->PersistenceSpinBox, SIGNAL(valueChanged(double)), this, SLOT(__onPersistenceChanged(double)));
	connect(m_pUi->SeedSpinBox, SIGNAL(valueChanged(int)), this, SLOT(__onSeedChanged(int)));
	connect(m_pUi->DisturbButton, SIGNAL(clicked()), this, SLOT(__onDisturbButtonClicked()));
	connect(m_pUi->RevertButton, SIGNAL(clicked()), this, SLOT(__onRevertButtonClicked()));

	// Color synthesis
	connect(m_pUi->HoleRadioButton, SIGNAL(clicked(bool)), this, SLOT(__onPickModeChanged()));
	connect(m_pUi->ReferenceRadioButton, SIGNAL(clicked(bool)), this, SLOT(__onPickModeChanged()));
	connect(m_pUi->OutlierRadioButton, SIGNAL(clicked(bool)), this, SLOT(__onPickModeChanged()));
	connect(m_pUi->BoundaryRadioButton, SIGNAL(clicked(bool)), this, SLOT(__onPickModeChanged()));
	connect(m_pUi->SynthesizeButton, SIGNAL(clicked()), this, SLOT(__onSynthesizeButtonClicked()));

	connect(m_pUi->DeleteButton, SIGNAL(clicked()), this, SLOT(__onDeleteButtonClicked()));
	connect(m_pUi->ExportButton, SIGNAL(clicked()), this, SLOT(__onExportButtonClicked()));

	// Auxiliary graphics
	connect(m_pUi->EmphasizePointCheckBox, SIGNAL(stateChanged(int)), this, SLOT(__onEmphasizePointStateChanged(int)));
	connect(m_pUi->EmphasizeNewPointCheckBox, SIGNAL(stateChanged(int)), this, SLOT(__onEmphasizeNewPointStateChanged(int)));
	connect(m_pUi->DisplayNormalCheckBox, SIGNAL(stateChanged(int)), this, SLOT(__onDisplayNormalStateChanged(int)));
	connect(m_pUi->DisplayPolygonCheckBox, SIGNAL(stateChanged(int)), this, SLOT(__onDisplayPolygonStateChanged(int)));
	connect(m_pUi->DisplayNewNormalCheckBox, SIGNAL(stateChanged(int)), this, SLOT(__onDisplayNewNormalStateChanged(int)));
	connect(m_pUi->DisplayContourCheckBox, SIGNAL(stateChanged(int)), this, SLOT(__onDisplayContourStateChanged(int)));
	connect(m_pUi->MinContourSpinBox, SIGNAL(valueChanged(int)), this, SLOT(__onMinContourChanged(int)));
	connect(m_pUi->MaxContourSpinBox, SIGNAL(valueChanged(int)), this, SLOT(__onMaxContourChanged(int)));
	connect(m_pUi->IntervalSpinBox, SIGNAL(valueChanged(int)), this, SLOT(__onIntervalChanged(int)));

	// ElevationMap
	connect(m_pUi->ElevationMapButton, SIGNAL(clicked()), this, SLOT(__onElevationMapButtonClicked()));

	// Auto hole Filling
	connect(m_pUi->FillingButton, SIGNAL(clicked()), this, SLOT(__onFillingButtonClicked()));
	connect(m_pUi->FillingEmphasizeCheckBox, SIGNAL(stateChanged(int)), this, SLOT(__onEmphasizeNewPointStateChanged(int)));
}

//*****************************************************************
//FUNCTION:
void PCLViewer::__initWidget()
{
	m_pUi->DisplayPanel->SetRenderWindow(m_pVisualizer->getRenderWindow());
	m_pVisualizer->setInteractor(m_pUi->DisplayPanel->GetInteractor(), m_pUi->DisplayPanel->GetRenderWindow());
}

//*****************************************************************
//FUNCTION:
void PCLViewer::__set2Default()
{
	GET_PROPERTY(definition::K_NEIGHBOR, m_KNeighbor);
	GET_PROPERTY(definition::ANGLE_THRESHOLD, m_AngleThreshold);
	GET_PROPERTY(definition::RADIUS_SEARCH, m_RadiusSearch);
	GET_PROPERTY(definition::NORMAL_FACTOR, m_NormalFactor);
	GET_PROPERTY(definition::SCALE_FACTOR, m_ScaleFactor);
	GET_PROPERTY(definition::ITERATION, m_Iteration);
	GET_PROPERTY(definition::DENSITY, m_Density);
	GET_PROPERTY(definition::FREQUENCY, m_Frequency);
	GET_PROPERTY(definition::OCTAVE, m_Octave);
	GET_PROPERTY(definition::PERSISTENCE, m_Persistence);
	GET_PROPERTY(definition::SEED, m_Seed);
	GET_PROPERTY(definition::ELEVATIONMAPPATH, m_ElevationMapPath);
	GET_PROPERTY(definition::ELEVATIONMAPRESOLUTIONX, m_ElevationMapResolutionX);
	GET_PROPERTY(definition::ELEVATIONMAPRESOLUTIONY, m_ElevationMapResolutionY);

	m_pUi->NeighborSpinBox->setValue(m_KNeighbor);
	m_pUi->AngleThresholdSpinBox->setValue(m_AngleThreshold);
	m_pUi->RadiusSpinBox->setValue(m_RadiusSearch);
	m_pUi->NormalSpinBox->setValue(m_NormalFactor);
	m_pUi->ScaleSpinBox->setValue(m_ScaleFactor);
	m_pUi->IterationSpinBox->setValue(m_Iteration);
	m_pUi->DensitySpinBox->setValue(m_Density);
	m_pUi->FrequencySpinBox->setValue(m_Frequency);
	m_pUi->OctaveSpinBox->setValue(m_Octave);
	m_pUi->PersistenceSpinBox->setValue(m_Persistence);
	m_pUi->SeedSpinBox->setValue(m_Seed);

	m_pUi->NeighborSlider->setValue(m_KNeighbor);
	m_pUi->AngleThresholdSlider->setValue(m_AngleThreshold);
	m_pUi->RadiusSlider->setValue(m_RadiusSearch);
	m_pUi->NormalSlider->setValue(m_NormalFactor);
	m_pUi->ScaleSlider->setValue(m_ScaleFactor);
	m_pUi->IterationSlider->setValue(m_Iteration);
	m_pUi->DensitySlider->setValue(m_Density);
	m_pUi->FrequencySlider->setValue(m_Frequency);
	m_pUi->OctaveSlider->setValue(m_Octave);
	m_pUi->PersistenceSlider->setValue(m_Persistence);

	m_pPickModes->addButton(m_pUi->HoleRadioButton, static_cast<int>(visualization::EPickMode::Hole));
	m_pPickModes->addButton(m_pUi->ReferenceRadioButton, static_cast<int>(visualization::EPickMode::Reference));
	m_pPickModes->addButton(m_pUi->OutlierRadioButton, static_cast<int>(visualization::EPickMode::Outlier));
	m_pPickModes->addButton(m_pUi->BoundaryRadioButton, static_cast<int>(visualization::EPickMode::Boundary));
	m_pUi->HoleRadioButton->setChecked(true);
}

//*****************************************************************
//FUNCTION:
void PCLViewer::__onDetectButtonClicked()
{
	m_pVisualizer->detectBoundary(m_KNeighbor, m_AngleThreshold, m_RadiusSearch, m_NormalFactor);
	m_pVisualizer->refresh();
}

//*****************************************************************
//FUNCTION:
void PCLViewer::__onNeighborChanged(int vValue)
{
	m_KNeighbor = vValue;
	m_pVisualizer->detectBoundary(m_KNeighbor, m_AngleThreshold, m_RadiusSearch, m_NormalFactor);
	m_pVisualizer->refresh();
}

//*****************************************************************
//FUNCTION:
void PCLViewer::__onAngleThresholdChanged(double vValue)
{
	m_AngleThreshold = vValue;
	m_pVisualizer->detectBoundary(m_KNeighbor, m_AngleThreshold, m_RadiusSearch, m_NormalFactor);
	m_pVisualizer->refresh();
}

//*****************************************************************
//FUNCTION:
void PCLViewer::__onRadiusChanged(double vValue)
{
	m_RadiusSearch = vValue;
	m_pVisualizer->detectBoundary(m_KNeighbor, m_AngleThreshold, m_RadiusSearch, m_NormalFactor);
	m_pVisualizer->refresh();
}

//*****************************************************************
//FUNCTION:
void PCLViewer::__onNormalChanged(double vValue)
{
	m_NormalFactor = vValue;
	m_pVisualizer->correctNormal(m_NormalFactor);
	m_pVisualizer->refresh();
}

//*****************************************************************
//FUNCTION:
void PCLViewer::__onClearBoundaryButtonClicked()
{
	m_pVisualizer->clearAll(true);
	m_pVisualizer->refresh();
}

//*****************************************************************
//FUNCTION:
void PCLViewer::__onScaleFactorChanged(double vValue)
{
	m_ScaleFactor = vValue;
	m_pVisualizer->generatePoints(m_ScaleFactor, m_Iteration, m_Density);
	m_pVisualizer->refresh();
}

//*****************************************************************
//FUNCTION:
void PCLViewer::__onIterationChanged(int vValue)
{
	m_Iteration = vValue;
	m_pVisualizer->generatePoints(m_ScaleFactor, m_Iteration, m_Density);
	m_pVisualizer->refresh();
}

//*****************************************************************
//FUNCTION:
void PCLViewer::__onDensityChanged(int vValue)
{
	m_Density = vValue;
	m_pVisualizer->generatePoints(m_ScaleFactor, m_Iteration, m_Density);
	m_pVisualizer->refresh();
}

//*****************************************************************
//FUNCTION:
void PCLViewer::__onGenerateButtonClicked()
{
	m_pVisualizer->generatePoints(m_ScaleFactor, m_Iteration, m_Density);
	m_pVisualizer->refresh();
}

//*****************************************************************
//FUNCTION:
void PCLViewer::__onClearNewPointButtonClicked()
{
	m_pVisualizer->clearNewCloudAll(true);
	m_pVisualizer->refresh();
}

//*****************************************************************
//FUNCTION:
void PCLViewer::__onFrequencyChanged(double vValue)
{
	m_Frequency = vValue;
	m_pVisualizer->disturb(m_Frequency, m_Octave, m_Persistence, m_Seed);
	m_pVisualizer->refresh();
}

//*****************************************************************
//FUNCTION:
void PCLViewer::__onOctaveChanged(int vValue)
{
	m_Octave = vValue;
	m_pVisualizer->disturb(m_Frequency, m_Octave, m_Persistence, m_Seed);
	m_pVisualizer->refresh();
}

//*****************************************************************
//FUNCTION:
void PCLViewer::__onPersistenceChanged(double vValue)
{
	m_Persistence = vValue;
	m_pVisualizer->disturb(m_Frequency, m_Octave, m_Persistence, m_Seed);
	m_pVisualizer->refresh();
}

//*****************************************************************
//FUNCTION:
void PCLViewer::__onSeedChanged(int vValue)
{
	m_Seed = vValue;
	m_pVisualizer->disturb(m_Frequency, m_Octave, m_Persistence, m_Seed);
	m_pVisualizer->refresh();
}

//*****************************************************************
//FUNCTION:
void PCLViewer::__onDisturbButtonClicked()
{
	m_pVisualizer->disturb(m_Frequency, m_Octave, m_Persistence, m_Seed);
	m_pVisualizer->refresh();
}

//*****************************************************************
//FUNCTION:
void PCLViewer::__onRevertButtonClicked()
{
	m_pVisualizer->revert();
	m_pVisualizer->refresh();
}

//*****************************************************************
//FUNCTION:
void PCLViewer::__onEmphasizePointStateChanged(int vValue)
{
	const bool Emphasize = (vValue != Qt::Unchecked);
	m_pVisualizer->clearBoundary(false);
	m_pVisualizer->displayBoundary(Emphasize);
	m_pVisualizer->refresh();
}

//*****************************************************************
//FUNCTION:
void PCLViewer::__onEmphasizeNewPointStateChanged(int vValue)
{
	const bool Emphasize = (vValue != Qt::Unchecked);
	m_pVisualizer->clearNewCloud(false);
	m_pVisualizer->displayNewCloud(Emphasize);
	m_pVisualizer->refresh();
}

//*****************************************************************
//FUNCTION:
void PCLViewer::__onDisplayNormalStateChanged(int vValue)
{
	const bool DisplayNormal = (vValue != Qt::Unchecked);
	m_pVisualizer->displayNormal(DisplayNormal);
	m_pVisualizer->refresh();
}

//*****************************************************************
//FUNCTION:
void PCLViewer::__onDisplayPolygonStateChanged(int vValue)
{
	const bool DisplayPolygon = (vValue != Qt::Unchecked);
	m_pVisualizer->displayPolygon(DisplayPolygon);
	m_pVisualizer->refresh();
}

//*****************************************************************
//FUNCTION:
void PCLViewer::__onDisplayNewNormalStateChanged(int vValue)
{
	const bool DisplayNormal = (vValue != Qt::Unchecked);
	m_pVisualizer->displayNewNormal(DisplayNormal);
	m_pVisualizer->refresh();
}

//*****************************************************************
//FUNCTION:
void PCLViewer::__onDisplayContourStateChanged(int vValue)
{
	const bool DisplayContour = (vValue != Qt::Unchecked);
	m_pVisualizer->displayContours(DisplayContour);
	m_pVisualizer->refresh();
}

//*****************************************************************
//FUNCTION:
void PCLViewer::__onPickModeChanged()
{
	const visualization::EPickMode PickMode = static_cast<visualization::EPickMode>(m_pPickModes->checkedId());
	m_pVisualizer->setPickMode(PickMode);
}

//*****************************************************************
//FUNCTION:
void PCLViewer::__onSynthesizeButtonClicked()
{
	m_pVisualizer->synthesizeColor();
	m_pVisualizer->refresh();
}

//*****************************************************************
//FUNCTION:
void PCLViewer::__onDeleteButtonClicked()
{
	m_pVisualizer->deletePoints();
	m_pVisualizer->refresh();
}

//*****************************************************************
//FUNCTION:
void PCLViewer::__onElevationMapButtonClicked()
{
	m_pVisualizer->generateElevationMap(m_ElevationMapResolutionX, m_ElevationMapResolutionY, m_ElevationMapPath);
}

//*****************************************************************
//FUNCTION:
void PCLViewer::__onFillingButtonClicked()
{
	m_pVisualizer->autoHoleFilling(m_KNeighbor, m_AngleThreshold, m_RadiusSearch, m_NormalFactor, m_ScaleFactor, m_Iteration, m_Density);
}

//*****************************************************************
//FUNCTION:
void PCLViewer::__onSmoothCorrectionStateChanged(int vValue)
{
	m_SmoothCorrection = (vValue != Qt::Unchecked);
	m_pVisualizer->generatePoints(m_ScaleFactor, m_Iteration, m_Density, m_SmoothCorrection, m_InwardCorrection, m_StepCorrection);
	m_pVisualizer->refresh();
}

//*****************************************************************
//FUNCTION:
void PCLViewer::__onInwardCorrectionStateChanged(int vValue)
{
	m_InwardCorrection = (vValue != Qt::Unchecked);
	m_pVisualizer->generatePoints(m_ScaleFactor, m_Iteration, m_Density, m_SmoothCorrection, m_InwardCorrection, m_StepCorrection);
	m_pVisualizer->refresh();
}

//*****************************************************************
//FUNCTION:
void PCLViewer::__onStepCorrectionStateChanged(int vValue)
{
	m_StepCorrection = (vValue != Qt::Unchecked);
	m_pVisualizer->generatePoints(m_ScaleFactor, m_Iteration, m_Density, m_SmoothCorrection, m_InwardCorrection, m_StepCorrection);
	m_pVisualizer->refresh();
}

//*****************************************************************
//FUNCTION:
void PCLViewer::__onExportButtonClicked()
{
	m_pVisualizer->exportModel();
}

//*****************************************************************
//FUNCTION:
void PCLViewer::__onMinContourChanged(int vValue)
{
	m_MinContour = vValue;
	m_pVisualizer->clearContours();
	m_pVisualizer->displayContours(true, m_MinContour, m_MaxContour, m_Interval);
	m_pVisualizer->refresh();
}

//*****************************************************************
//FUNCTION:
void PCLViewer::__onMaxContourChanged(int vValue)
{
	m_MaxContour = vValue;
	m_pVisualizer->clearContours();
	m_pVisualizer->displayContours(true, m_MinContour, m_MaxContour, m_Interval);
	m_pVisualizer->refresh();
}

//*****************************************************************
//FUNCTION:
void PCLViewer::__onIntervalChanged(int vValue)
{
	m_Interval = vValue;
	m_pVisualizer->clearContours();
	m_pVisualizer->displayContours(true, m_MinContour, m_MaxContour, m_Interval);
	m_pVisualizer->refresh();
}