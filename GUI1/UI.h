#pragma once
#include <QMainWindow>
#include <QButtonGroup>

namespace Ui
{
	class PCLViewer;
}

class PCLViewer : public QMainWindow
{
	Q_OBJECT

public:
	PCLViewer(int argc, char* argv[], QWidget* vParent = 0);
	~PCLViewer() override;

private Q_SLOTS:
	void __onEmphasizePointStateChanged(int vValue);
	void __onEmphasizeNewPointStateChanged(int vValue);
	void __onDisplayNormalStateChanged(int vValue);
	void __onDisplayPolygonStateChanged(int vValue);
	void __onDisplayNewNormalStateChanged(int vValue);
	void __onDisplayContourStateChanged(int vValue);
	void __onMinContourChanged(int vValue);
	void __onMaxContourChanged(int vValue);
	void __onIntervalChanged(int vValue);

	// Boundary detection
	void __onNeighborChanged(int vValue);
	void __onAngleThresholdChanged(double vValue);
	void __onRadiusChanged(double vValue);
	void __onNormalChanged(double vValue);
	void __onDetectButtonClicked();
	void __onClearBoundaryButtonClicked();

	// Point generation
	void __onScaleFactorChanged(double vValue);
	void __onIterationChanged(int vValue);
	void __onDensityChanged(int vValue);
	void __onGenerateButtonClicked();
	void __onClearNewPointButtonClicked();
	void __onSmoothCorrectionStateChanged(int vValue);
	void __onInwardCorrectionStateChanged(int vValue);
	void __onStepCorrectionStateChanged(int vValue);

	// Disturbance
	void __onFrequencyChanged(double vValue);
	void __onOctaveChanged(int vValue);
	void __onPersistenceChanged(double vValue);
	void __onSeedChanged(int vValue);
	void __onDisturbButtonClicked();
	void __onRevertButtonClicked();

	// Color synthesis
	void __onPickModeChanged();
	void __onSynthesizeButtonClicked();

	void __onDeleteButtonClicked();
	void __onExportButtonClicked();

	// Elevation
	void __onElevationMapButtonClicked();

	// Auto hole Filling
	void __onFillingButtonClicked();

private:
	void __connectSignal();
	void __initWidget();
	void __set2Default();

	std::shared_ptr<visualization::CVisualizer> m_pVisualizer;
	std::shared_ptr<Ui::PCLViewer> m_pUi;
	std::shared_ptr<QButtonGroup> m_pPickModes;

	int m_KNeighbor, m_Iteration, m_Density, m_Octave, m_Seed, m_ElevationMapResolutionX, m_ElevationMapResolutionY, m_MinContour = 0, m_MaxContour = 0, m_Interval = 1;
	double m_AngleThreshold, m_RadiusSearch, m_NormalFactor, m_ScaleFactor, m_Frequency, m_Persistence;
	bool m_SmoothCorrection = true, m_InwardCorrection = true, m_StepCorrection = true;
	std::string m_ElevationMapPath;
};