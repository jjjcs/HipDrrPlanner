#ifndef MainWindow_H
#define MainWindow_H

#include <QMainWindow>
#include <memory>

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow
{
	Q_OBJECT
public:
	MainWindow();
	~MainWindow();
private slots:
	void SlotLogNormalization();
	void SlotTruncationCorrection();
	void SlotTruncationCorrection_cuda();
	void SlotFDKReconstruction();
	void SlotParkerWeighting();
	void SlotParkerWeighting_cuda();
private:
	void LogNormalization();
	void TruncationCorrection();
	void TruncationCorrection_cuda();
	void FDKReconstruction();
	void ParkerWeighting();
	void ParkerWeighting_cuda();
private:
	Ui::MainWindow* m_Ui;
};

#endif 